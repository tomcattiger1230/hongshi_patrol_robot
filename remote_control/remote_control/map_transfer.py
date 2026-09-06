"""Map-session transfer helpers for the macOS remote GUI."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import hashlib
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import tempfile
from typing import Callable, Sequence
import uuid


_SAFE_NAME = re.compile(r"[^A-Za-z0-9._-]+")
_SAFE_REMOTE_DIRECTORY = re.compile(r"^(?:~?/)?[A-Za-z0-9._/-]+$")


@dataclass(frozen=True)
class UploadedMapSession:
    remote_prefix: str
    mode: str


def safe_map_name(value: str) -> str:
    """Return a portable basename suitable for both SCP and the robot."""
    name = _SAFE_NAME.sub("_", value.strip()).strip("._-")
    if not name:
        digest = hashlib.sha256(value.encode("utf-8")).hexdigest()[:12]
        name = f"map-{digest}"
    return name[:80]


def validate_remote_map_directory(value: str) -> str:
    directory = value.strip().rstrip("/")
    if (
        not directory
        or not _SAFE_REMOTE_DIRECTORY.fullmatch(directory)
        or ".." in PurePosixPath(directory).parts
    ):
        raise ValueError("远端地图目录只能包含安全路径字符，且不能包含 '..'")
    return directory


def map_image_path(yaml_path: str | Path) -> Path:
    """Resolve the image referenced by a ROS map YAML."""
    path = Path(yaml_path).expanduser().resolve()
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip() != "image":
            continue
        image = value.strip()
        if image[:1] in {"'", '"'}:
            image = str(ast.literal_eval(image))
        result = Path(image).expanduser()
        return result.resolve() if result.is_absolute() else (path.parent / result).resolve()
    raise ValueError("地图 YAML 缺少 image 字段")


class SshMapSessionTransfer:
    """Move map artifacts over the user's existing SSH key connection."""

    def __init__(
        self,
        ssh_target: str,
        remote_map_directory: str = "~/robot320_maps",
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ) -> None:
        if not ssh_target.strip() or ssh_target.startswith("-"):
            raise ValueError("SSH 目标不能为空")
        self.ssh_target = ssh_target.strip()
        self.remote_map_directory = validate_remote_map_directory(remote_map_directory)
        self._transfer_directory = (
            self.remote_map_directory[2:]
            if self.remote_map_directory.startswith("~/")
            else self.remote_map_directory
        )
        self._run = runner

    def remote_prefix(self, name: str) -> str:
        return f"{self.remote_map_directory}/{safe_map_name(name)}"

    def upload(self, yaml_path: str | Path) -> UploadedMapSession:
        yaml_file = Path(yaml_path).expanduser().resolve()
        image_file = map_image_path(yaml_file)
        posegraph = yaml_file.with_suffix(".posegraph")
        data = yaml_file.with_suffix(".data")
        has_posegraph = posegraph.is_file() and data.is_file()
        if posegraph.is_file() != data.is_file():
            raise ValueError("持续建图会话必须同时包含 .posegraph 和 .data")
        required = [yaml_file, image_file]
        if has_posegraph:
            required.extend((posegraph, data))
        for path in required:
            if not path.is_file():
                raise FileNotFoundError(path)

        name = safe_map_name(yaml_file.stem)
        prefix = self.remote_prefix(name)
        transfer_prefix = f"{self._transfer_directory}/{name}"
        remote_stage = (
            f"{self._transfer_directory}/.incoming-{name}-{uuid.uuid4().hex[:8]}"
        )
        self._ssh(["mkdir", "-p", "--", remote_stage])
        try:
            with tempfile.TemporaryDirectory(prefix="robot320-upload-") as temporary:
                upload_dir = Path(temporary)
                normalized_yaml = upload_dir / f"{name}.yaml"
                normalized_image = upload_dir / f"{name}{image_file.suffix.lower()}"
                shutil.copy2(yaml_file, normalized_yaml)
                shutil.copy2(image_file, normalized_image)
                _rewrite_yaml_image(normalized_yaml, normalized_image.name)
                upload_files = [normalized_yaml, normalized_image]
                if has_posegraph:
                    normalized_posegraph = upload_dir / f"{name}.posegraph"
                    normalized_data = upload_dir / f"{name}.data"
                    shutil.copy2(posegraph, normalized_posegraph)
                    shutil.copy2(data, normalized_data)
                    upload_files.extend((normalized_posegraph, normalized_data))
                self._scp_to(upload_files, remote_stage)
            remote_image = PurePosixPath(f"{name}{image_file.suffix.lower()}")
            remote_yaml = PurePosixPath(f"{name}.yaml")
            moves = [
                (remote_stage, remote_yaml, f"{transfer_prefix}.yaml"),
                (
                    remote_stage,
                    remote_image,
                    f"{transfer_prefix}{image_file.suffix.lower()}",
                ),
            ]
            if has_posegraph:
                moves.extend(
                    (
                        (
                            remote_stage,
                            PurePosixPath(f"{name}.posegraph"),
                            f"{transfer_prefix}.posegraph",
                        ),
                        (
                            remote_stage,
                            PurePosixPath(f"{name}.data"),
                            f"{transfer_prefix}.data",
                        ),
                    )
                )
            commands = [
                f"mv -- {shlex.quote(str(PurePosixPath(stage) / source))} {shlex.quote(target)}"
                for stage, source, target in moves
            ]
            self._ssh_shell(" && ".join(commands))
        finally:
            self._ssh(["rm", "-rf", "--", remote_stage], check=False)
        return UploadedMapSession(prefix, "continuing" if has_posegraph else "localization")

    def download(self, remote_prefix: str, local_yaml_path: str | Path) -> tuple[Path, ...]:
        local_yaml = Path(local_yaml_path).expanduser().resolve()
        if local_yaml.suffix.lower() not in {".yaml", ".yml"}:
            local_yaml = local_yaml.with_suffix(".yaml")
        local_yaml.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="robot320-map-") as temporary:
            temporary_path = Path(temporary)
            suffixes = (".yaml", ".pgm", ".posegraph", ".data")
            self._scp_from([f"{remote_prefix}{suffix}" for suffix in suffixes], temporary_path)
            remote_name = PurePosixPath(remote_prefix).name
            outputs = []
            for suffix in suffixes:
                source = temporary_path / f"{remote_name}{suffix}"
                destination = local_yaml.with_suffix(suffix)
                shutil.move(str(source), destination)
                outputs.append(destination)
            _rewrite_yaml_image(outputs[0], outputs[1].name)
            return tuple(outputs)

    def _ssh(self, arguments: Sequence[str], check: bool = True) -> None:
        command = " ".join(shlex.quote(str(value)) for value in arguments)
        self._ssh_shell(command, check=check)

    def _ssh_shell(self, command: str, check: bool = True) -> None:
        self._run(
            [
                "ssh",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=8",
                self.ssh_target,
                command,
            ],
            check=check,
            capture_output=True,
            text=True,
            timeout=60,
        )

    def _scp_to(self, sources: Sequence[Path], remote_directory: str) -> None:
        self._run(
            [
                "scp",
                "-q",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=8",
                *(str(path) for path in sources),
                f"{self.ssh_target}:{remote_directory}/",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=120,
        )

    def _scp_from(self, sources: Sequence[str], destination: Path) -> None:
        self._run(
            [
                "scp",
                "-q",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=8",
                *(f"{self.ssh_target}:{source}" for source in sources),
                str(destination),
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=120,
        )


def _rewrite_yaml_image(yaml_path: Path, image_name: str) -> None:
    lines = yaml_path.read_text(encoding="utf-8").splitlines()
    replaced = False
    for index, line in enumerate(lines):
        if line.split("#", 1)[0].strip().startswith("image:"):
            lines[index] = f"image: {image_name}"
            replaced = True
            break
    if not replaced:
        raise ValueError("下载的地图 YAML 缺少 image 字段")
    yaml_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
