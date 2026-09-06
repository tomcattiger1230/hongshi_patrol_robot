from pathlib import Path
import subprocess

import pytest

from remote_control.map_transfer import (
    SshMapSessionTransfer,
    map_image_path,
    safe_map_name,
    validate_remote_map_directory,
)


def _write_session(directory: Path, complete: bool = True) -> Path:
    yaml_path = directory / "车间 A.yaml"
    (directory / "floor image.pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
    yaml_path.write_text(
        "image: 'floor image.pgm'\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    if complete:
        yaml_path.with_suffix(".posegraph").write_bytes(b"posegraph")
        yaml_path.with_suffix(".data").write_bytes(b"data")
    return yaml_path


def test_map_transfer_validates_names_and_resolves_yaml_image(tmp_path):
    yaml_path = _write_session(tmp_path)

    assert safe_map_name("车间 A") == "A"
    assert safe_map_name("一楼地图").startswith("map-")
    assert map_image_path(yaml_path) == tmp_path / "floor image.pgm"
    assert validate_remote_map_directory("~/robot320_maps") == "~/robot320_maps"
    with pytest.raises(ValueError):
        validate_remote_map_directory("~/robot320_maps/../escape")


def test_upload_normalizes_names_and_detects_complete_session(tmp_path):
    yaml_path = _write_session(tmp_path)
    calls = []

    def runner(arguments, **kwargs):
        calls.append((arguments, kwargs))
        if arguments[0] == "scp":
            uploaded = [Path(item) for item in arguments[6:-1]]
            assert {item.name for item in uploaded} == {
                "A.yaml",
                "A.pgm",
                "A.posegraph",
                "A.data",
            }
            assert "image: A.pgm" in uploaded[0].read_text(encoding="utf-8")
        return subprocess.CompletedProcess(arguments, 0, "", "")

    transfer = SshMapSessionTransfer("robot@example", runner=runner)
    result = transfer.upload(yaml_path)

    assert result.remote_prefix == "~/robot320_maps/A"
    assert result.mode == "continuing"
    assert [call[0][0] for call in calls] == ["ssh", "scp", "ssh", "ssh"]


def test_upload_rejects_incomplete_posegraph_pair(tmp_path):
    yaml_path = _write_session(tmp_path, complete=False)
    yaml_path.with_suffix(".posegraph").write_bytes(b"posegraph")
    transfer = SshMapSessionTransfer("robot@example")

    with pytest.raises(ValueError, match="同时包含"):
        transfer.upload(yaml_path)
