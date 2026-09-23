#!/usr/bin/env python3
# ruff: noqa: F403,F405
"""Manage NUC-local recording for the two Hikrobot USB cameras."""

from __future__ import annotations

import argparse
import base64
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof

os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
sys.path.insert(0, "/opt/MVS/Samples/64/Python/MvImport")
from MvCameraControl_class import *  # noqa: E402,F403


CAMERAS = {
    "right": {"serial": "DB0168357", "label": "右相机"},
    "left": {"serial": "DB0168290", "label": "左相机"},
}
RESOLUTIONS = {(4096, 2460), (2048, 1230), (1280, 768), (1024, 615)}
FPS_VALUES = {1, 2, 5, 10}
ROOT = Path.home() / "haikang_records"
STATE = ROOT / ".active.json"
CONTROL_LOCK = ROOT / ".control.lock"
SESSION_RE = re.compile(r"^\d{8}_\d{6}(?:_\d{2})?$")
STOP = False


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def read_json(path: Path, default=None):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else default
    except (OSError, ValueError):
        return default


def process_alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, TypeError, ValueError):
        return False


def active_status(include_preview=False) -> dict:
    state = read_json(STATE, {}) or {}
    session_name = state.get("session", "")
    session = ROOT / session_name if SESSION_RE.fullmatch(session_name) else None
    workers = []
    for camera, worker in (state.get("workers") or {}).items():
        if camera not in CAMERAS or not isinstance(worker, dict):
            continue
        worker_status = read_json(session / f".{camera}.json", {}) if session else {}
        alive = process_alive(worker.get("pid"))
        item = {
            "camera": camera,
            "label": CAMERAS[camera]["label"],
            "pid": worker.get("pid"),
            "alive": alive,
            "state": worker_status.get("state", "starting" if alive else "finished"),
            "frames": int(worker_status.get("frames", 0)),
            "elapsed": float(worker_status.get("elapsed", 0.0)),
            "error": str(worker_status.get("error", ""))[-300:],
        }
        preview = session / f"preview_{camera}.jpg" if session else None
        if include_preview and preview and preview.is_file() and preview.stat().st_size <= 1024 * 1024:
            item["preview_jpeg"] = base64.b64encode(preview.read_bytes()).decode("ascii")
        workers.append(item)
    recording = any(item["alive"] for item in workers)
    result = {
        "recording": recording,
        "session": session_name if session else "",
        "directory": str(session) if session else "",
        "started_at": state.get("started_at"),
        "config": state.get("config", {}),
        "workers": workers,
    }
    return result


def unique_session() -> Path:
    ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    stem = time.strftime("%Y%m%d_%H%M%S")
    candidate = ROOT / stem
    suffix = 1
    while candidate.exists():
        candidate = ROOT / f"{stem}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(mode=0o700)
    return candidate


def start_recording(args) -> dict:
    if args.fps not in FPS_VALUES or (args.width, args.height) not in RESOLUTIONS:
        raise ValueError("unsupported fps or resolution")
    if not 5 <= args.duration <= 28800:
        raise ValueError("duration must be between 5 and 28800 seconds")
    ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    with CONTROL_LOCK.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if active_status()["recording"]:
            raise RuntimeError("a recording session is already active")
        if shutil.disk_usage(ROOT).free < 5 * 1024**3:
            raise RuntimeError("less than 5 GiB free; recording refused")
        session = unique_session()
        cameras = list(CAMERAS) if args.camera == "both" else [args.camera]
        config = {
            "camera": args.camera,
            "fps": args.fps,
            "width": args.width,
            "height": args.height,
            "duration": args.duration,
        }
        state = {
            "session": session.name,
            "started_at": time.time(),
            "config": config,
            "workers": {},
        }
        atomic_json(session / "metadata.json", {**state, "camera_serials": CAMERAS})
        for camera in cameras:
            log = (session / f"{camera}.log").open("ab", buffering=0)
            command = [
                sys.executable, str(Path(__file__).resolve()), "worker",
                "--camera", camera, "--fps", str(args.fps),
                "--width", str(args.width), "--height", str(args.height),
                "--duration", str(args.duration), "--session", session.name,
            ]
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True,
            )
            log.close()
            state["workers"][camera] = {"pid": process.pid}
        atomic_json(STATE, state)
    return active_status()


def stop_recording() -> dict:
    ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    with CONTROL_LOCK.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        state = read_json(STATE, {}) or {}
        pids = [worker.get("pid") for worker in (state.get("workers") or {}).values()]
        for pid in pids:
            if process_alive(pid):
                try:
                    os.killpg(int(pid), signal.SIGTERM)
                except OSError:
                    pass
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and any(process_alive(pid) for pid in pids):
            time.sleep(0.1)
        for pid in pids:
            if process_alive(pid):
                try:
                    os.killpg(int(pid), signal.SIGKILL)
                except OSError:
                    pass
    return active_status()


def list_sessions() -> dict:
    ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
    sessions = []
    for directory in sorted(ROOT.iterdir(), reverse=True):
        if not directory.is_dir() or not SESSION_RE.fullmatch(directory.name):
            continue
        metadata = read_json(directory / "metadata.json", {}) or {}
        files = []
        total = 0
        for path in sorted(directory.glob("*.mp4")):
            size = path.stat().st_size
            files.append({"name": path.name, "bytes": size})
            total += size
        sessions.append({
            "name": directory.name,
            "bytes": total,
            "files": files,
            "config": metadata.get("config", {}),
        })
    return {"root": str(ROOT), "sessions": sessions[:200]}


def decode(value):
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def find_device(serial):
    result = MV_CC_DEVICE_INFO_LIST()
    code = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, result)
    if code != MV_OK:
        raise RuntimeError(f"camera enumeration failed: 0x{code:x}")
    for index in range(result.nDeviceNum):
        info = cast(result.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents
        if decode(info.SpecialInfo.stUsb3VInfo.chSerialNumber) == serial:
            return info
    raise RuntimeError(f"camera serial {serial} unavailable")


def encode_jpeg(cam, frame, quality=80):
    output_size = max(frame.stFrameInfo.nWidth * frame.stFrameInfo.nHeight * 3 + 4096, 1024 * 1024)
    output = (c_ubyte * output_size)()
    params = MV_SAVE_IMAGE_PARAM_EX3()
    memset(byref(params), 0, sizeof(params))
    params.pData = frame.pBufAddr
    params.nDataLen = frame.stFrameInfo.nFrameLen
    params.enPixelType = frame.stFrameInfo.enPixelType
    params.nWidth = frame.stFrameInfo.nWidth
    params.nHeight = frame.stFrameInfo.nHeight
    params.pImageBuffer = output
    params.nBufferSize = output_size
    params.enImageType = MV_Image_Jpeg
    params.nJpgQuality = quality
    params.iMethodValue = 1
    code = cam.MV_CC_SaveImageEx3(params)
    if code != MV_OK or not params.nImageLen:
        raise RuntimeError(f"JPEG encoding failed: 0x{code:x}")
    return bytes(output[:params.nImageLen])


def worker(args) -> None:
    global STOP
    session = ROOT / args.session
    if not SESSION_RE.fullmatch(args.session) or not session.is_dir():
        raise ValueError("invalid session")
    camera = args.camera
    status_path = session / f".{camera}.json"
    output_path = session / f"{camera}.mp4"

    def handle_stop(_signal, _frame):
        global STOP
        STOP = True

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)
    cam = MvCamera()
    frame = MV_FRAME_OUT()
    created = opened = grabbing = False
    ffmpeg = None
    frames = 0
    started = time.monotonic()
    try:
        atomic_json(status_path, {"state": "warming", "frames": 0, "elapsed": 0})
        MvCamera.MV_CC_Initialize()
        info = find_device(CAMERAS[camera]["serial"])
        code = cam.MV_CC_CreateHandle(info)
        if code != MV_OK:
            raise RuntimeError(f"create handle failed: 0x{code:x}")
        created = True
        code = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if code != MV_OK:
            raise RuntimeError(f"open camera failed: 0x{code:x}")
        opened = True
        cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        cam.MV_CC_SetIntValue("AutoExposureTimeUpperLimit", 200000)
        cam.MV_CC_SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS)
        cam.MV_CC_SetEnumValue("GainAuto", 2)
        code = cam.MV_CC_StartGrabbing()
        if code != MV_OK:
            raise RuntimeError(f"start capture failed: 0x{code:x}")
        grabbing = True
        ffmpeg = subprocess.Popen([
            "/usr/bin/ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-framerate", str(args.fps),
            "-i", "pipe:0", "-vf", f"scale={args.width}:{args.height}",
            "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_path),
        ], stdin=subprocess.PIPE)
        warm_until = time.monotonic() + 5.0
        next_frame = warm_until
        last_preview = 0.0
        record_started = None
        last_jpeg = None
        completed = False
        while not STOP:
            memset(byref(frame), 0, sizeof(frame))
            code = cam.MV_CC_GetImageBuffer(frame, 2000)
            if code != MV_OK or not frame.pBufAddr:
                if STOP:
                    break
                raise RuntimeError(f"frame timeout: 0x{code:x}")
            now = time.monotonic()
            try:
                if now < next_frame:
                    continue
                if record_started is None:
                    record_started = now
                    atomic_json(status_path, {"state": "recording", "frames": 0, "elapsed": 0})
                elapsed = now - record_started
                if elapsed >= args.duration:
                    completed = True
                    break
                jpeg = encode_jpeg(cam, frame)
                last_jpeg = jpeg
                # If acquisition is slower than the selected output rate,
                # duplicate the newest image to keep MP4 timing honest.
                due = max(1, min(args.fps * 2, int((now - next_frame) * args.fps) + 1))
                for _ in range(due):
                    ffmpeg.stdin.write(jpeg)
                    frames += 1
                    next_frame += 1.0 / args.fps
                if now - last_preview >= 1.0:
                    import cv2
                    import numpy as np
                    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is not None:
                        preview_width = min(960, image.shape[1])
                        preview_height = round(image.shape[0] * preview_width / image.shape[1])
                        preview = cv2.resize(image, (preview_width, preview_height))
                        ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 65])
                        if ok:
                            temporary = session / f".preview_{camera}.tmp"
                            temporary.write_bytes(encoded.tobytes())
                            temporary.replace(session / f"preview_{camera}.jpg")
                    last_preview = now
                atomic_json(status_path, {"state": "recording", "frames": frames, "elapsed": elapsed})
            finally:
                cam.MV_CC_FreeImageBuffer(frame)
                memset(byref(frame), 0, sizeof(frame))
        if completed and last_jpeg is not None:
            for _ in range(max(0, args.duration * args.fps - frames)):
                ffmpeg.stdin.write(last_jpeg)
                frames += 1
        atomic_json(status_path, {
            "state": "stopped" if STOP else "complete", "frames": frames,
            "elapsed": max(0.0, (time.monotonic() - record_started) if record_started else 0.0),
        })
    except Exception as exc:
        atomic_json(status_path, {"state": "error", "frames": frames,
                                  "elapsed": time.monotonic() - started, "error": str(exc)})
        raise
    finally:
        if frame.pBufAddr:
            cam.MV_CC_FreeImageBuffer(frame)
        if grabbing:
            cam.MV_CC_StopGrabbing()
        if opened:
            cam.MV_CC_CloseDevice()
        if created:
            cam.MV_CC_DestroyHandle()
        MvCamera.MV_CC_Finalize()
        if ffmpeg is not None:
            if ffmpeg.stdin:
                try:
                    ffmpeg.stdin.close()
                except OSError:
                    pass
            try:
                ffmpeg.wait(timeout=10)
            except subprocess.TimeoutExpired:
                ffmpeg.kill()


def parser():
    result = argparse.ArgumentParser()
    commands = result.add_subparsers(dest="command", required=True)
    start = commands.add_parser("start")
    start.add_argument("--camera", choices=("right", "left", "both"), default="both")
    start.add_argument("--fps", type=int, required=True)
    start.add_argument("--width", type=int, required=True)
    start.add_argument("--height", type=int, required=True)
    start.add_argument("--duration", type=int, required=True)
    status = commands.add_parser("status")
    status.add_argument("--include-preview", action="store_true")
    commands.add_parser("stop")
    commands.add_parser("list")
    worker_parser = commands.add_parser("worker")
    worker_parser.add_argument("--camera", choices=tuple(CAMERAS), required=True)
    worker_parser.add_argument("--fps", type=int, required=True)
    worker_parser.add_argument("--width", type=int, required=True)
    worker_parser.add_argument("--height", type=int, required=True)
    worker_parser.add_argument("--duration", type=int, required=True)
    worker_parser.add_argument("--session", required=True)
    return result


def main():
    args = parser().parse_args()
    if args.command == "worker":
        worker(args)
        return
    if args.command == "start":
        result = start_recording(args)
    elif args.command == "stop":
        result = stop_recording()
    elif args.command == "list":
        result = list_sessions()
    else:
        result = active_status(args.include_preview)
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
