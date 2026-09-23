#!/usr/bin/env python3
# ruff: noqa: F403,F405
"""Capture one JPEG from either Hikrobot USB camera and exit."""
import argparse
import fcntl
import json
import os
import sys
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof

os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
sys.path.insert(0, "/opt/MVS/Samples/64/Python/MvImport")
from MvCameraControl_class import *  # noqa: E402,F403


def decode(value):
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def devices():
    result = MV_CC_DEVICE_INFO_LIST()
    code = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, result)
    if code != MV_OK:
        raise RuntimeError(f"camera enumeration failed: 0x{code:x}")
    items = []
    for index in range(result.nDeviceNum):
        info = cast(result.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents
        items.append((index, info, decode(info.SpecialInfo.stUsb3VInfo.chSerialNumber),
                      decode(info.SpecialInfo.stUsb3VInfo.chModelName)))
    return items


def capture(index, quality):
    items = devices()
    if index < 0 or index >= len(items):
        raise RuntimeError(f"camera {index + 1} unavailable; detected {len(items)}")
    cam = MvCamera()
    frame = MV_FRAME_OUT()
    created = opened = grabbing = False
    try:
        code = cam.MV_CC_CreateHandle(items[index][1])
        if code != MV_OK:
            raise RuntimeError(f"create handle failed: 0x{code:x}")
        created = True
        code = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if code != MV_OK:
            raise RuntimeError(f"open camera failed: 0x{code:x}")
        opened = True
        code = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if code != MV_OK:
            raise RuntimeError(f"disable trigger failed: 0x{code:x}")
        # Let each independently powered camera adapt before taking the still.
        # Factory upper limit was only ~27.8 ms on both onboard cameras, which
        # produced valid but nearly black JPEGs in their current installation.
        cam.MV_CC_SetIntValue("AutoExposureTimeUpperLimit", 200000)
        cam.MV_CC_SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS)
        cam.MV_CC_SetEnumValue("GainAuto", 2)
        code = cam.MV_CC_StartGrabbing()
        if code != MV_OK:
            raise RuntimeError(f"start capture failed: 0x{code:x}")
        grabbing = True
        memset(byref(frame), 0, sizeof(frame))
        for attempt in range(12):
            code = cam.MV_CC_GetImageBuffer(frame, 8000)
            if code != MV_OK or not frame.pBufAddr:
                raise RuntimeError(f"frame timeout: 0x{code:x}")
            if attempt < 11:
                cam.MV_CC_FreeImageBuffer(frame)
                memset(byref(frame), 0, sizeof(frame))
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
    finally:
        if frame.pBufAddr:
            cam.MV_CC_FreeImageBuffer(frame)
        if grabbing:
            cam.MV_CC_StopGrabbing()
        if opened:
            cam.MV_CC_CloseDevice()
        if created:
            cam.MV_CC_DestroyHandle()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--camera", type=int, choices=(1, 2))
    parser.add_argument("--quality", type=int, default=75, choices=range(51, 100))
    args = parser.parse_args()
    MvCamera.MV_CC_Initialize()
    try:
        with open(f"/run/user/{os.getuid()}/robot320-hik-usb.lock", "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if args.list:
                print(json.dumps([{"camera": i + 1, "serial": s, "model": m} for i, _, s, m in devices()]))
                return
            if args.camera is None:
                parser.error("--camera is required unless --list is used")
            sys.stdout.buffer.write(capture(args.camera - 1, args.quality))
    finally:
        MvCamera.MV_CC_Finalize()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
