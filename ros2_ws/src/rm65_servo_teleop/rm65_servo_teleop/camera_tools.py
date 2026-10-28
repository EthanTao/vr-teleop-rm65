"""No-arm camera preparation, acquisition and capture verification CLI."""

import argparse
import json
from pathlib import Path
import time

import yaml

from .camera import CameraConfig, enumerate_devices
from .camera_runtime import CameraRuntime
from .peripheral_recording import verify_capture
from .sensor_pairing import pair_capture


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    inspect = sub.add_parser("inspect", help="preview or explicitly enumerate SDK devices/profiles")
    inspect.add_argument("--execute-readonly", action="store_true")
    record = sub.add_parser("record")
    record.add_argument("--source", choices=["mock", "realsense"], default="mock")
    record.add_argument("--profile", type=Path, required=True)
    record.add_argument("--root", type=Path, default=Path("build/experiments"))
    record.add_argument("--seconds", type=float, default=3)
    record.add_argument("--run-label", default="")
    record.add_argument("--execute-readonly", action="store_true")
    verify = sub.add_parser("verify")
    verify.add_argument("path", type=Path)
    pair = sub.add_parser("pair", help="same-host arrival candidates; not exposure synchronization")
    pair.add_argument("--capture", type=Path, required=True)
    pair.add_argument("--journal", type=Path, required=True)
    pair.add_argument("--output", type=Path, required=True)
    pair.add_argument("--max-delta", type=float, default=0.05)
    args = parser.parse_args(argv)
    runtime = None
    try:
        if args.operation == "inspect":
            result = (
                {"preview_only": True, "devices_opened": False}
                if not args.execute_readonly
                else {"devices": enumerate_devices(), "firmware_updated": False, "calibration_performed": False}
            )
        elif args.operation == "verify":
            result = verify_capture(args.path)
        elif args.operation == "pair":
            result = {
                "path": str(pair_capture(args.capture, args.journal, args.output, max_delta_s=args.max_delta)),
                "exposure_synchronized": False,
            }
        else:
            if not 0 < args.seconds <= 3600:
                raise ValueError("seconds must be in (0,3600]")
            config = CameraConfig(**yaml.safe_load(args.profile.read_text(encoding="utf-8")))
            if args.source == "realsense" and not args.execute_readonly:
                result = {
                    "preview_only": True,
                    "devices_opened": False,
                    "serial": config.serial,
                    "source": args.source,
                    "profile": str(args.profile),
                }
            else:
                runtime = CameraRuntime(args.source, config, args.root, run_label=args.run_label)
                runtime.start()
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline and runtime.error is None:
                    time.sleep(0.02)
                runtime.close()
                if runtime.error:
                    raise RuntimeError(runtime.error)
                if runtime.recorder is None:
                    raise RuntimeError("no capture created")
                result = {"path": str(runtime.recorder.path), **verify_capture(runtime.recorder.path)}
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError, TypeError, ImportError, RuntimeError) as exc:
        print(
            json.dumps(
                {
                    "error": str(exc),
                    "capture_path": str(runtime.recorder.path) if runtime and runtime.recorder else None,
                }
            )
        )
        return 2
    finally:
        if runtime:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
