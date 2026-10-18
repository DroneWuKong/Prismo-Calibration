"""Portable calibration catalog and immutable, downloadable recording sessions."""
import hashlib
import html
import io
import json
from pathlib import Path
import secrets
import shutil
import threading
import time
import zipfile

if __package__:
    from .sensor_calibration import solve_recording
    from .orqa_setup import BOARDS, FIRMWARES, INTEGRATIONS, normalize_setup, setup_label
else:
    from sensor_calibration import solve_recording
    from orqa_setup import BOARDS, FIRMWARES, INTEGRATIONS, normalize_setup, setup_label

PROFILES = BOARDS
CATALOG = [
    {"id": "camera", "name": "Camera lens", "description": "Check the lens, image centre and distortion for an APB or attached camera.", "mode": "Recording import · Prismo guided capture", "href": "/camera"},
    {"id": "camera_imu", "name": "Camera + IMU", "description": "Align a camera with the FC motion sensor and measure timing and sensor spacing.", "mode": "Recording import · Prismo guided capture", "href": "/camera#next"},
    {"id": "optical_flow", "name": "Optical flow", "description": "Check motion scale, sensor orientation, reversed axes and drift against measured movement.", "mode": "Recording import"},
    {"id": "camera_rig", "name": "Camera alignment", "description": "Align two or more cameras: boresight, sensor spacing and physical adjustment angles.", "mode": "Recording import"},
]


class Workspace:
    def __init__(self, output):
        self.root = Path(output) / "sensor-sessions"
        self.lock = threading.Lock()

    def catalog(self):
        return {"sensors": CATALOG, "profiles": PROFILES, "integrations": INTEGRATIONS, "firmwares": FIRMWARES,
                "edition": "Orqa sensor calibration", "scope": "Orqa APB, H7 and F405 equipment, with or without Prismo"}

    def history(self):
        if not self.root.exists():
            return []
        results = []
        for folder in sorted(self.root.glob("session-*"), reverse=True)[:100]:
            try:
                results.append(json.loads((folder / "result.json").read_text()))
            except (OSError, ValueError):
                continue
        return results

    def calibrate(self, data, profile, setup=None):
        setup = normalize_setup(setup, profile)
        if not isinstance(data, dict) or data.get("data_origin") not in ("recording", "synthetic"):
            raise ValueError("Recording must declare data_origin: recording or synthetic")
        with self.lock:
            self.root.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(self.root).free < 64 * 1024 * 1024:
                raise ValueError("At least 64 MB free storage is required")
            if len(list(self.root.glob("session-*"))) >= 100:
                raise ValueError("Archive previous sensor sessions before adding more (100 session limit)")
            encoded = json.dumps(data, sort_keys=True, allow_nan=False).encode()
            result = solve_recording(data)
            session_id = "session-" + time.strftime("%Y%m%d-%H%M%S", time.gmtime()) + "-" + secrets.token_hex(4)
            result.update({"session_id": session_id, "device_profile": profile,
                           "setup": setup, "setup_label": setup_label(setup),
                           "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                           "data_origin": data["data_origin"], "source_sha256": hashlib.sha256(encoded).hexdigest()})
            if data["data_origin"] == "synthetic":
                result.update(bench_validated=False, simulation_passed=True)
                if result.get('openvins_chain'):
                    result['openvins_chain'] = result['openvins_chain'].replace('%YAML:1.0\n', '%YAML:1.0\n# SYNTHETIC EXAMPLE: not a physical sensor calibration\n', 1)
            # The directory becomes visible only after all evidence is saved.
            pending = self.root / (".pending-" + secrets.token_hex(8))
            pending.mkdir()
            try:
                (pending / "recording.json").write_bytes(encoded)
                (pending / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False))
                (pending / "report.html").write_text(report(result))
                if result.get('openvins_chain'):
                    (pending / "kalibr_imucam_chain.yaml").write_text(result['openvins_chain'])
                pending.rename(self.root / session_id)
            except Exception:
                shutil.rmtree(pending)
                raise
            return result

    def export(self, session_id):
        # Never use a request path to traverse directories or follow symlinks.
        if not isinstance(session_id, str) or not session_id.startswith("session-") or any(c not in "session-0123456789abcdef" for c in session_id):
            raise ValueError("Unknown sensor session")
        folder = self.root / session_id
        if folder.is_symlink() or not folder.is_dir():
            raise ValueError("Unknown sensor session")
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
            names = ["recording.json", "result.json", "report.html"]
            if (folder / 'kalibr_imucam_chain.yaml').exists():
                names.append('kalibr_imucam_chain.yaml')
            for name in names:
                path = folder / name
                if not path.is_file() or path.is_symlink():
                    raise ValueError("Incomplete sensor session")
                archive.writestr(name, path.read_bytes())
        return buffer.getvalue()


def report(result):
    title = {'camera': 'Camera lens', 'camera_imu': 'Camera + IMU', 'optical_flow': 'Optical flow', 'camera_rig': 'Camera alignment'}[result['kind']]
    status = "Example only — synthetic measurements" if result["data_origin"] == "synthetic" else "Passed imported recording checks"
    rows = []
    if result['kind'] == 'camera':
        rows = [('Camera', result['sensor_id']), ('Lens model', result['distortion_model']),
                ('Validation error', f'{result["validation_rms_px"]:.3f} px'),
                ('Image size', str(result['image_size']))]
    elif result['kind'] == 'camera_imu':
        rows = [('Camera / IMU', result['sensor_id']), ('Time offset', f'{result["camera_to_imu_time_shift_s"] * 1000:.1f} ms'),
                ('Validation gyro RMS', f'{result["validation_gyro_rms_rad_s"]:.5f} rad/s'),
                ('Validation acceleration RMS', f'{result["validation_accel_rms_m_s2"]:.4f} m/s²')]
    elif result["kind"] == "optical_flow":
        rows = [("Sensor", result["sensor_id"]), ("Reference frame", result["reference_frame"]),
                ("X / Y gain", " / ".join(f"{v:.4f}" for v in result["axis_gain"])),
                ("Reversed axis mapping", "Yes" if result["axes_reflected"] else "No"),
                ("Validation error", f'{100 * result["validation_error"]["relative_error"]:.2f}%')]
    else:
        rows.append(("Reference camera", result["reference_camera"]))
        for camera, values in result["cameras"].items():
            rows.extend([(f"{camera} · boresight", f'{values["boresight_error_deg"]:.3f}°'),
                         (f"{camera} · spacing", f'{values["baseline_m"] * 1000:.1f} mm'),
                         (f"{camera} · validation error", f'{values["validation_rms_px"]:.3f} px'),
                         (f"{camera} · physical adjustment XYZ", str(values["parallel_adjustment_xyz_deg"]))])
    table = "".join(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>" for k, v in rows)
    details = html.escape(json.dumps(result, indent=2))
    return f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Orqa sensor calibration · {title} report</title><style>body{{font:16px/1.5 system-ui;max-width:900px;margin:40px auto;padding:20px;color:#17221c}}th,td{{padding:12px;text-align:left;border-bottom:1px solid #ccc}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}small{{color:#536058}}</style>
<p>ORQA EQUIPMENT / SENSOR CALIBRATION</p><h1>{title}</h1><h2>{status}</h2><p>{html.escape(result['created_utc'])} · {html.escape(result.get('setup_label', result['device_profile']))}</p>
<table>{table}</table><p>Calibration describes the imported measurements. Source provenance is supplied by the recorder. Changes are exported for review; no device settings are applied.</p>
<p>Camera adjustments use optical X right, Y down, Z forward, in the reference camera frame. R = Rz(z) Ry(y) Rx(x). Remeasure after changing a mount. Flight qualification and VIO activation remain separate.</p>
<details><summary>Complete calibration and conventions</summary><pre>{details}</pre></details></html>'''
