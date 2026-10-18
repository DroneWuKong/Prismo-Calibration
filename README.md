# Prismo Calibration

**Sensor Cal — Orqa edition.** A standalone, local app for Orqa equipment and
Orqa equipment with Prismo. Choose the board, integration and installed firmware;
follow the sensor steps; check fresh measurements; download the result and evidence.
No cloud service or Internet connection is needed while using the app.

Setup choices are Orqa DTK/APB, H7 and F405, each with optional Prismo integration.
Firmware family and version are operator-supplied provenance, not automatic
detection or proof of compatibility. The numeric core remains separate from the
Orqa experience. See the [Orqa edition guide](apb/docs/ORQA_EDITION.md).

| Workflow | Available now | Input |
| --- | --- | --- |
| Camera lens | Intrinsics, radtan/fisheye distortion, separate view checks | Checkerboard photos or recording import; guided capture with Prismo APB services |
| Camera + IMU | Rotation, sensor spacing, gyro/accelerometer bias and time offset, separate motion check | Recording import; guided capture with Prismo APB services |
| Optical flow | XY scale/axis mapping, orientation, drift, separate recording check | Recording import |
| Multi-camera alignment | 2–8 cameras, relative transforms, metric spacing, boresight and physical adjustment angles | Synchronized target-corner recording import |

Orqa-only mode uses all four import workflows without contacting Prismo. Prismo
mode offers a read-only readiness check before exposing guided capture. H7/F405
guided setups need an attached APB camera/companion and the recorder's MAVLink
services. Direct stock-Orqa USB/MSP capture and parameter application are not
implemented. Old profile IDs remain accepted for compatibility. See the
[recording guide](apb/docs/SENSOR_RECORDINGS.md).
Other sensors, including standalone IMU intrinsics, compass, barometer and
rangefinder calibration, need additional workflows and are not claimed as supported.

## Install and launch

Use Python 3.10–3.12 (verified with 3.12):

```sh
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/orqa-sensor-cal --open --output ./sessions
```

Alternatively run `python -m prismo_calibration` from an environment where the
package is installed. The app listens on <http://127.0.0.1:8091/> by default.
`prismo-calibration` remains a compatible launch command. The home screen includes
Orqa/Prismo setup, connection checks, sensor selection, photo/recording imports,
clearly labelled synthetic examples and saved imported sessions.
Each successful import saves the original recording, a JSON calibration and an
offline HTML report in a downloadable ZIP. Sessions survive application restarts.

The existing camera screen is at <http://127.0.0.1:8091/camera>. It displays a
checkerboard on a laptop, captures camera views, estimates camera/IMU alignment
and timing, and checks the result using a separate motion recording. It includes
ASUS PX13, Flow Z13 and Zephyrus G14 display presets.

The four stages are camera calibration, camera/IMU alignment, independent
validation, and session download. Validated sessions include an OpenVINS
camera-chain export. This calibrates inputs for VIO; estimator activation and
physical navigation validation are separate. Results retain `vio_ready=false`.

## Run the web UI

For local development, use Python 3.12:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python apb/scripts/calibration_app.py --output ./sessions
```

Open <http://127.0.0.1:8091/>. Imports and solving work locally; live capture runs
on the APB and requires its camera and flight-controller telemetry services.
The default integration uses:

| Service | Address |
| --- | --- |
| Timestamped, pre-overlay camera feed | `http://127.0.0.1:8082/` |
| APB telemetry and preflight checks | `http://127.0.0.1:8088/` |
| Dedicated MAVLink-router calibration endpoint | `127.0.0.1:14568/UDP` |
| Installed calibration web UI | `http://192.168.75.1:8091/` |

Remove propellers, keep the aircraft disarmed, and verify the displayed reference
size against a ruler before recording. See the original
[operating guide](apb/docs/GUIDED_CAMERA_CALIBRATION.md) for the complete workflow,
capture requirements and solver limits.

## Files

| Path | Purpose |
| --- | --- |
| `apb/scripts/calibration_home.html` | Customer sensor workspace and recording import |
| `apb/scripts/calibration_home.js` | Orqa setup, readiness checks and import UI |
| `apb/scripts/calibration_app.html` | Web UI, display presets and checkerboard |
| `apb/scripts/calibration_app.py` | Standalone HTTP server and camera calibration |
| `apb/scripts/sensor_calibration.py` | Hardware-neutral optical-flow and camera-rig solvers |
| `apb/scripts/calibration_workspace.py` | Hardware profiles, immutable session storage and reports |
| `apb/scripts/calibration_examples.py` | Synthetic recording examples |
| `apb/scripts/calibration_recordings.py` | Lens and camera/IMU recording adapters |
| `apb/scripts/calibration_photos.py` | Bounded JPEG/PNG target extraction |
| `apb/scripts/orqa_setup.py` | Orqa equipment, integration and firmware rules |
| `apb/scripts/calibration_motion.py` | Guided camera/IMU recordings |
| `apb/scripts/calibration_imu.py` | IMU recording and clock synchronization |
| `apb/scripts/camera_imu_calibration.py` | Alignment solver, validation and export |
| `apb/scripts/wingman_vio*.py`, `fusion_control.py` | Original camera and timing dependencies |
| `apb/deploy/wingman-calibration.service` | APB systemd service |
| `artifacts/2026-10-14/` | Original saved APB install bundle and manifest |
| `migration/source.json` | Source revision, file hashes and commit mapping |

## Verify

The existing tests use synthetic camera/IMU data, a simulated UDP router, and
Node.js for browser logic. Install Node.js and the development requirements, then:

```sh
.venv/bin/python -m pytest -q
```

The OpenCV dependency is pinned to 4.6 to match the original APB verification
target. The repository does not contain the complete APB detector or OpenVINS
build system; those remain in `DroneWuKong/Ai-Project`.

Build an installable wheel with `python -m pip wheel --no-deps . -w dist`.
The wheel contains both browser interfaces and the local solvers. Deploying to
an APB still requires its target-compatible Python/OpenCV installation and live
services. The service template explicitly retains `/data/wingman/calibration`;
desktop launches default to `./sessions`.

## Saved APB installer

The original install bundle is preserved byte for byte. Verify it before use:

```sh
cd artifacts/2026-10-14
sha256sum -c SHA256SUMS
mkdir -p /tmp/prismo-calibration-package
tar -xzf guided-calibration-update.tar.gz -C /tmp/prismo-calibration-package
python3 /tmp/prismo-calibration-package/install-guided-calibration.py --check-only
```

Actual installation is an APB administrator operation: run the extracted
`install-guided-calibration.py` as root on the APB. It backs up replaced files,
checks disarmed telemetry and timestamped video, configures the dedicated router
endpoint, and starts the calibration service. The saved detector binary targets
Scarthgap/aarch64. Its original receipt has `flight_qualified=false`.

## Origin and license

Extracted from `DroneWuKong/Ai-Project`, branch
`codex/apb-guided-calibration-20261014`, at commit
`173c89a5bc0697cea636ec02d34b90c944a1d8ef`. Relevant history, authors, dates,
messages and file paths are retained; extraction changes commit IDs.
See [migration notes](migration/README.md).

Copyright (c) 2026 Midwest Nice UAS LLC. Proprietary; see [LICENSE](LICENSE).
