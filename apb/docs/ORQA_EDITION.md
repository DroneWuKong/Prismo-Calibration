# Sensor Cal — Orqa edition

This edition is for Orqa equipment, and Orqa equipment with Prismo. The broader
hardware-neutral handbook companion is a separate development track. The
[new-chat handoff](UNIVERSAL_APP_NEW_CHAT.txt) preserves that boundary and explains
how to avoid overwriting this checkout's uncommitted work.

## Start

Install the package and run `orqa-sensor-cal --open`.
`prismo-calibration` and `python -m prismo_calibration` remain compatible.

1. Choose Orqa DTK/APB, H7 or F405.
2. Choose **Orqa equipment** or **Orqa equipment with Prismo**.
3. Select installed firmware, or “Not sure yet”; optionally record its version.
4. Check the setup, choose a sensor and follow its acquisition instructions.
5. Review the fresh-recording check and download the result and source evidence.

Board and firmware choices are operator declarations, not detected device IDs.
They do not imply that a board contains a camera or optical-flow sensor. Connect
and record the actual sensors fitted to the aircraft.

## Available paths

| Task | Orqa equipment | Orqa equipment with Prismo |
| --- | --- | --- |
| Camera lens | JPEG/PNG photo batches or recording import | Same inputs, plus checked APB guided capture |
| Camera + IMU alignment | Corrected IMU and target-pose recording import | Same input, plus checked APB guided recording |
| Optical flow | Measured-motion recording import | Measured-motion recording import |
| Multiple-camera alignment | Synchronized target-corner recording import | Synchronized target-corner recording import |

“Check my setup” in Orqa-only mode checks import capability without contacting
Prismo, the FC or the camera. In Prismo mode it checks fresh disarmed telemetry
and that the configured camera endpoint serves MJPEG. It does not start an IMU
recorder or write settings. Capture quality, timestamps, synchronization and
motion rate are checked by the recording/solver path. Readiness is invalidated
when setup changes or the page reloads; capture rechecks telemetry at start and
during recording.

H7/F405 guided setups require a connected APB companion and camera services.
Selecting Prismo does not make a standalone FC supply a camera. Declared
Betaflight/INAV setups retain imports and do not expose the MAVLink RAW_IMU
recorder. Direct stock-Orqa serial/USB/MSP capture is not implemented.

## Easy camera-photo workflow

Choose **Camera lens → Import & calibrate → Use checkerboard photos**.

- Name the actual camera and enter the measured square size (10–40 mm).
- Use **Open the measured checkerboard** for the ruler-adjusted display target.
  This helper passes the chosen square size, disables recording and requires no
  Prismo connection. Check physical display scale against a ruler.
- Select at least 24 varied training photos and 6 genuinely new check photos.
  Each batch supports up to 48 photos. Vary target position, distance and tilt
  about both axes. Keep the same lens, focus, crop and pixel geometry.
- JPEG/PNG files must be below 750 KB each, at most 12 megapixels with no side
  above 4096 pixels, and below 5.8 MB combined (about 8 MB after encoding).
- Confirm the physical square measurement and fixed camera configuration.
- The app extracts the 7 × 5 inner corners, fits the lens and checks new views.
  Missing/poorly fitted targets or inconsistent resolution are rejected.

These must be images **from the camera being calibrated**. Taking pictures of
the APB with a different camera does not calibrate the APB lens. Original image
bytes are kept in the imported recording; extracted observations are included
with the result. Image order is used for static lens fitting only and is
explicitly not presented as a capture clock or timing calibration.

## What changes, and where

Calibration writes a new session below the selected output directory, which
defaults to `./sessions`. The APB service template retains
`/data/wingman/calibration`. Existing active settings are not overwritten.

| Result | Intended consumer |
| --- | --- |
| Camera matrix and lens distortion | Image processing / vision pipeline at the recorded image geometry |
| Camera/IMU rotation, spacing, timing and biases | Visual-inertial estimator configuration |
| Optical-flow matrix and drift | Flow-processing software or a separately verified firmware converter |
| Relative camera transforms and boresight | Multi-camera vision software or measured mount adjustments |

Imported sessions contain the original input, `result.json` and `report.html`.
Representable camera/IMU results also include `kalibr_imucam_chain.yaml`. Lens
models with additional nonzero distortion terms are retained in JSON and receive
an explicit export note; they are not silently truncated into four coefficients.
Synthetic examples remain labelled in reports, JSON and any estimator YAML.

The app does not apply FC parameters, enable navigation, flash firmware or move
mounts. Prismo live motion capture temporarily requests 100 Hz RAW_IMU telemetry
and attempts to restore the prior interval (or firmware default) on exit.
Installing an APB service/router endpoint is a separate administrator operation.

Standalone accelerometer/gyro intrinsic, compass, barometer and rangefinder
calibration are not implemented here; use the installed firmware's appropriate
tools. The camera/IMU fit assumes corrected IMU scale and axes. Physical
validation remains separate from synthetic software checks.

## Provenance and compatibility

New results preserve declared board, integration, firmware and version. Changing
equipment between live lens capture and IMU recording is rejected. Legacy IDs
`prismo_apb`, `h7`, `f405` and `generic` remain accepted by the import API; no
historical recordings are rewritten.

The saved 2026-10-14 APB installer is the original archived bundle and does not
install this edition. Equipment references include the local Orqa APB/H7/F405
source review and [Orqa's equipment page](https://orqafpv.com/enterprise); these
do not establish physical verification of this app on a particular board.
