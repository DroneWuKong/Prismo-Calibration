# Guided APB camera calibration

Open `http://192.168.75.1:8091/` in a full browser window on the laptop display.
The on-board `wingman-calibration` service serves both the UI and capture API.
Only capture uses the camera: opening the page does not start video processing.

1. Select the ASUS ProArt PX13 preset, or adjust the two-inch ruler reference.
   The PX13 estimate uses ASUS's published 13.3-inch, 16:10, 2880x1800 panel:
   https://www.asus.com/us/laptops/for-creators/proart/proart-px13-hn7306/techspec/
   The calculated active area is approximately 286.5 x 179.1 mm. Advertised
   diagonal rounding, non-native resolution, browser zoom and a different monitor
   make the ruler check necessary. CSS physical units are not trusted.
2. Confirm the ruler reference, remove propellers and leave the aircraft disarmed.
3. Start capture. Keep the laptop still and follow the spoken movements.
   Use 25 mm squares, or explicitly select a smaller board if it does not fit.
   The app refuses to silently scale a board to fit the viewport.
4. Captures require detected corners, sufficient sharpness, a brief steady pose,
   and a view sufficiently different from the previously saved observations.
   Camera width/height come from actual frames, never a requested capture size.
5. At least 24 views, image coverage and tilt about both axes are required.
   Pinhole/radtan and fisheye/equidistant fits are compared on held-out views.
   Saved fits require training RMS <=0.8 px and held-out RMS <=1.0 px.
6. Download the ZIP containing physical screen settings, source images, detected
   corners and the checked camera calibration. Previous calibration files and
   flight-controller settings are never overwritten by this tool.

The output is camera intrinsics, NOT a calibrated visual-inertial estimator.
`camera_imu_calibrated`, `capture_timing_validated` and `vio_ready` remain false.
It does not invent camera-to-IMU transforms or scale, enable FC forwarding,
change modes, arm, run motors or write flight-controller parameters.
A fresh disarmed FC heartbeat is required before capture and every 10 seconds.
The tool reads the existing status/preflight APIs and sends no MAVLink commands.

Camera source: existing pre-overlay MJPEG fanout on localhost:8082. No V4L2
ownership change and no detector restart. Captures decode at up to 4 Hz and run
at nice 10 with a 200 MB service limit. Up to 48 views per session and 20 sessions;
32 MB free storage is required before starting. Sessions are saved under
`/data/wingman/calibration`, and each output has a unique directory.

API: same-origin POST plus ephemeral CSRF token; no cross-origin capture starts.
The LAN-facing tool has the same private bench network assumption as the APB
camera feed. It is not an Internet-facing calibration service.

## Verification

Run using OpenCV 4.6 (the APB version):
`python -m pytest -q apb/scripts/tests/test_calibration_app.py`
Synthetic projection tests recover pinhole and fisheye focal lengths and reject
repeated views and inconsistent corners. HTTP tests exercise origin rejection,
setup prerequisites, export scope and missing-heartbeat refusal. Browser checks
cover the PX13 preset, fullscreen board display and insufficient-window warning.
These checks do not substitute for a physical capture or a known-target check.
