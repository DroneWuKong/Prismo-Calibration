# Guided APB camera and IMU calibration

The on-board service hosts the browser interface, recorder, solvers and results.
Once installed, it requires aircraft power and a USB network connection, but no
remote developer host, Internet, ROS or Mission Planner.

Open http://192.168.75.1:8091/ in a full browser window on the laptop display.

1. Select the PX13 preset and check the two-inch reference against a ruler.
   ASUS specifies a 13.3-inch, 16:10, 2880x1800 panel, implying approximately
   286.5x179.1 mm of active display. The advertised diagonal is rounded.
2. Remove propellers, leave the aircraft disarmed and start camera capture.
   Follow the spoken movements. Sharp, distinct views are saved automatically.
   At least 24 views, sufficient image coverage and tilt in both axes are needed.
   Radtan and equidistant lens fits are checked against held-out target views.
3. Select Next: align camera + IMU. Follow the 60-second motion sequence while
   keeping the stationary laptop's whole board visible. Vary rotation direction
   and speed, and include small translations in all axes.
4. Select Check with a second recording. This independent 30-second recording
   validates the candidate; it is not used to fit the calibration.
5. Download the session ZIP. Results, original images, target corners, camera
   poses and synchronized IMU data also remain in the APB's session directory.
   A checked camera/IMU result includes an OpenVINS camera-chain file.

## Prerequisites and behavior

The localhost:8082 pre-overlay camera feed must have capture-monotonic timestamps.
Old detector binaries without those timestamps are rejected. Frame receipt time
is not substituted. Install the current Scarthgap/aarch64 detector build first.

The recorder uses a dedicated loopback MAVLink-router endpoint on port 14568.
It never takes over a GCS endpoint. TIMESYNC maps the FC clock into the companion
clock. During motion capture it temporarily requests primary RAW_IMU at 100 Hz.
The original reported interval is restored on exit, or the FC default if querying
the interval is unsupported. No persistent FC parameters, flight modes, arming,
motor commands or navigation-enabling commands are changed.

Fresh disarmed telemetry is checked before and during capture. Browser loss ends
recording. Camera data needs at least 12 Hz and IMU data at least 100 Hz, allowing
5% cadence tolerance, with monotonic overlapping timestamps and no long gaps.
Each motion stage is bounded to 90 seconds, 15000 IMU samples, 1800 frames and
32 MB of JPEGs. At least 64 MB free storage is required before starting motion.

Sessions are isolated below /data/wingman/calibration. Existing runtime settings
and calibration files are not overwritten or automatically activated. Same-origin
POST requests and an ephemeral CSRF token protect capture starts. This is a
private bench-network service, not an Internet-facing service.

## Solver and limits

The bench solver uses local polynomial derivatives of fixed-target camera poses.
It estimates camera/IMU rotation, gyro bias and timing offset from angular motion,
then jointly fits metric sensor spacing, gravity and accelerometer bias. It
rejects insufficient excitation, ambiguous timing, poor conditioning, excessive
residuals, implausible geometry and large translation uncertainty. A separate
recording must agree before the result is marked checked.

OpenVINS T_imu_cam is the inverse of measured camera-from-IMU geometry. Its timing
convention is IMU sample time = camera sample time + timeshift_cam_imu.

This solver is not Kalibr's spline batch implementation and has not demonstrated
equivalent metrological performance. It assumes FC-corrected IMU scale and axes.
It does not independently calibrate IMU nonorthogonality, rolling shutter,
long-term noise/random walk or temperature compensation. Native VIO installation,
noise configuration, initialization and physical navigation validation remain
separate. The output retains vio_ready=false; calibration is not flight proof.

## Software verification

Run with OpenCV 4.6, matching the APB:

    python -m pytest -q apb/scripts/tests tests/vio

Tests recover known camera intrinsics, camera/IMU rotation, a 37 ms offset and a
nonzero metric lever arm from synthetic trajectories, then validate a separate
trajectory. Other tests cover invalid data, actual UDP clock synchronization and
stream-rate restoration against a simulated router, generated MAVLink fixtures,
missing camera timestamps, setup checks, HTTP origin enforcement and export scope.
The four-stage browser interface and checkerboard fit have been visually checked.
Physical recordings are still required after installation.

References:

- https://www.asus.com/us/laptops/for-creators/proart/proart-px13-hn7306/techspec/
- https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration
- https://docs.openvins.com/gs-calibration.html
