# Sensor recordings and hardware adapters

Sensor Cal's Orqa edition runs locally as `orqa-sensor-cal`,
`prismo-calibration` or `python -m prismo_calibration`. Setup records Orqa
APB/H7/F405 hardware, optional Prismo integration and declared firmware/version.
Legacy profile IDs remain supported. See the [Orqa edition guide](ORQA_EDITION.md)
for customer procedures and capture limits.

The sensor workspace at `/` imports all four recording types and also extracts
lens observations from JPEG/PNG photo batches. Guided Prismo APB lens and
camera/IMU capture is at `/camera`. These workflows
share an application and output root; live capture retains its original session
downloads, while imported recordings appear in the workspace history.

## Customer workflow

1. Select your hardware profile and sensor workflow.
2. Follow the three preparation/recording steps. Capture training measurements
   and a genuinely new validation recording. Aircraft recording retains the
   existing propellers-removed and disarmed setup requirements.
3. Choose the recorder's Prismo JSON export. Camera rigs also require confirming
   target correspondence, rigidity and synchronized capture.
4. Run “Check recording & calibrate”. Fix the stated acquisition issue if rejected.
5. Download the ZIP: `recording.json`, `result.json` and offline `report.html`.
   Review the adjustment, change a mount if needed, and record again.

Examples can be downloaded from each workflow. They carry `data_origin: synthetic`
and remain marked “Example only” in history and reports. Their results retain
`bench_validated=false` and `simulation_passed=true`. Relabelling source data is
not a substitute for collecting physical evidence.

No calibration is automatically written to the FC, enabled in an estimator, or
treated as flight qualification. Imported provenance is asserted by the recorder;
these checks cannot prove that a physical experiment or independent operator existed.

## Common envelope

```json
{
  "schema": 1,
  "kind": "optical_flow",
  "data_origin": "recording",
  "training": {"recording_id": "bench-001", "samples": []},
  "validation": {"recording_id": "bench-002", "samples": []}
}
```

Every sample has `time_s`, a strictly increasing capture time in seconds within
its recording. Training and validation IDs must differ; exact reused observation
vectors are rejected even if relabelled or retimestamped. At most 160 samples per
recording and 8 MB per HTTP import are allowed. JSON numbers must be finite.

Imports require no connected flight controller. All solving uses the supplied
measurements. Source SHA-256, hardware profile and UTC solve time accompany each
result. The timestamp describes solving, not acquisition.

## Camera lens recording

Use `kind: camera`, `sensor_id`, `square_mm` (10–40), `image_size: [width,height]`
and `distortion_model` (`auto`, `radtan` or `equidistant`). Training needs at least
24 samples; validation needs at least 6 new samples, with tilt about both axes.
Each sample has `time_s` and `corners_px`: 35 row-major pixel pairs for the 7 × 5
inner corners of an 8 × 6 checkerboard. Image dimensions and original geometry
must match. The lens fit uses training only and is checked on the separate
validation recording. Exact reused observations are rejected.

For the photo adapter, each group has `images_base64` instead of `samples`: plain
base64 JPEG/PNG bytes without a data-URL prefix, plus the group's `recording_id`.
Image size is derived from the files. Original image bytes are preserved and
extracted observations accompany the result. `time_basis` is explicitly image
order, not a capture clock.

## Camera/IMU recording

Use `kind: camera_imu`, `sensor_id`, `intrinsics` (validated lens calibration),
`reference_frame` (fixed-target world frame), `pose_convention: R_world_camera`
and `clock_synchronized: true`. Both `training` and `validation` have distinct
`recording_id` values, plus:

```json
{
  "poses": {"times": [], "rotations": [], "positions": []},
  "imu": {"times": [], "gyro": [], "accel": []}
}
```

Populate `poses` with 100–1800 target poses per recording: capture seconds,
3 × 3 world-from-camera rotations and camera positions in metres. Populate
`imu` with 1200–15000 corrected samples: capture seconds, gyroscope rad/s and
accelerometer specific force in m/s² in a consistent IMU frame. Each recording
must fit within 90 seconds. Recorders must map capture clocks to a common time
basis; receipt times are not substitutes. Both recordings must share the same
fixed-target world frame, including gravity orientation. Solver cadence, gap,
excitation, geometry and timing checks apply. Reused pose or IMU observations
are rejected, even with new IDs.

The app estimates from training and validates against the second recording.
Recorders establish units, frames and clock provenance; the synchronization flag
is a supplied assertion, not independent proof. This does not replace standalone
IMU intrinsic calibration.

## Optical flow

Additional envelope fields:

```json
{
  "sensor_id": "downward-flow-01",
  "reference_frame": "bench X right / Y forward",
  "gyro_compensated": true
}
```

One sample (illustrative; a full fit needs at least 12 training and 6 new
validation windows, with motion in both signs of both reference axes):

```json
{
  "time_s": 2.0,
  "duration_s": 0.8,
  "flow_rad": [0.1, -0.04],
  "distance_m": 0.8,
  "reference_displacement_m": [0.08, -0.032],
  "integrated_gyro_rad": [0.001, 0.0, 0.0]
}
```

Use a stationary, textured, flat target and a level downward-looking sensor.
Measure optical-centre-to-plane distance; keep that distance constant within
each integration window. Measure sensor displacement independently (a marked
bench or surveyed reference), in the named XY frame. Vary direction, speed and
duration; collect diagonal as well as axial movements. Range, flow, gyro and
reference displacement must cover the same windows. Rotation magnitude must
remain at or below 0.02 rad per window. The app cannot verify unrecorded tilt or
height changes.

`flow_rad` is a recorder-normalized, gyro-compensated integrated angular-flow
vector. The recorder defines and preserves its two input axes. It must convert
raw sensor units, compensate rotation, and integrate the same window as the
reference. The fitted matrix maps those axes to the declared reference frame.
Do not copy raw MAVLink `OPTICAL_FLOW_RAD.integrated_x/y` blindly: those are flow
*around* axes, and adapters must resolve sensor axis/sign conventions and gyro
compensation. Pixel flow also needs calibrated focal-length conversion first.

The fitted model is:

```text
reference_displacement_m / distance_m
    = flow_to_reference_matrix @ flow_rad
      + drift_reference_rad_s * duration_s
```

The 2 × 2 matrix includes scale, rotation, axis reversal and cross-axis coupling.
The drift vector is in the declared reference frame. `axis_gain` is the norm of
each input-axis column, not a substitute for the full matrix. Reflected mappings
have `rotation_deg=null`; a reflection is not a physical rotation.

Training must constrain both flow axes and drift (normalized design condition
number at most 30). Both recordings need bidirectional XY excitation. Matrix
singular values must be 0.2–5 with anisotropy at most 5. Both training and fresh
validation must have angular vector RMS error at most 0.03 rad and relative RMS
error at most 12%. These are initial bench acceptance limits, not accuracy
specifications for every sensor. Timing offset, rolling shutter, terrain slopes
and a faulty distance sensor are not independently calibrated by this fit.

## Multiple cameras and boresight

The `kind` is `camera_rig`. Additional envelope fields:

```json
{
  "reference_camera": "cam0",
  "square_mm": 25,
  "correspondence_verified": true,
  "cameras": {
    "cam0": {
      "intrinsics_validated": true,
      "camera_matrix": [[520, 0, 320], [0, 525, 240], [0, 0, 1]],
      "dist_coeffs": [0, 0, 0, 0],
      "distortion_model": "radtan",
      "image_size": [640, 480]
    }
  }
}
```

The fragment above shows one camera; a complete rig requires 2–8 distinct camera
IDs and an actual validated lens calibration for each. Lens models may differ
between cameras (`radtan` or `equidistant`), as may resolutions. Use the exact
resolution and optical configuration of each lens calibration.

Each sample has this structure:

```json
{
  "time_s": 2.0,
  "observations": {
    "cam0": {"time_s": 2.0, "corners_px": []},
    "cam1": {"time_s": 2.001, "corners_px": []}
  }
}
```

Replace each empty array with 35 `[x, y]` pixel pairs: the 7 × 5 inner corners
of an 8 × 6 checkerboard, row-major from the **same physical origin** in every
camera. A checkerboard alone has a 180° orientation ambiguity. The recorder must
disambiguate it (e.g. a marked physical origin and orientation-aware tracking);
independent corner detection order is insufficient. Screens must have ruler-
verified scale; printed targets must have independently measured squares.

Use a rigid rig and synchronized capture timestamps on a common clock. Camera
timestamps must each be within 5 ms of the sample time. This bound is necessary
but does not make a moving target safe from synchronization errors; pause the
target for each sample. Include varied left/right and up/down target tilt in
both recordings. At least 12 training and 6 new validation sets are required.
Every camera must see the target in every set. Non-overlapping FOVs, pairwise
chain targets, camera/IMU temporal offsets and mechanical actuator control are
not supported by this initial rig solver.

Each target pose is recovered with the calibrated lens model. Training-only
relative rotations are averaged on SO(3), and relative translations are averaged
in the target camera frame. This is a bounded PnP pose-averaging bench fit, not
joint bundle adjustment. Poor lens agreement, behind-camera poses, less than
20 px target span, missing tilt, rotation spread above 1.5°, translation spread
above 15 mm, or spacing above 5 m are rejected. Training and validation each need
bidirectional cross-camera reprojection RMS at or below 1.5 px. Reported pose
spread is a consistency diagnostic, not a metrological uncertainty bound.

The result contains, for every non-reference camera:

- `T_camera_reference`: homogeneous transform, `p_camera = T @ p_reference`.
- `T_reference_camera`: inverse transform, including the camera origin in the
  reference camera's coordinates. Translation is in metres.
- `boresight_error_deg`: separation between optical forward axes. Image-plane
  twist is also represented in the full rotation, even with zero boresight error.
- `parallel_adjustment_rotation_reference`: active physical rotation of the
  target camera, expressed in the reference camera frame, to make its axes
  parallel to the reference axes. It does not move the camera origin.
- `parallel_adjustment_xyz_deg`: Euler representation of that correction,
  `R_adjust = Rz(z) Ry(y) Rx(x)`. Optical axes are X right, Y down, Z forward.
  This is not an aircraft roll/pitch/yaw or mount-servo convention. At Euler
  singularities the value is null; the rotation matrix remains authoritative.

Applying these transforms in software can align geometry without moving mounts.
For mechanical adjustment, use a mount-specific axis conversion, move the mount,
and remeasure. The app does not claim closed-loop adjustment or automatically
write extrinsics to a camera, FC or estimator.

## Adapter boundary and future sensors

Capture adapters own discovery, transport, timing, raw units, axis conventions,
and ground-truth acquisition. Solvers own numeric validation and calibration.
`sensor_calibration.SOLVERS` registers imported workflows; the workspace catalog
and UI describe supported procedures. Keep new hardware protocols outside the
numeric solvers. Firmware parameter exports need explicit frame/sign conversion
and device verification before they can be offered as apply actions.

Next adapter work for Prismo/APB/H7/F405 is acquiring synchronized optical-flow,
range, IMU and reference movement; exporting corresponding multi-camera corners;
and verifying physical datasets on each firmware/sensor combination. The wider
hardware-neutral app can then add dedicated accelerometer/gyro intrinsic,
magnetometer, rangefinder and barometer workflows with suitable references and
independent checks. The current camera/IMU fit assumes FC-corrected IMU scale and
axes; it does not independently calibrate all IMU intrinsics.

## References

- [OpenCV 4.6 camera calibration and pose conventions](https://docs.opencv.org/4.6.0/d9/d0c/group__calib3d.html)
- [OpenCV 4.6 fisheye model](https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html)
- [MAVLink OPTICAL_FLOW_RAD field units](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD)
