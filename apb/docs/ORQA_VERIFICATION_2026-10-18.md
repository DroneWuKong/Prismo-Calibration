# Orqa edition software verification — 2026-10-18

This records the Orqa edition follow-up to the initial standalone-workspace
verification. The earlier 71-test note remains unchanged as its historical
checkpoint. This edition was checked on GupB/Linux with Python 3.12.3,
NumPy 1.26.4 and OpenCV 4.6.0.

- Full automated suite: **89 passed**.
- Added recovery checks for radtan/fisheye lens recording imports and lens
  calibration from rendered checkerboard photo batches, including JPEG and PNG.
- Added camera/IMU import checks recovering the known 37 ms time offset and
  nonzero sensor spacing from separate synthetic trajectories.
- Orqa-only setup was checked to make no Prismo/FC/camera connection calls.
  Prismo readiness checks were exercised with simulated disarmed telemetry and
  MJPEG responses, including unavailable-service behavior. Declared MSP firmware
  does not enable the MAVLink recorder. Legacy profile IDs remain accepted.
- Setup metadata, firmware version, exported estimator YAML and synthetic labels
  were checked in the archived session. Reused observations and unsynchronized
  motion inputs were rejected. Photo size headers are checked before decoding.
- Real headless Chromium checks passed at desktop and 390 px mobile widths:
  all four recording imports, Orqa/Prismo choices, firmware provenance, rig
  confirmation, downloads, saved setup/history, invalid-file recovery, photo
  confirmation/missing-target recovery, and the board-only checkerboard helper.
- Guided-navigation readiness was simulated in browser testing. The chosen
  Orqa H7 / Prismo setup reached the camera screen with the correct context;
  this is not evidence of an attached board. The target helper preserved square
  size and kept capture disabled. No browser JavaScript errors or mobile
  horizontal overflow were observed. Desktop/mobile screenshots were inspected.
- `node --check`, `pip check` and `git diff --check` passed. The application
  builds as `prismo_calibration-0.3.0-py3-none-any.whl` and exposes both
  `orqa-sensor-cal` and the compatible `prismo-calibration` command.

Artifacts are local and excluded from Git under `dist/orqa-verification/` and
`dist/prismo_calibration-0.3.0-py3-none-any.whl`. Repeat browser verification with
`tests/browser/workspace_smoke.py`, as documented in the earlier verification
note. Playwright is an optional QA dependency.

These are software and synthetic-data checks. No physical Orqa APB/H7/F405
calibration, firmware application, Windows/macOS qualification, navigation
activation or flight qualification is claimed. Historical installer bundles,
migration receipts and other checkouts were not modified. Development changes
remain local; no commit, push or deployment was performed for this work.
