# Migration verification — October 17, 2026

Verified in the extracted GupB checkout with Python 3.12.3, OpenCV 4.6.0,
NumPy 1.26.4, pytest 9.1.1 and the installed Node.js runtime.

- All 18 extracted files match the SHA-256 hashes of the original source Git
  blobs recorded in `source.json`.
- All 14 extracted historical commits retain their original author dates and
  messages; their original and extracted IDs are mapped in `source.json`.
- Existing tests: **42 passed in 5.71 seconds**. These cover camera intrinsics,
  camera/IMU alignment and independent validation, UDP recording and clock sync,
  browser lifecycle and display presets, HTTP request checks, and supporting
  VIO reader regressions.
- A fresh HTTP server process served the original UI bytes, reported an idle
  session and generated a session token.
- The saved installer passed `--check-only`, including its aarch64 detector
  check. Every file in its original manifest matched its SHA-256 hash.

The original October 14 test environment at
`/tmp/apb-calibration-opencv46-20261014` supplied the dependencies. A new dependency
installation was interrupted by GupB's full disk; verification then used that
existing environment against files in this new checkout.

These are software and package checks. The original bundle's physical-validation
status and `flight_qualified=false` receipt remain unchanged.
