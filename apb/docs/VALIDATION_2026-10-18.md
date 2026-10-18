# Standalone sensor workspace verification — 2026-10-18

Verified on GupB, Linux, Python 3.12.3, NumPy 1.26.4 and OpenCV 4.6.0.
This is software verification using synthetic recordings and a local browser.
No physical APB/H7/F405 recording, mount adjustment, Windows/macOS qualification,
flight qualification, or independent human measurement is claimed.

- `python -m pytest -q`: **71 passed**. The original camera/IMU tests remain
  included. Added tests recover known optical-flow scale, orientation, drift and
  axis reversal; mixed radtan/fisheye rig transforms and metric spacing; and
  three-camera geometry with a non-default reference camera.
- Rejection tests cover changed mounts, inconsistent corner origins, absent
  intrinsics/cameras, inadequate tilt or flow excitation, excessive rotation,
  bad timing, invalid numeric values and reused validation observations.
- HTTP checks exercise the home and camera routes, origin/token enforcement,
  full-size rig imports, session history and ZIP export. Workspace tests verify
  example provenance, hardware profile validation, immutable sessions, report
  escaping, rejection without saving a successful result, and export boundaries.
- `tests/browser/workspace_smoke.py` passed using headless Chromium. It exercises
  both recording imports, the rig confirmation gate, F405/H7 result provenance,
  downloaded ZIP contents, synthetic labels, history reload, bad-file recovery,
  camera navigation and a 390 px mobile viewport. No browser JavaScript errors
  or mobile horizontal overflow were observed. Desktop home and rig-result
  screenshots were also visually inspected.
- The camera screen restores completed calibration and continuation controls
  after navigating through the sensor home; its browser-logic regression passed.
- `pip wheel --no-deps . -w dist` produced an installable wheel. An isolated
  target installation outside the checkout loaded the browser assets and ran
  the mixed-lens rig solver. `pip check` and `git diff --check` passed.

Local generated artifacts (excluded from Git):

- `dist/prismo_calibration-0.2.0-py3-none-any.whl`
- `dist/verification/home-desktop.png`
- `dist/verification/home-mobile.png`
- `dist/verification/optical_flow-result.png`
- `dist/verification/camera_rig-result.png`

To repeat browser verification, install the package plus `playwright`, provision
Chromium with `playwright install chromium`, then run:

```sh
python tests/browser/workspace_smoke.py --artifacts dist/verification
```

`--browser-executable` can select an already provisioned Chromium. The script
starts and stops an isolated local server with temporary session storage.
Playwright is an optional verification dependency, not an application dependency.

The application is implemented locally. Direct optical-flow/multi-camera capture
adapters, firmware-specific parameter application and physical dataset validation
remain follow-up work. The saved 2026-10-14 installer and migration evidence have
not been modified or represented as validation of this version.
