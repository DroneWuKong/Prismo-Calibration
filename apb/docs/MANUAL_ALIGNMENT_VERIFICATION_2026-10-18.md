# Orqa manual alignment milestone — 2026-10-18

Version 0.4.0 packages the assisted manual multi-camera alignment and lever-arm
workbench at `/alignment`, linked from the existing home screen. The Orqa product
focus, four numerical workflows and numerical source remain unchanged.

The independently packaged four-file component matches the Sensor Calibration
Lab SHA-256 manifest. There are no runtime imports from the Lab checkout. The
existing proprietary LICENSE and historical installer/evidence records remain
unchanged. Development stays on the existing `main` branch and origin; this
milestone makes a local commit without pushing or publishing.

See [MANUAL_ALIGNMENT.md](MANUAL_ALIGNMENT.md) for the bench procedure, signed
measurement convention, worksheet schema and limits. The workbench supports
2–8 camera views or JPEG/PNG stills, native-pixel target marks, principal-point
guides from matching lens results, overlay/wipe and focus mode. Up to 16 sensor
positions use an explicit common datum/axes, mm/cm/m, sensing-point definitions
and optional coordinate bounds. Review computes `p_sensor - p_reference` in
metres; unknown bounds remain unknown. No firmware parameters are generated.

Checks on GupB/Linux:

- **89 automated tests passed**, retaining the existing numerical/regression
  suite. The Lab's corresponding suite passed **143 tests**, including the pure
  shared component's geometry, units, signs, bounds and invalid-input checks.
- Headless Chromium passed this Orqa host's manual workflow: 2/8 views, focus
  controls, target marks, zoom/wipe, signed vectors, unit conversion, IMU reference,
  unknown bounds, confirmation gates, worksheet export/reopen, frozen times,
  source/preview hashes, corrupt-preview rejection and synthetic inheritance.
- Synthetic camera streams checked permissions, duplicate devices,
  principal-point linking, freeze/resume/stop, settings changes, disconnect and
  pending-request cleanup. JPEG/PNG imports, lever-only worksheets and narrow
  layouts passed. No JavaScript errors or external requests were observed.
- The 0.4.0 wheel's 23 packaged source/resource files match current source bytes,
  including all four manual assets. Installation from local wheels and dependency
  checks passed. The installed package serves matching assets at
  `http://127.0.0.1:8095/alignment`. The existing historical-checkout preview on
  8091 was left untouched.

Logs, screenshots, generated worksheets, package checks and hashes are retained
locally under ignored `dist/verification-0.4/`. The committed
`MANUAL_ALIGNMENT_VERIFICATION_0.4.json` identifies the wheel and artifact hashes.
Previous verification notes remain historical records.

All new browser streams, images and measurements are synthetic. Manual worksheet
claims remain unverified: no numerical calibration pass, physical mount accuracy,
camera driver qualification, synchronized exposures, verified uncertainty,
firmware application or flight qualification is established. Positions are in
the operator-declared measurement frame and infer no camera/IMU rotation.
