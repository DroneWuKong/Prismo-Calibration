# Orqa edition commit checkpoint — 2026-10-18

At the user's request, the pending Orqa edition source, packaging, tests and
documentation are committed in this existing Prismo-Calibration repository on
its existing `main` branch. This checkpoint records the work previously retained
uncommitted while the separate Sensor Calibration Lab was developed. The Lab
remains in its own repository; its source is not staged into this checkout.

Checks repeated against the pending Orqa source on GupB/Linux:

- Full automated suite: **89 passed** (Python 3.12.3, NumPy 1.26.4, OpenCV 4.6.0).
- Headless Chromium: Orqa/Prismo setup, all four imports, readiness/validation
  gates, ZIP reports, firmware provenance, history, recoverable errors, camera
  navigation and mobile layout passed. The source package mapping matched
  `pyproject.toml`; no package was installed into or changed in the Lab repository.
- `node --check apb/scripts/calibration_home.js` and `git diff --check` passed.

Local artifacts are ignored by Git under
`dist/commit-verification-2026-10-18/`. Earlier verification notes and the
universal-app handoff remain unchanged as historical checkpoints. The proprietary
LICENSE, archived installers, migration receipts, remote and active branch are
unchanged. This operation makes a local commit; it does not push or deploy.

These checks use synthetic recordings, rendered images and simulated readiness.
They establish no physical Orqa/Prismo hardware calibration, independent timing
verification, firmware application, navigation activation or flight qualification.
