"""Orqa product setup and capabilities; no firmware or sensor parameters are written."""

BOARDS = [
    {"id": "orqa_apb", "name": "Orqa DTK / APB", "capture": "APB camera + FC recordings"},
    {"id": "orqa_h7", "name": "Orqa H7", "capture": "FC and attached-sensor recordings"},
    {"id": "orqa_f405", "name": "Orqa F405", "capture": "FC and attached-sensor recordings"},
]
INTEGRATIONS = [{"id": "orqa", "name": "Orqa equipment"},
                {"id": "prismo", "name": "Orqa equipment with Prismo"}]
FIRMWARES = [{"id": key, "name": label} for key, label in
             (("unknown", "Not sure yet"), ("ardupilot", "ArduPilot"), ("px4", "PX4"),
              ("betaflight", "Betaflight"), ("inav", "INAV"), ("other", "Other firmware"))]
LEGACY = {"prismo_apb": "orqa_apb", "h7": "orqa_h7", "f405": "orqa_f405", "generic": "other"}


def normalize_setup(value=None, profile="prismo_apb"):
    if not isinstance(profile, str) or profile not in {b['id'] for b in BOARDS} | set(LEGACY):
        raise ValueError("Choose a supported hardware profile")
    board = LEGACY.get(profile, profile)
    if value is None:
        # Preserve the provenance of older API clients and saved sessions.
        value = {"hardware": board, "integration": "prismo" if profile in LEGACY and profile != 'generic' else 'orqa'}
    if not isinstance(value, dict) or value.get("hardware") != board:
        raise ValueError("Setup hardware must match the selected hardware profile")
    integration = value.get("integration")
    firmware = value.get("firmware", "unknown")
    version = value.get("firmware_version", "")
    if not isinstance(integration, str) or not isinstance(firmware, str) or integration not in ('orqa', 'prismo') or firmware not in {f['id'] for f in FIRMWARES}:
        raise ValueError("Choose the Orqa/Prismo setup and installed firmware")
    if not isinstance(version, str) or len(version) > 80 or any(ord(c) < 32 for c in version):
        raise ValueError("Firmware version must be a short text value")
    return {"hardware": board, "integration": integration, "firmware": firmware,
            "firmware_version": version.strip(), "identity_source": "operator_selected"}


def capabilities(setup):
    prismo = setup['integration'] == 'prismo'
    # The legacy recorder requires MAVLink RAW_IMU. A declared MSP-based setup
    # cannot enable that path simply by selecting the Prismo integration.
    live = prismo and setup['firmware'] in ('unknown', 'ardupilot', 'px4', 'other')
    return {"recording_imports": ['camera', 'camera_imu', 'optical_flow', 'camera_rig'],
            "guided_capture": live,
            "live_requirements": "Prismo APB camera service, fresh disarmed FC telemetry and a dedicated MAVLink calibration endpoint",
            "parameter_application": False,
            "note": ("Check the Prismo APB services before guided recording. H7/F405 setups need an attached APB camera/companion."
                     if live else "Use saved recordings for this setup. Guided capture requires a supported Prismo APB connection.")}


def setup_label(setup):
    board = next((b['name'] for b in BOARDS if b['id'] == setup['hardware']), 'Other hardware')
    return board + (' · with Prismo' if setup['integration'] == 'prismo' else ' · Orqa equipment')
