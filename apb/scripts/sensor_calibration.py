"""Hardware-neutral, bounded bench solvers. Inputs use explicit SI units and frames."""
import hashlib
import math

import cv2
import numpy as np


def array(value, shape, label):
    try:
        result = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"Invalid {label}") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"Invalid {label}: expected finite values with shape {shape}")
    return result


def number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Invalid {label}")
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{label} must be between {low} and {high}")
    return result


def identifier(value, label):
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= 100:
        raise ValueError(f"Provide a {label} (1–100 characters)")
    return value.strip()


def recordings(data, minimum):
    groups = []
    for name, count in zip(("training", "validation"), minimum):
        group = data.get(name)
        if not isinstance(group, dict):
            raise ValueError(f"Provide a separate {name} recording")
        identifier(group.get("recording_id"), f"{name} recording ID")
        samples = group.get("samples")
        if not isinstance(samples, list) or not count <= len(samples) <= 160:
            raise ValueError(f"{name}: collect {count}–160 samples")
        if any(not isinstance(s, dict) for s in samples):
            raise ValueError(f"Invalid {name} samples")
        times = [number(s.get("time_s"), 0, 1e12, "capture time in seconds") for s in samples]
        if np.any(np.diff(times) <= 0):
            raise ValueError(f"{name}: capture times must increase")
        groups.append(group)
    if groups[0]["recording_id"] == groups[1]["recording_id"]:
        raise ValueError("Validation must use a separate recording ID")
    return groups


def reject_reused(training, validation):
    """Do not accept relabeled/retimestamped training observations as validation."""
    def signature(value):
        return hashlib.sha256(np.round(np.asarray(value).ravel(), 8).tobytes()).digest()
    used = {signature(v) for v in training}
    if any(signature(v) in used for v in validation):
        raise ValueError("Validation reuses training observations; record new measurements")


def solve_optical_flow(data):
    """Fit reference displacement / distance = A @ flow_rad + bias * duration.

    flow_rad is the adapter's gyro-compensated integrated angular flow, NOT an
    unconverted MAVLink field. The reference XY frame is declared by the user.
    Flat target, normal optical axis, constant height within each motion window.
    """
    sensor = identifier(data.get("sensor_id"), "sensor ID")
    frame = identifier(data.get("reference_frame"), "reference XY frame")
    if data.get("gyro_compensated") is not True:
        raise ValueError("Use gyro-compensated angular flow in radians")
    groups = recordings(data, (12, 6))
    matrices, targets, fingerprints = [], [], []
    for group in groups:
        x, y, signatures = [], [], []
        for sample in group["samples"]:
            flow = array(sample.get("flow_rad"), (2,), "angular flow")
            displacement = array(sample.get("reference_displacement_m"), (2,), "measured displacement")
            height = number(sample.get("distance_m"), .05, 20, "target distance in metres")
            duration = number(sample.get("duration_s"), .02, 30, "integration duration in seconds")
            gyro = array(sample.get("integrated_gyro_rad"), (3,), "integrated rotation")
            if np.linalg.norm(gyro) > .02:
                raise ValueError("Keep the sensor level; repeat windows with more than 0.02 rad rotation")
            if np.linalg.norm(flow) > 2 or np.linalg.norm(displacement / height) > 2:
                raise ValueError("Use shorter motion windows (angular displacement below 2 rad)")
            x.append([*flow, duration])
            y.append(displacement / height)
            signatures.append([*flow, *displacement, height, duration, *gyro])
        x, y = np.asarray(x), np.asarray(y)
        if np.linalg.eigvalsh(np.cov(y.T))[0] < .0001:
            raise ValueError("Move in both reference directions, including diagonal movements")
        if any(y[:, axis].min() >= -.01 or y[:, axis].max() <= .01 for axis in range(2)):
            raise ValueError("Include forward and reverse movements on both reference axes")
        matrices.append(x)
        targets.append(y)
        fingerprints.append(signatures)
    reject_reused(*fingerprints)
    x, y = matrices[0], targets[0]
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale < 1e-8) or np.linalg.cond(x / scale) > 30:
        raise ValueError("Flow axes and drift are not observable; vary direction, speed and duration")
    coefficients = np.linalg.lstsq(x, y, rcond=None)[0]
    transform = coefficients[:2].T
    singular = np.linalg.svd(transform, compute_uv=False)
    if singular.min() < .2 or singular.max() > 5 or singular.max() / singular.min() > 5:
        raise ValueError("Implausible flow scale or axis coupling; check units, distance and reference axes")
    errors = []
    for x, y in zip(matrices, targets):
        residual = x @ coefficients - y
        rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
        relative = rms / max(float(np.sqrt(np.mean(np.sum(y ** 2, axis=1)))), 1e-9)
        if relative > .12 or rms > .03:
            raise ValueError("Flow does not agree with measured movement; check texture, height and synchronization")
        errors.append({"rms_rad": rms, "relative_error": relative})
    u, _, vt = np.linalg.svd(transform)
    orthogonal = u @ vt
    reflected = bool(np.linalg.det(transform) < 0)
    return {"schema": 1, "kind": "optical_flow", "sensor_id": sensor,
            "reference_frame": frame, "flow_to_reference_matrix": transform.tolist(),
            "drift_reference_rad_s": coefficients[2].tolist(),
            "axis_gain": np.linalg.norm(transform, axis=0).tolist(),
            "axes_reflected": reflected,
            "rotation_deg": None if reflected else math.degrees(math.atan2(orthogonal[1, 0], orthogonal[0, 0])),
            "training_error": errors[0], "validation_error": errors[1],
            "training_samples": len(matrices[0]), "validation_samples": len(matrices[1]),
            "equation": "reference_displacement_m / distance_m = flow_to_reference_matrix @ flow_rad + drift_reference_rad_s * duration_s",
            "validation_recording_id": groups[1]["recording_id"], "bench_validated": True,
            "validation_scope": "Separate imported recording; source provenance is supplied by the recorder",
            "vio_ready": False, "flight_qualified": False}


def camera_model(config):
    if not isinstance(config, dict) or config.get("intrinsics_validated") is not True:
        raise ValueError("Every camera needs validated lens calibration")
    k = array(config.get("camera_matrix"), (3, 3), "camera matrix")
    size = array(config.get("image_size"), (2,), "image size")
    if np.any(size < 16) or np.any(size > 16384) or np.any(size != np.floor(size)):
        raise ValueError("Invalid calibrated camera resolution")
    if not np.allclose(k[2], [0, 0, 1]) or k[0, 0] <= 0 or k[1, 1] <= 0 or abs(k[0, 1]) > 1e-8 or abs(k[1, 0]) > 1e-8:
        raise ValueError("Invalid pinhole camera matrix")
    if not 0 < k[0, 2] < size[0] or not 0 < k[1, 2] < size[1]:
        raise ValueError("Camera principal point is outside the image")
    model = config.get("distortion_model")
    if model not in ("radtan", "equidistant"):
        raise ValueError("Use radtan or equidistant lens calibration")
    d = np.asarray(config.get("dist_coeffs"), dtype=float)
    if d.ndim != 1 or not np.isfinite(d).all() or len(d) not in ((4,) if model == "equidistant" else (4, 5, 8, 12, 14)):
        raise ValueError("Invalid lens distortion coefficients")
    return k, d, model, size


def project(objects, rotation, translation, camera):
    k, d, model, _ = camera
    rvec = cv2.Rodrigues(rotation)[0]
    if model == "equidistant":
        points, _ = cv2.fisheye.projectPoints(objects.reshape(-1, 1, 3), rvec, translation.reshape(3, 1), k, d)
    else:
        points, _ = cv2.projectPoints(objects, rvec, translation, k, d)
    return points.reshape(-1, 2)


def pose(objects, observation, camera, time):
    if not isinstance(observation, dict):
        raise ValueError("Missing synchronized camera observation")
    stamp = number(observation.get("time_s"), 0, 1e12, "camera capture time")
    if abs(stamp - time) > .005:
        raise ValueError("Camera observations must share a capture clock and agree within 5 ms")
    k, d, model, size = camera
    points = array(observation.get("corners_px"), (len(objects), 2), "corresponding target corners")
    if np.any(points < 0) or np.any(points >= size):
        raise ValueError("Target corners must be inside the calibrated image")
    if np.any(np.ptp(points, axis=0) < 20):
        raise ValueError("Move the target closer; it must span at least 20 pixels in both directions")
    normalized = cv2.fisheye.undistortPoints(points.reshape(-1, 1, 2), k, d) if model == "equidistant" else None
    ok, r, t = cv2.solvePnP(objects, normalized if normalized is not None else points,
                          np.eye(3) if normalized is not None else k, None if normalized is not None else d)
    if not ok or not np.isfinite(t).all():
        raise ValueError("Cannot determine target pose")
    rotation = cv2.Rodrigues(r)[0]
    translation = t.ravel()
    if np.any((objects @ rotation.T + translation)[:, 2] <= 0):
        raise ValueError("Target is behind the camera")
    error = np.linalg.norm(project(objects, rotation, translation, camera) - points, axis=1)
    if np.sqrt(np.mean(error ** 2)) > 1:
        raise ValueError("Target observations do not match lens calibration")
    return rotation, translation, points


def mean_rotation(rotations):
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    return u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt


def euler_xyz(rotation):
    """R = Rz(z) Ry(y) Rx(x); degrees about optical right/down/forward axes."""
    y = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    if abs(math.cos(y)) < 1e-6:
        return None  # matrix remains authoritative at Euler singularities
    return [math.degrees(math.atan2(rotation[2, 1], rotation[2, 2])), math.degrees(y),
            math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))]


def solve_camera_rig(data):
    configs = data.get("cameras")
    if not isinstance(configs, dict) or not 2 <= len(configs) <= 8:
        raise ValueError("Provide lens calibrations for 2–8 cameras")
    cameras = {identifier(key, "camera ID"): camera_model(value) for key, value in configs.items()}
    if len(cameras) != len(configs):
        raise ValueError("Camera IDs must be distinct")
    reference = data.get("reference_camera")
    if reference not in cameras:
        raise ValueError("Choose a reference camera from the rig")
    if data.get("correspondence_verified") is not True:
        raise ValueError("Confirm the same physical target corner order in every camera")
    square = number(data.get("square_mm"), 5, 100, "target square size in mm")
    objects = np.zeros((35, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2) * square / 1000
    groups = recordings(data, (12, 6))
    parsed, fingerprints = [], []
    for group in groups:
        poses, signatures = [], []
        last_stamps = {key: -1. for key in cameras}
        for sample in group["samples"]:
            observations = sample.get("observations")
            if not isinstance(observations, dict):
                raise ValueError("Provide synchronized observations for every camera")
            row = {key: pose(objects, observations.get(key), camera, sample["time_s"])
                   for key, camera in cameras.items()}
            for key in cameras:
                stamp = observations[key]["time_s"]
                if stamp <= last_stamps[key]:
                    raise ValueError("Each camera's capture times must increase; do not reuse frames")
                last_stamps[key] = stamp
            poses.append(row)
            signatures.append(np.concatenate([row[key][2].ravel() for key in cameras]))
        normals = np.array([row[reference][0][:, 2] for row in poses])
        if min(np.ptp(normals[:, :2], axis=0)) < .2:
            raise ValueError("Tilt the target left/right AND up/down in each recording")
        parsed.append(poses)
        fingerprints.append(signatures)
    reject_reused(*fingerprints)
    results = {}
    for key in cameras:
        if key == reference:
            continue
        rotations, translations = [], []
        for row in parsed[0]:
            ra, ta, _ = row[reference]
            rb, tb, _ = row[key]
            r = rb @ ra.T
            rotations.append(r)
            translations.append(tb - r @ ta)
        r = mean_rotation(rotations)
        t = np.mean(translations, axis=0)
        spread_deg = max(math.degrees(np.linalg.norm(cv2.Rodrigues(ri @ r.T)[0])) for ri in rotations)
        spread_m = max(np.linalg.norm(ti - t) for ti in translations)
        if spread_deg > 1.5 or spread_m > .015 or np.linalg.norm(t) > 5:
            raise ValueError("Camera geometry is inconsistent; check rigidity, synchronization and corner order")
        errors = []
        for rows in parsed:
            residual = []
            for row in rows:
                ra, ta, points_a = row[reference]
                rb, tb, points_b = row[key]
                residual.extend(np.linalg.norm(project(objects, r @ ra, r @ ta + t, cameras[key]) - points_b, axis=1))
                residual.extend(np.linalg.norm(project(objects, r.T @ rb, r.T @ (tb - t), cameras[reference]) - points_a, axis=1))
            rms = float(np.sqrt(np.mean(np.square(residual))))
            if rms > 1.5:
                raise ValueError("Camera alignment failed the cross-camera reprojection check; record sharper, synchronized views")
            errors.append(rms)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        inverse = np.eye(4)
        inverse[:3, :3], inverse[:3, 3] = r.T, -r.T @ t
        results[key] = {"T_camera_reference": transform.tolist(), "T_reference_camera": inverse.tolist(),
                        "baseline_m": float(np.linalg.norm(t)),
                        "boresight_error_deg": math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1)))),
                        "parallel_adjustment_xyz_deg": euler_xyz(r),
                        "parallel_adjustment_rotation_reference": r.tolist(),
                        "training_rms_px": errors[0], "validation_rms_px": errors[1],
                        "rotation_spread_deg": spread_deg, "translation_spread_m": float(spread_m)}
    return {"schema": 1, "kind": "camera_rig", "reference_camera": reference, "cameras": results,
            "coordinate_convention": "Optical axes: X right, Y down, Z forward. p_camera = T_camera_reference @ p_reference (homogeneous, metres).",
            "adjustment_convention": "Physical rotation in reference optical coordinates to make camera axes parallel: R_adjust = R_camera_reference. Euler Rz(z) Ry(y) Rx(x); remeasure after adjustment. Translation is not adjusted.",
            "training_samples": len(parsed[0]), "validation_samples": len(parsed[1]),
            "validation_recording_id": groups[1]["recording_id"], "bench_validated": True,
            "validation_scope": "Separate imported recording; source provenance is supplied by the recorder",
            "vio_ready": False, "flight_qualified": False}


def solve_lens_recording(data):
    if __package__:
        from .calibration_recordings import solve_lens
    else:
        from calibration_recordings import solve_lens
    return solve_lens(data)


def solve_imu_recording(data):
    if __package__:
        from .calibration_recordings import solve_camera_imu
    else:
        from calibration_recordings import solve_camera_imu
    return solve_camera_imu(data)


SOLVERS = {"camera": solve_lens_recording, "camera_imu": solve_imu_recording,
           "optical_flow": solve_optical_flow, "camera_rig": solve_camera_rig}


def solve_recording(data):
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError("Use a schema 1 Prismo recording")
    kind = data.get("kind")
    if not isinstance(kind, str) or kind not in SOLVERS:
        raise ValueError("Choose camera, camera_imu, optical_flow or camera_rig calibration")
    try:
        return SOLVERS[kind](data)
    except (cv2.error, np.linalg.LinAlgError, TypeError, OverflowError) as exc:
        raise ValueError("Recording could not be solved; check numeric fields, units and target observations") from exc
