"""Clearly marked synthetic example recordings, also useful for solver verification."""
import cv2
import numpy as np


def flow_example():
    rng = np.random.default_rng(19)
    angle = np.deg2rad(12)
    transform = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ np.diag([1.12, .94])
    drift = np.array([.002, -.001])
    data = {"schema": 1, "kind": "optical_flow", "data_origin": "synthetic", "sensor_id": "example-flow",
            "reference_frame": "bench X right / Y forward", "gyro_compensated": True}
    for name, count in (("training", 30), ("validation", 12)):
        samples = []
        for i in range(count):
            displacement = rng.uniform(-.22, .22, 2)
            height = rng.uniform(.65, 1.1)
            duration = rng.uniform(.4, 1.5)
            flow = np.linalg.solve(transform, displacement / height - drift * duration)
            flow += rng.normal(0, .0001, 2)
            samples.append({"time_s": 1. + i * 2., "flow_rad": flow.tolist(),
                            "reference_displacement_m": displacement.tolist(), "distance_m": height,
                            "duration_s": duration, "integrated_gyro_rad": [0., 0., 0.]})
        data[name] = {"recording_id": f"example-flow-{name}", "samples": samples}
    return data


def rig_example(models=("radtan", "equidistant"), count=2):
    rng = np.random.default_rng(23)
    k = np.array([[520., 0, 320.], [0, 525., 240.], [0, 0, 1.]])
    cameras, geometry = {}, {}
    for i in range(count):
        model = models[i % len(models)]
        cameras[f"cam{i}"] = {"camera_matrix": k.tolist(), "dist_coeffs": [0., 0., 0., 0.],
                              "image_size": [640, 480], "distortion_model": model, "intrinsics_validated": True}
        geometry[f"cam{i}"] = (cv2.Rodrigues(np.array([.025 * i, -.035 * i, .012 * i]))[0], np.array([.06 * i, .008 * i, .003 * i]))
    objects = np.zeros((35, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2) * .025
    data = {"schema": 1, "kind": "camera_rig", "data_origin": "synthetic", "reference_camera": "cam0",
            "square_mm": 25, "correspondence_verified": True, "cameras": cameras}
    for name, n in (("training", 24), ("validation", 12)):
        samples = []
        for i in range(n):
            r = cv2.Rodrigues(np.array([-.38 + (i % 4) * .25, -.4 + (i // 4 % 3) * .35, .08 * np.sin(i)]))[0]
            t = np.array([-.08 + rng.uniform(-.02, .02), -.05 + rng.uniform(-.02, .02), rng.uniform(.5, .75)])
            observations = {}
            for key, (rc, tc) in geometry.items():
                rvec, translation = cv2.Rodrigues(rc @ r)[0], rc @ t + tc
                if cameras[key]["distortion_model"] == "equidistant":
                    points, _ = cv2.fisheye.projectPoints(objects.reshape(-1, 1, 3), rvec, translation, k, np.zeros(4))
                else:
                    points, _ = cv2.projectPoints(objects, rvec, translation, k, np.zeros(4))
                points = points.reshape(-1, 2) + rng.normal(0, .015, (35, 2))
                observations[key] = {"time_s": 1. + i * 2., "corners_px": points.tolist()}
            samples.append({"time_s": 1. + i * 2., "observations": observations})
        data[name] = {"recording_id": f"example-rig-{name}", "samples": samples}
    return data


def lens_example(model='radtan'):
    rng = np.random.default_rng(31)
    k = np.array([[520., 0, 320.], [0, 525., 240.], [0, 0, 1.]])
    objects = np.zeros((35, 3), dtype=np.float64)
    objects[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2) * .025
    data = {'schema': 1, 'kind': 'camera', 'data_origin': 'synthetic', 'sensor_id': 'example-camera',
            'square_mm': 25, 'image_size': [640, 480], 'distortion_model': model}
    for name, count in (('training', 30), ('validation', 12)):
        samples = []
        for i in range(count):
            r = np.array([-.5 + (i % 6) * .2, -.5 + (i // 6) * .22, .15 * np.sin(i)])
            t = np.array([-.075 + .04 * np.sin(i), -.05 + .03 * np.cos(i), .4 + .07 * (i % 4)])
            t += rng.uniform(-.01, .01, 3)
            if model == 'equidistant':
                points, _ = cv2.fisheye.projectPoints(objects.reshape(-1, 1, 3), r, t, k, np.array([-.02, .002, 0., 0.]))
            else:
                points, _ = cv2.projectPoints(objects, r, t, k, np.array([-.09, .015, .001, -.001, 0.]))
            points = points.reshape(-1, 2) + rng.normal(0, .015, (35, 2))
            samples.append({'time_s': 1. + i * 2, 'corners_px': points.tolist()})
        data[name] = {'recording_id': f'example-lens-{name}', 'samples': samples}
    return data


def motion_example_group(start):
    def trajectory(t):
        angle = np.array([.27*np.sin(1.7*t)+.1*np.sin(2.6*t), .3*np.sin(1.2*t+.4), .22*np.sin(1.9*t+.8)])
        rx = cv2.Rodrigues(np.array([angle[0], 0., 0.]))[0]
        ry = cv2.Rodrigues(np.array([0., angle[1], 0.]))[0]
        rz = cv2.Rodrigues(np.array([0., 0., angle[2]]))[0]
        return rz @ ry @ rx, np.array([.12*np.sin(.9*t), .09*np.sin(1.1*t+.6), .1*np.sin(1.3*t)])
    rci = cv2.Rodrigues(np.array([.2, -.35, .1]))[0]
    lever = np.array([.045, -.02, .025])
    bg, ba, gravity = np.array([.003, -.002, .001]), np.array([.025, -.03, .015]), np.array([0., 0., -9.80665])
    ct = np.arange(start + .3, start + 21.7, .05)
    poses = {'times': ct.tolist(), 'rotations': [], 'positions': []}
    for t in ct:
        r, pos = trajectory(t)
        poses['rotations'].append(r.tolist())
        poses['positions'].append(pos.tolist())
    it = np.arange(start, start + 22, .005)
    gyro, accel, eps = [], [], .0001
    for t in it - .037:
        r, pos = trajectory(t)
        rp, pp = trajectory(t + eps)
        rm, pm = trajectory(t - eps)
        rd, rdd = (rp - rm) / (2 * eps), (rp - 2 * r + rm) / eps ** 2
        pdd = (pp - 2 * pos + pm) / eps ** 2
        w = r.T @ rd
        omega = np.array([w[2, 1], w[0, 2], w[1, 0]])
        gyro.append((rci.T @ omega + bg).tolist())
        accel.append(((r @ rci).T @ (pdd + rdd @ lever - gravity) + ba).tolist())
    return {'recording_id': f'example-motion-{start}', 'poses': poses,
            'imu': {'times': it.tolist(), 'gyro': gyro, 'accel': accel}}


def imu_example():
    intrinsics = {'camera_matrix': [[520., 0., 320.], [0., 525., 240.], [0., 0., 1.]],
                  'dist_coeffs': [0., 0., 0., 0.], 'image_size': [640, 480],
                  'distortion_model': 'radtan', 'intrinsics_validated': True}
    return {'schema': 1, 'kind': 'camera_imu', 'data_origin': 'synthetic', 'sensor_id': 'example-camera-imu',
            'reference_frame': 'fixed-target', 'pose_convention': 'R_world_camera', 'clock_synchronized': True,
            'intrinsics': intrinsics, 'training': motion_example_group(0.), 'validation': motion_example_group(40.)}


def example(kind):
    if kind == 'camera':
        return lens_example()
    if kind == 'camera_imu':
        return imu_example()
    if kind == "optical_flow":
        return flow_example()
    if kind == "camera_rig":
        return rig_example()
    raise ValueError("Unknown recording example")
