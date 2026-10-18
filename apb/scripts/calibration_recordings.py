"""Recording adapters for lens and camera/IMU solvers, independent of APB services."""
import numpy as np

if __package__:
    from .sensor_calibration import array, camera_model, identifier, number, pose, recordings, reject_reused
    from .camera_imu_calibration import solve_alignment, validate_alignment, export_openvins_chain
else:
    from sensor_calibration import array, camera_model, identifier, number, pose, recordings, reject_reused
    from camera_imu_calibration import solve_alignment, validate_alignment, export_openvins_chain


def solve_lens(data):
    if __package__:
        from .calibration_app import solve, heldout_error, object_points
    else:
        from calibration_app import solve, heldout_error, object_points
    photos = isinstance(data.get('training'), dict) and 'images_base64' in data['training']
    if photos:
        if __package__:
            from .calibration_photos import observations
        else:
            from calibration_photos import observations
        data = observations(data)
    sensor = identifier(data.get('sensor_id'), 'camera ID')
    square = number(data.get('square_mm'), 10, 40, 'target square size in mm')
    size = array(data.get('image_size'), (2,), 'image size')
    if np.any(size < 16) or np.any(size > 16384) or np.any(size != np.floor(size)):
        raise ValueError('Use the actual camera resolution')
    size = tuple(int(v) for v in size)
    model = data.get('distortion_model', 'auto')
    if model not in ('auto', 'radtan', 'equidistant'):
        raise ValueError('Choose auto, radtan or equidistant lens fitting')
    groups = recordings(data, (24, 6))
    views = []
    for group in groups:
        points = [array(s.get('corners_px'), (35, 2), '7 × 5 target corners') for s in group['samples']]
        if any(np.any(p < 0) or np.any(p >= size) for p in points):
            raise ValueError('Target corners must match the camera resolution')
        views.append(points)
    reject_reused(*views)
    result = solve([p.reshape(-1, 1, 2) for p in views[0]], size, square, model)
    camera = camera_model(result)
    objects = object_points(square)
    normals = []
    for sample in groups[1]['samples']:
        r, _, _ = pose(objects, sample, camera, sample['time_s'])
        normals.append(r[:, 2])
    if min(np.ptp(np.array(normals)[:, :2], axis=0)) < .2:
        raise ValueError('Tilt the target in both directions in the validation recording')
    error = heldout_error([objects for _ in views[1]], [p.reshape(-1, 1, 2) for p in views[1]],
                          np.array(result['camera_matrix']), np.array(result['dist_coeffs']), result['distortion_model'])
    if error > 1:
        raise ValueError('Lens calibration failed the new recording; use sharper, more varied views')
    result.update(kind='camera', sensor_id=sensor, validation_rms_px=error,
                  training_samples=len(views[0]), validation_samples=len(views[1]),
                  validation_recording_id=groups[1]['recording_id'], bench_validated=True,
                  independent_validation_passed=True, flight_qualified=False,
                  validation_scope='Separate imported target recording; recorder supplies source provenance')
    if photos:
        result['photo_observations'] = {'training': groups[0], 'validation': groups[1], 'time_basis': data['time_basis']}
    return result


def motion_group(group):
    if not isinstance(group, dict):
        raise ValueError('Provide training and validation motion recordings')
    identifier(group.get('recording_id'), 'motion recording ID')
    poses, imu = group.get('poses'), group.get('imu')
    if not isinstance(poses, dict) or not isinstance(imu, dict):
        raise ValueError('Motion recording needs timestamped target poses and IMU samples')
    count = len(poses.get('times', []))
    icount = len(imu.get('times', []))
    if not 100 <= count <= 1800 or not 1200 <= icount <= 15000:
        raise ValueError('Use 100–1800 target poses and 1200–15000 IMU samples per recording')
    rotations = array(poses.get('rotations'), (count, 3, 3), 'world-from-camera rotations')
    positions = array(poses.get('positions'), (count, 3), 'camera positions in metres')
    gyro = array(imu.get('gyro'), (icount, 3), 'gyroscope radians per second')
    accel = array(imu.get('accel'), (icount, 3), 'acceleration metres per second squared')
    pt = array(poses['times'], (count,), 'camera capture seconds')
    it = array(imu['times'], (icount,), 'IMU capture seconds')
    if pt[-1] - pt[0] > 90 or it[-1] - it[0] > 90:
        raise ValueError('Each motion recording must fit within 90 seconds')
    return poses, imu, np.column_stack((rotations.reshape(count, 9), positions)), np.column_stack((gyro, accel))


def solve_camera_imu(data):
    identifier(data.get('sensor_id'), 'camera/IMU pair ID')
    identifier(data.get('reference_frame'), 'shared fixed-target world frame')
    if data.get('clock_synchronized') is not True or data.get('pose_convention') != 'R_world_camera':
        raise ValueError('Provide synchronized capture clocks and R_world_camera target poses')
    intrinsics = data.get('intrinsics')
    camera_model(intrinsics)
    training, validation = data.get('training'), data.get('validation')
    tp, ti, pose_train, imu_train = motion_group(training)
    vp, vi, pose_val, imu_val = motion_group(validation)
    if training['recording_id'] == validation['recording_id']:
        raise ValueError('Use a separate validation recording ID')
    reject_reused(pose_train, pose_val)
    reject_reused(imu_train, imu_val)
    result = validate_alignment(solve_alignment(tp, ti), vp, vi)
    result.update(kind='camera_imu', sensor_id=data['sensor_id'],
                  reference_frame=data['reference_frame'], intrinsics=intrinsics,
                  training_samples=len(tp['times']), validation_samples=len(vp['times']),
                  validation_recording_id=validation['recording_id'], bench_validated=True,
                  flight_qualified=False, capture_timing_validated=True,
                  imu_intrinsics_source='Recorder-supplied corrected IMU scale and axes',
                  validation_scope='Separate imported motion recording; recorder supplies source and synchronization provenance')
    if intrinsics['distortion_model'] == 'radtan' and any(abs(v) > 1e-10 for v in intrinsics['dist_coeffs'][4:]):
        result['export_note'] = 'JSON only: this lens has additional nonzero distortion terms that the four-coefficient OpenVINS exporter cannot represent.'
    else:
        result['openvins_chain'] = export_openvins_chain(intrinsics, result)
    return result
