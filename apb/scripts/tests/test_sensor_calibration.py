"""Known-geometry recovery, failed validation and recording boundary checks."""
import copy
import io
import json
from pathlib import Path
import sys
import threading
import urllib.error
import urllib.request
import zipfile

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_examples import flow_example, rig_example
from calibration_workspace import Workspace
from sensor_calibration import solve_recording
import calibration_app as app


def test_flow_recovers_scale_orientation_and_drift():
    result = solve_recording(flow_example())
    np.testing.assert_allclose(result['axis_gain'], [1.12, .94], atol=.001)
    assert result['rotation_deg'] == pytest.approx(12, abs=.02)
    np.testing.assert_allclose(result['drift_reference_rad_s'], [.002, -.001], atol=.00003)
    assert result['validation_error']['relative_error'] < .002
    assert not result['axes_reflected']
    assert not result['flight_qualified'] and not result['vio_ready']


def test_flow_recovers_axis_reversal_without_inventing_a_rotation():
    data = flow_example()
    for group in ('training', 'validation'):
        for sample in data[group]['samples']:
            sample['flow_rad'][0] *= -1
    result = solve_recording(data)
    assert result['axes_reflected'] and result['rotation_deg'] is None
    np.testing.assert_allclose(result['axis_gain'], [1.12, .94], atol=.001)


@pytest.mark.parametrize('kind', ['optical_flow', 'camera_rig'])
def test_validation_cannot_reuse_retimestamped_training_observations(kind):
    data = flow_example() if kind == 'optical_flow' else rig_example()
    data['validation']['samples'] = copy.deepcopy(data['training']['samples'])
    for sample in data['validation']['samples']:
        sample['time_s'] += 100
        for observation in sample.get('observations', {}).values():
            observation['time_s'] += 100
    with pytest.raises(ValueError, match='reuses training'):
        solve_recording(data)


def test_flow_rejects_failed_independent_validation():
    data = flow_example()
    for sample in data['validation']['samples']:
        sample['reference_displacement_m'][0] *= 1.8
    with pytest.raises(ValueError, match='does not agree'):
        solve_recording(data)


@pytest.mark.parametrize('problem,match', [('one_axis', 'both reference'), ('rotation', 'Keep the sensor level'),
                                         ('no_compensation', 'gyro-compensated'), ('bad_time', 'increase'),
                                         ('nonfinite', 'finite'), ('same_id', 'separate recording ID')])
def test_flow_rejects_unusable_recordings(problem, match):
    data = flow_example()
    if problem == 'one_axis':
        for sample in data['training']['samples']:
            sample['reference_displacement_m'][1] = 0
    elif problem == 'rotation':
        data['validation']['samples'][0]['integrated_gyro_rad'] = [.1, 0, 0]
    elif problem == 'no_compensation':
        data['gyro_compensated'] = False
    elif problem == 'bad_time':
        data['training']['samples'][1]['time_s'] = 0
    elif problem == 'nonfinite':
        data['training']['samples'][0]['flow_rad'][0] = float('nan')
    else:
        data['validation']['recording_id'] = data['training']['recording_id']
    with pytest.raises(ValueError, match=match):
        solve_recording(data)


@pytest.mark.parametrize('models', [('radtan', 'radtan'), ('equidistant', 'equidistant'), ('radtan', 'equidistant')])
def test_camera_rig_recovers_known_metric_geometry_and_physical_adjustment(models):
    result = solve_recording(rig_example(models))
    camera = result['cameras']['cam1']
    transform = np.array(camera['T_camera_reference'])
    expected_rotation = cv2.Rodrigues(np.array([.025, -.035, .012]))[0]
    np.testing.assert_allclose(transform[:3, :3], expected_rotation, atol=.0002)
    np.testing.assert_allclose(transform[:3, 3], [.06, .008, .003], atol=.0002)
    np.testing.assert_allclose(transform @ camera['T_reference_camera'], np.eye(4), atol=1e-9)
    # The correction is an active rotation in the reference frame. It must
    # physically align the measured camera basis with the reference basis.
    camera_basis_in_reference = np.array(camera['T_reference_camera'])[:3, :3]
    np.testing.assert_allclose(np.array(camera['parallel_adjustment_rotation_reference']) @ camera_basis_in_reference,
                               np.eye(3), atol=1e-9)
    assert camera['validation_rms_px'] < .05
    assert result['validation_samples'] == 12


def test_three_camera_rig_uses_chosen_reference():
    data = rig_example(count=3)
    data['reference_camera'] = 'cam1'
    result = solve_recording(data)
    assert set(result['cameras']) == {'cam0', 'cam2'}
    assert result['cameras']['cam2']['baseline_m'] == pytest.approx(.0606, abs=.0003)


@pytest.mark.parametrize('problem,match', [('timing', '5 ms'), ('intrinsics', 'validated lens'),
                                         ('corner_order', 'corner order'), ('missing_camera', 'Missing synchronized'),
                                         ('resolution', 'inside'), ('flat', 'Tilt the target')])
def test_camera_rig_rejects_bad_capture_and_geometry(problem, match):
    data = rig_example()
    if problem == 'timing':
        data['training']['samples'][0]['observations']['cam1']['time_s'] += .03
    elif problem == 'intrinsics':
        data['cameras']['cam1']['intrinsics_validated'] = False
    elif problem == 'corner_order':
        data['correspondence_verified'] = False
    elif problem == 'missing_camera':
        del data['training']['samples'][0]['observations']['cam1']
    elif problem == 'resolution':
        data['training']['samples'][0]['observations']['cam1']['corners_px'][0][0] = 700
    else:
        for sample in data['training']['samples']:
            for key in data['cameras']:
                sample['observations'][key]['corners_px'] = copy.deepcopy(data['training']['samples'][0]['observations'][key]['corners_px'])
    with pytest.raises(ValueError, match=match):
        solve_recording(data)


def test_camera_rig_fails_independent_validation_with_changed_mount():
    data = rig_example()
    for sample in data['validation']['samples']:
        for point in sample['observations']['cam1']['corners_px']:
            point[0] += 8
    with pytest.raises(ValueError, match='cross-camera reprojection'):
        solve_recording(data)


def test_camera_rig_rejects_inconsistent_physical_corner_origin():
    data = rig_example()
    data['training']['samples'][3]['observations']['cam1']['corners_px'].reverse()
    with pytest.raises(ValueError, match='inconsistent'):
        solve_recording(data)


@pytest.mark.parametrize('profile', [None, {}, 'unknown-board'])
def test_workspace_requires_known_profile(tmp_path, profile):
    with pytest.raises(ValueError, match='hardware profile'):
        Workspace(tmp_path).calibrate(flow_example(), profile)


def test_workspace_preserves_provenance_examples_history_and_export(tmp_path):
    workspace = Workspace(tmp_path)
    data = flow_example()
    data['sensor_id'] = '<script>alert(1)</script>'
    result = workspace.calibrate(data, 'f405')
    assert not result['bench_validated'] and result['simulation_passed']
    assert result['device_profile'] == 'f405'
    assert Workspace(tmp_path).history() == [result]
    with zipfile.ZipFile(io.BytesIO(workspace.export(result['session_id']))) as archive:
        assert set(archive.namelist()) == {'recording.json', 'result.json', 'report.html'}
        assert json.loads(archive.read('recording.json')) == data
        assert b'&lt;script&gt;' in archive.read('report.html')
        assert b'Example only' in archive.read('report.html')
    data['data_origin'] = 'recording'
    real = workspace.calibrate(data, 'h7')
    assert real['bench_validated'] and 'simulation_passed' not in real
    assert real['session_id'] != result['session_id']
    assert len(workspace.history()) == 2


def test_failed_recording_never_creates_successful_session(tmp_path):
    workspace = Workspace(tmp_path)
    data = flow_example()
    data['validation']['samples'] = []
    with pytest.raises(ValueError):
        workspace.calibrate(data, 'generic')
    assert workspace.history() == []
    for path in ('../secret', 'session-../../secret', 'session-not-found'):
        with pytest.raises(ValueError, match='Unknown'):
            workspace.export(path)


def test_http_import_calibrate_history_and_download(tmp_path):
    server = app.ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
    server.session = app.Session(tmp_path, 'unused')
    server.ui = Path(app.__file__).with_suffix('.html')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        data = json.load(urllib.request.urlopen(base + '/api/workspace/example/camera_rig'))
        token = json.load(urllib.request.urlopen(base + '/api/token'))['token']
        payload = json.dumps({'recording': data, 'device_profile': 'prismo_apb'}).encode()
        # A real rig recording is larger than the old capture-settings limit.
        assert len(payload) > 8192
        headers = {'Origin': base, 'X-Calibration-Token': token, 'Content-Type': 'application/json'}
        req = urllib.request.Request(base + '/api/workspace/calibrate', data=payload, headers=headers)
        result = json.load(urllib.request.urlopen(req))['result']
        assert not result['bench_validated']
        history = json.load(urllib.request.urlopen(base + '/api/workspace/history'))
        assert history['sessions'][0]['session_id'] == result['session_id']
        with zipfile.ZipFile(io.BytesIO(urllib.request.urlopen(base + '/api/workspace/export/' + result['session_id']).read())) as archive:
            assert 'report.html' in archive.namelist()
        headers['Origin'] = 'http://other.example'
        req = urllib.request.Request(base + '/api/workspace/calibrate', data=payload, headers=headers)
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(req)
        assert exc.value.code == 403
    finally:
        server.shutdown()
        server.server_close()
