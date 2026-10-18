"""Orqa-only operation, truthful Prismo capability checks and new recording paths."""
import base64
import copy
import io
import json
from pathlib import Path
import struct
import sys
import zipfile

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibration_app as app
from calibration_examples import example, lens_example
from calibration_photos import photo_corners
from calibration_workspace import Workspace
from orqa_setup import capabilities, normalize_setup
from sensor_calibration import solve_recording


def setup(board='orqa_apb', integration='orqa', firmware='unknown'):
    return {'hardware': board, 'integration': integration, 'firmware': firmware, 'firmware_version': 'test-1'}


@pytest.mark.parametrize('board', ['orqa_apb', 'orqa_h7', 'orqa_f405'])
def test_orqa_only_setup_requires_no_prismo_services(tmp_path, monkeypatch, board):
    def forbidden(*args, **kwargs):
        raise AssertionError('Orqa-only setup must not contact Prismo or a controller')
    monkeypatch.setattr(app, 'verify_disarmed', forbidden)
    import urllib.request
    monkeypatch.setattr(urllib.request, 'urlopen', forbidden)
    session = app.Session(tmp_path, 'unused')
    result = session.check_setup({'device_profile': board, 'setup': setup(board)})
    assert not result['guided_ready']
    assert result['checks'][0]['status'] == 'ready'
    assert result['capabilities']['recording_imports'] == ['camera', 'camera_imu', 'optical_flow', 'camera_rig']
    with pytest.raises(ValueError, match='recording imports'):
        session.start({'device_profile': board, 'setup': setup(board)})


@pytest.mark.parametrize('firmware', ['betaflight', 'inav'])
def test_declared_msp_firmware_does_not_enable_mavlink_recorder(firmware):
    config = normalize_setup(setup(integration='prismo', firmware=firmware), 'orqa_apb')
    assert not capabilities(config)['guided_capture']


def test_prismo_readiness_checks_disarmed_telemetry_and_camera_without_recording(tmp_path, monkeypatch):
    monkeypatch.setattr(app, 'verify_disarmed', lambda: None)
    import urllib.request
    def camera(url, timeout):
        response = io.BytesIO()
        response.headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
        return response
    monkeypatch.setattr(urllib.request, 'urlopen', camera)
    session = app.Session(tmp_path, 'http://camera.invalid/')
    result = session.check_setup({'device_profile': 'orqa_h7', 'setup': setup('orqa_h7', 'prismo', 'ardupilot')})
    assert result['guided_ready']
    assert result['firmware_identity'].startswith('Operator-selected')
    assert not session.running
    assert not list(tmp_path.iterdir())


def test_prismo_services_unavailable_leaves_imports_ready(tmp_path, monkeypatch):
    def unavailable():
        raise ValueError('No fresh flight-controller heartbeat')
    monkeypatch.setattr(app, 'verify_disarmed', unavailable)
    import urllib.request
    def no_camera(*args, **kwargs):
        raise OSError('Camera unavailable')
    monkeypatch.setattr(urllib.request, 'urlopen', no_camera)
    result = app.Session(tmp_path, 'unused').check_setup({'device_profile': 'orqa_apb', 'setup': setup(integration='prismo')})
    assert not result['guided_ready']
    assert [c['status'] for c in result['checks']] == ['ready', 'unavailable', 'unavailable']


def test_setup_mismatch_is_rejected_and_old_profile_ids_are_preserved():
    with pytest.raises(ValueError, match='match'):
        normalize_setup(setup('orqa_h7'), 'orqa_f405')
    assert normalize_setup(None, 'h7')['hardware'] == 'orqa_h7'
    assert normalize_setup(None, 'prismo_apb')['integration'] == 'prismo'
    assert normalize_setup(None, 'generic')['integration'] == 'orqa'


@pytest.mark.parametrize('model', ['radtan', 'equidistant'])
def test_lens_recording_recovers_geometry_and_checks_new_views(model):
    result = solve_recording(lens_example(model))
    assert result['distortion_model'] == model
    assert result['validation_rms_px'] < .1
    np.testing.assert_allclose([result['camera_matrix'][0][0], result['camera_matrix'][1][1]], [520, 525], atol=2)
    assert result['independent_validation_passed']
    assert not result['vio_ready'] and not result['capture_timing_validated']


def test_lens_recording_rejects_reused_training_views():
    data = lens_example()
    data['validation']['samples'] = copy.deepcopy(data['training']['samples'])
    with pytest.raises(ValueError, match='reuses training'):
        solve_recording(data)


@pytest.fixture(scope='module')
def motion():
    return example('camera_imu')


def test_orqa_only_imu_recording_recovers_alignment_and_exports_without_device(tmp_path, motion):
    result = Workspace(tmp_path).calibrate(motion, 'orqa_f405', setup('orqa_f405'))
    assert result['setup']['integration'] == 'orqa'
    assert result['setup']['firmware_version'] == 'test-1'
    assert result['camera_to_imu_time_shift_s'] == pytest.approx(.037, abs=.003)
    np.testing.assert_allclose(result['t_camera_from_imu_m'], [.045, -.02, .025], atol=.005)
    assert result['camera_imu_calibrated'] and not result['bench_validated']
    with zipfile.ZipFile(io.BytesIO(Workspace(tmp_path).export(result['session_id']))) as archive:
        assert 'kalibr_imucam_chain.yaml' in archive.namelist()
        assert b'SYNTHETIC EXAMPLE' in archive.read('kalibr_imucam_chain.yaml')
        assert b'Orqa F405' in archive.read('report.html')


def test_import_requires_synchronized_camera_imu_clock(motion):
    data = {**motion, 'clock_synchronized': False}
    with pytest.raises(ValueError, match='synchronized'):
        solve_recording(data)


def test_import_rejects_reused_motion_even_with_new_ids(motion):
    data = {**motion, 'validation': {**motion['training'], 'recording_id': 'relabeled'}}
    with pytest.raises(ValueError, match='reuses training'):
        solve_recording(data)


def photo_recording():
    target = np.full((400, 500), 255, np.uint8)
    for y in range(6):
        for x in range(8):
            if (x + y) % 2 == 0:
                target[50 + y * 50:100 + y * 50, 50 + x * 50:100 + x * 50] = 0
    k = np.array([[520., 0, 320.], [0, 525., 240.], [0, 0, 1.]])
    world = np.array([[-.05, -.05, 0], [.2, -.05, 0], [.2, .15, 0], [-.05, .15, 0]], dtype=float)
    source = np.array([[0, 0], [500, 0], [500, 400], [0, 400]], dtype=np.float32)
    data = {'schema': 1, 'kind': 'camera', 'data_origin': 'synthetic', 'sensor_id': 'photo-example', 'square_mm': 25, 'distortion_model': 'radtan'}
    for name, count, offset in (('training', 30, 0), ('validation', 12, .013)):
        images = []
        for i in range(count):
            r = np.array([-.5 + i % 6 * .2, -.5 + (i // 6) * .22, .08 * np.sin(i)])
            t = np.array([-.075 + .025 * np.sin(i) + offset, -.05 + .02 * np.cos(i), .5 + .03 * (i % 4)])
            dest = cv2.projectPoints(world, r, t, k, None)[0].reshape(-1, 2).astype(np.float32)
            transform = cv2.getPerspectiveTransform(source, dest)
            image = cv2.warpPerspective(target, transform, (640, 480), borderValue=255)
            ok, encoded = cv2.imencode('.png', image)
            assert ok
            images.append(base64.b64encode(encoded).decode())
        data[name] = {'recording_id': 'photos-' + name, 'images_base64': images}
    return data


def test_photo_batches_extract_target_and_recover_lens_geometry():
    result = solve_recording(photo_recording())
    assert result['validation_rms_px'] < .4
    np.testing.assert_allclose([result['camera_matrix'][0][0], result['camera_matrix'][1][1]], [520, 525], atol=8)
    assert 'image_order_only' in result['photo_observations']['time_basis']
    assert not result['capture_timing_validated']


def test_photo_adapter_rejects_oversized_dimensions_before_decoding():
    raw = b'\x89PNG\r\n\x1a\n' + b'\0\0\0\rIHDR' + struct.pack('>II', 50000, 50000)
    with pytest.raises(ValueError, match='megapixels'):
        photo_corners(base64.b64encode(raw).decode())


def test_photo_adapter_rejects_missing_target():
    ok, encoded = cv2.imencode('.png', np.full((480, 640), 255, np.uint8))
    assert ok
    with pytest.raises(ValueError, match='8 × 6'):
        photo_corners(base64.b64encode(encoded).decode())


def test_photo_adapter_reads_jpeg_target_and_keeps_pixel_geometry():
    data = photo_recording()
    raw = base64.b64decode(data['training']['images_base64'][0])
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    ok, jpeg = cv2.imencode('.jpg', image)
    assert ok
    corners, size = photo_corners(base64.b64encode(jpeg).decode())
    assert size == [640, 480] and len(corners) == 35
