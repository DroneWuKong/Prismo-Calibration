"""Legacy tracker regressions: SI units, specific force, clock gaps and quality."""
import importlib.util
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "apb" / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("vio_bench", SCRIPTS / "wingman_vio.py")
vio = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vio)


def tracker(samples=(), orientation=None):
    obj = vio.WingmanVio.__new__(vio.WingmanVio)
    obj.imu = Mock()
    obj.imu.drain_since.return_value = list(samples)
    obj.orientation = np.array([1., 0., 0., 0.]) if orientation is None else orientation
    obj.vel = np.zeros(3)
    obj.pos = np.zeros(3)
    obj.prev_ts_us = 1_000_000
    obj.prev_gray = np.zeros((180, 320), dtype=np.uint8)
    obj.prev_pts = None
    obj._baro_lock = threading.Lock()
    obj.baro_alt_m = 2.
    obj.frame_count = 0
    obj.out_sock = Mock()
    obj.out_addr = ("127.0.0.1", 14560)
    return obj


def test_scaled_imu_millisecond_clock_and_field_offsets():
    sample = vio.parse_scaled_imu(struct.pack("<Ihhhhhhhhh", 12345, 10, 20, -1000, 100, -200, 300, 0, 0, 0))
    assert sample["ts"] == 12_345_000
    np.testing.assert_allclose(sample["accel"], [.0980665, .196133, -9.80665])
    np.testing.assert_allclose(sample["gyro"], [.1, -.2, .3])


def test_raw_imu_truncated_zero_tail():
    payload = struct.pack("<Qhhhhhhhhh", 123456, 0, 0, -1000, 0, 0, 0, 0, 0, 0)
    sample = vio.parse_raw_imu(payload.rstrip(b"\0"))
    assert sample["ts"] == 123456
    np.testing.assert_allclose(sample["accel"], [0, 0, -9.80665])
    np.testing.assert_allclose(sample["gyro"], [0, 0, 0])


@pytest.mark.parametrize("roll,pitch", [(0,0), (.3,0), (0,-.4), (.2,.3)])
def test_stationary_specific_force_does_not_accelerate(roll, pitch):
    q = vio.quat_from_euler(roll,pitch,0.)
    force = vio.quat_rotate(q * [1,-1,-1,-1], np.array([0.,0.,-vio.G_MPS2]))
    samples = [{"ts":t,"gyro":(0.,0.,0.),"accel":force} for t in range(1_010_000,1_100_001,10_000)]
    obj = tracker(samples, q)
    _, dv, _ = obj._imu_pre_integrate(1_000_000,1_100_000)
    np.testing.assert_allclose(dv, [0,0,0], atol=1e-12)


def test_real_acceleration_is_preserved():
    obj = tracker([{"ts":1_050_000,"gyro":(0.,0.,0.),"accel":(2.,0.,-vio.G_MPS2)}])
    _, dv, _ = obj._imu_pre_integrate(1_000_000,1_050_000)
    np.testing.assert_allclose(dv, [.1,0,0],atol=1e-12)


@pytest.mark.parametrize("timestamp", [1_000_000, 999_000, 2_000_000])
def test_camera_clock_discontinuity_discards_pair(timestamp):
    obj = tracker()
    obj.vel[:] = 4
    assert obj.step(obj.prev_gray, timestamp) is None
    np.testing.assert_array_equal(obj.pos, [0,0,0])
    np.testing.assert_array_equal(obj.vel, [0,0,0])


def test_corner_count_cannot_certify_metric_pose(monkeypatch):
    obj = tracker()
    monkeypatch.setattr(vio, "detect_features_gridded", lambda _: np.zeros((50,1,2),dtype=np.float32))
    est = obj.step(obj.prev_gray, 1_050_000)
    assert est["tracking_confidence"] == 1.
    assert est["conf"] == 0
    assert est["metric_validated"] is False
    assert est["diagnostic_only"] is True


def test_publish_does_not_make_old_capture_fresh():
    obj = tracker()
    est = {"ts":int(time.monotonic()*1e6)-2_000_000}
    old = est["ts"]
    obj.publish(est)
    obj.out_sock.sendto.assert_not_called()
    assert est["ts"] == old
