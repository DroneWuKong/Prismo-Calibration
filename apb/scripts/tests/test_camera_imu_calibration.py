import sys
from pathlib import Path
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import camera_imu_calibration as cal


def trajectory(t):
    angle=np.array([.27*np.sin(1.7*t)+.1*np.sin(2.6*t), .3*np.sin(1.2*t+.4), .22*np.sin(1.9*t+.8)])
    rx=cv2.Rodrigues(np.array([angle[0],0.,0.]))[0]
    ry=cv2.Rodrigues(np.array([0.,angle[1],0.]))[0]
    rz=cv2.Rodrigues(np.array([0.,0.,angle[2]]))[0]
    p=np.array([.12*np.sin(.9*t),.09*np.sin(1.1*t+.6),.1*np.sin(1.3*t)])
    return rz@ry@rx,p


def dataset(start=0.,duration=22.,shift=.037):
    Rci=cv2.Rodrigues(np.array([.2,-.35,.1]))[0];lever=np.array([.045,-.02,.025])
    bg=np.array([.003,-.002,.001]);ba=np.array([.025,-.03,.015]);g=np.array([0.,0.,-9.80665])
    ct=np.arange(start+.3,start+duration-.3,.05)
    poses={'times':ct.tolist(),'rotations':[],'positions':[]}
    for t in ct:
        r,p=trajectory(t);poses['rotations'].append(r);poses['positions'].append(p)
    it=np.arange(start,start+duration,.005);gyro=[];accel=[];eps=.0001
    for t in it-shift:
        R,p=trajectory(t);Rp,pp=trajectory(t+eps);Rm,pm=trajectory(t-eps)
        Rd=(Rp-Rm)/(2*eps);Rdd=(Rp-2*R+Rm)/eps**2;pdd=(pp-2*p+pm)/eps**2
        W=R.T@Rd;omega=np.array([W[2,1],W[0,2],W[1,0]])
        gyro.append(Rci.T@omega+bg)
        accel.append((R@Rci).T@(pdd+Rdd@lever-g)+ba)
    return poses,{'times':it.tolist(),'gyro':gyro,'accel':accel},Rci,lever


def test_recovers_rotation_delay_and_metric_lever_arm():
    poses,imu,R,lever=dataset()
    result=cal.solve_alignment(poses,imu)
    error=cv2.Rodrigues(np.array(result['R_camera_from_imu'])@R.T)[0]
    assert np.linalg.norm(error)<.005
    assert abs(result['camera_to_imu_time_shift_s']-.037)<.003
    np.testing.assert_allclose(result['t_camera_from_imu_m'],lever,atol=.005)
    assert not result['camera_imu_calibrated'] and not result['vio_ready']
    new_poses,new_imu,_,_=dataset(40.)
    checked=cal.validate_alignment(result,new_poses,new_imu)
    assert checked['independent_validation_passed'] and checked['camera_imu_calibrated']
    assert not checked['vio_ready']


def test_rejects_missing_excitation():
    poses,imu,_,_=dataset(duration=14.)
    poses['rotations']=[np.eye(3) for _ in poses['times']]
    with pytest.raises(ValueError,match='roll, pitch AND yaw'):cal.solve_alignment(poses,imu)


def test_rejects_clock_regressions():
    with pytest.raises(ValueError,match='Non-monotonic'):cal._times([i*.01 for i in range(25)]+[0.], 'IMU',100)


def test_rejects_slow_imu():
    with pytest.raises(ValueError,match='too slow'):cal._times(np.arange(30)*.1,'IMU',100)


def test_rejects_wrong_alignment_on_new_motion():
    poses,imu,_,_=dataset(40.,duration=14.)
    bad={'camera_to_imu_time_shift_s':0.,'R_camera_from_imu':np.eye(3).tolist(),'t_camera_from_imu_m':[0,0,0],
         'gyro_bias_rad_s':[0,0,0],'accel_bias_m_s2':[0,0,0],'gravity_world_m_s2':[0,0,-9.80665]}
    with pytest.raises(ValueError,match='Independent'):cal.validate_alignment(bad,poses,imu)


def test_openvins_export_inverts_the_measured_camera_from_imu_transform():
    import json
    R=cv2.Rodrigues(np.array([.1,.2,-.3]))[0];t=np.array([.04,-.02,.03])
    fit={'independent_validation_passed':True,'R_camera_from_imu':R.tolist(),'t_camera_from_imu_m':t.tolist(),
         'camera_to_imu_time_shift_s':.02}
    intrinsics={'camera_matrix':[[200,0,160],[0,210,90],[0,0,1]],'dist_coeffs':[.1,.2,0,0,0],
                'distortion_model':'radtan','image_size':[320,180]}
    chain=cal.export_openvins_chain(intrinsics,fit)
    rows=[json.loads(line.strip()[2:]) for line in chain.splitlines() if line.startswith('    - ')]
    transform=np.array(rows)
    imu_point=np.array([.4,.2,.1]);camera_point=R@imu_point+t
    np.testing.assert_allclose((transform@np.r_[camera_point,1])[:3],imu_point)
    fit['independent_validation_passed']=False
    with pytest.raises(ValueError):cal.export_openvins_chain(intrinsics,fit)
