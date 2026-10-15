"""Camera/IMU calibration candidate from a fixed metric target and corrected IMU.

R_WC maps camera vectors into the fixed target frame. R_CI and t_CI map
IMU vectors/positions into camera coordinates. Pose-derived angular rates
align against the gyroscope; lever arm, gravity and accelerometer bias are
then solved jointly. A separate recording must validate the candidate.
This is a bounded bench solver, not a claim of Kalibr equivalence or flight proof.
"""
import math
import cv2
import numpy as np


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def _times(values,label,min_rate):
    values=np.asarray(values,dtype=float)
    if values.ndim!=1 or len(values)<20 or not np.isfinite(values).all():raise ValueError(f"Invalid {label} timestamps")
    delta=np.diff(values)
    if np.any(delta<=0):raise ValueError(f"Non-monotonic {label} timestamps")
    if 1/np.median(delta)<min_rate*.95 or np.max(delta)>.25:raise ValueError(f"{label} samples too slow or missing")
    return values


def derivatives(times,rotations,positions):
    """Local polynomial derivatives in the tangent frame, with gap rejection."""
    ts=_times(times,'camera',12)
    rotations=np.asarray(rotations,dtype=float);positions=np.asarray(positions,dtype=float)
    if rotations.shape!=(len(ts),3,3) or positions.shape!=(len(ts),3):raise ValueError("Pose dimensions do not match timestamps")
    if not np.isfinite(rotations).all() or not np.isfinite(positions).all():raise ValueError("Nonfinite camera poses")
    if not np.allclose(rotations@rotations.transpose(0,2,1),np.eye(3),atol=1e-5) or not np.allclose(np.linalg.det(rotations),1,atol=1e-5):raise ValueError("Invalid pose rotations")
    output=[]
    for i in range(5,len(ts)-5):
        t=ts[i-5:i+6]-ts[i]
        if np.max(np.diff(t))>.10:continue
        relative=np.array([cv2.Rodrigues(rotations[i].T@r)[0].ravel() for r in rotations[i-5:i+6]])
        polynomial=np.stack([t**j for j in range(6)],axis=1)
        angles=np.linalg.lstsq(polynomial,relative,rcond=None)[0]
        path=np.linalg.lstsq(polynomial,positions[i-5:i+6],rcond=None)[0]
        omega=angles[1];alpha=angles[2]*2
        rdd=rotations[i]@(skew(alpha)+skew(omega)@skew(omega))
        output.append((ts[i],rotations[i],omega,rdd,path[2]*2))
    if len(output)<100:raise ValueError("Not enough continuous tracked target poses")
    return output


def interpolate(times,samples,query):
    if query.min()<times[0] or query.max()>times[-1]:raise ValueError("Camera and IMU recordings do not overlap")
    return np.stack([np.interp(query,times,samples[:,axis]) for axis in range(3)],axis=1)


def rotation_bias(gyro,visual):
    a=gyro-gyro.mean(axis=0);b=visual-visual.mean(axis=0)
    u,_,vt=np.linalg.svd(a.T@b)
    R=vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
    bias=gyro.mean(axis=0)-R.T@visual.mean(axis=0)
    residual=(gyro-bias)@R.T-visual
    return R,bias,float(np.sqrt(np.mean(residual**2)))


def _inputs(poses,imu):
    samples=derivatives(poses['times'],poses['rotations'],poses['positions'])
    it=_times(imu['times'],'IMU',100)
    gyro=np.asarray(imu['gyro'],dtype=float);accel=np.asarray(imu['accel'],dtype=float)
    if gyro.shape!=(len(it),3) or accel.shape!=gyro.shape or not np.isfinite(gyro).all() or not np.isfinite(accel).all():raise ValueError("Invalid IMU samples")
    if it[-1]-it[0]<12:raise ValueError("Record more motion before solving")
    ts=np.array([s[0] for s in samples]);visual=np.array([s[2] for s in samples])
    keep=(ts>it[0]+.16)&(ts<it[-1]-.16)
    samples=[s for s,k in zip(samples,keep) if k];ts=ts[keep];visual=visual[keep]
    if len(ts)<100:raise ValueError("Insufficient overlapping camera and IMU data")
    if np.linalg.eigvalsh(np.cov(visual.T))[0]<.005:raise ValueError("Move in roll, pitch AND yaw; one-axis motion cannot calibrate alignment")
    return samples,it,gyro,accel,ts,visual


def solve_alignment(poses,imu):
    samples,it,gyro,accel,ts,visual=_inputs(poses,imu)
    candidates=[]
    for shift in np.linspace(-.15,.15,301):
        g=interpolate(it,gyro,ts+shift)
        R,bias,error=rotation_bias(g,visual)
        candidates.append((error,float(shift),R,bias))
    error,shift,Rci,bg=min(candidates,key=lambda x:x[0])
    if abs(shift)>.14:raise ValueError("Camera/IMU delay is outside the supported timing search")
    if error>.035:raise ValueError("Camera and gyroscope rotation disagree; improve tracking or timestamp quality")
    # Reject an ambiguous timing solution, including constant-rate rotations.
    alternatives=[e for e,s,_,_ in candidates if abs(s-shift)>.025]
    if min(alternatives)<error*1.08+1e-4:raise ValueError("Timing is ambiguous; vary the speed and direction of rotation")
    f=interpolate(it,accel,ts+shift)
    A=[];y=[]
    for (_,Rwc,_,Rdd,acc),specific in zip(samples,f):
        Rwi=Rwc@Rci
        A.append(np.hstack([Rdd,-np.eye(3),Rwi]));y.append(Rwi@specific-acc)
    A=np.vstack(A);y=np.concatenate(y)
    scales=np.linalg.norm(A,axis=0)
    if np.any(scales<1e-6) or np.linalg.cond(A/scales)>300:raise ValueError("Lever arm and accelerometer bias are not observable; add varied tilt and translation")
    weights=np.ones(len(y))
    for _ in range(5):
        x=np.linalg.lstsq(A*weights[:,None],y*weights,rcond=None)[0]
        residual=A@x-y
        scale=max(.02,float(np.median(np.abs(residual)))*1.4826)
        weights=np.sqrt(np.minimum(1.,(2*scale)/np.maximum(np.abs(residual),1e-9)))
    translation,gravity,ba=x[:3],x[3:6],x[6:]
    accel_rms=float(np.sqrt(np.mean(residual**2)))
    sigma2=float(np.sum(residual**2)/max(1,len(y)-9))
    uncertainty=np.sqrt(np.diag(np.linalg.pinv(A.T@A)*sigma2))[:3]
    if not 9.3<np.linalg.norm(gravity)<10.3:raise ValueError("Gravity/scale check failed; check the displayed square size and camera fit")
    if np.linalg.norm(translation)>.5 or np.linalg.norm(ba)>1.:raise ValueError("Implausible camera/IMU geometry or accelerometer bias")
    if accel_rms>.35 or uncertainty.max()>.025:raise ValueError("Translation uncertainty is too large; use sharper images and varied motion")
    return {'schema':1,'R_camera_from_imu':Rci.tolist(),'t_camera_from_imu_m':translation.tolist(),
            'camera_to_imu_time_shift_s':shift,'time_convention':'imu_sample_time = camera_time + shift',
            'gyro_bias_rad_s':bg.tolist(),'accel_bias_m_s2':ba.tolist(),'gravity_world_m_s2':gravity.tolist(),
            'fit_gyro_rms_rad_s':error,'fit_accel_rms_m_s2':accel_rms,'translation_sigma_m':uncertainty.tolist(),
            'camera_imu_calibrated':False,'independent_validation_passed':False,'vio_ready':False}


def validate_alignment(candidate,poses,imu):
    samples,it,gyro,accel,ts,visual=_inputs(poses,imu)
    shift=float(candidate['camera_to_imu_time_shift_s'])
    R=np.array(candidate['R_camera_from_imu']);t=np.array(candidate['t_camera_from_imu_m'])
    bg=np.array(candidate['gyro_bias_rad_s']);ba=np.array(candidate['accel_bias_m_s2']);g=np.array(candidate['gravity_world_m_s2'])
    angular=(interpolate(it,gyro,ts+shift)-bg)@R.T-visual
    force=interpolate(it,accel,ts+shift)-ba
    acceleration=[]
    for (_,Rwc,_,Rdd,cdd),f in zip(samples,force):
        acceleration.extend(Rwc@R@f-(cdd+Rdd@t-g))
    gr=float(np.sqrt(np.mean(angular**2)));ar=float(np.sqrt(np.mean(np.square(acceleration))))
    if gr>.04 or ar>.4:raise ValueError("Independent motion check failed; repeat calibration rather than enabling VIO")
    return {**candidate,'independent_validation_passed':True,'validation_gyro_rms_rad_s':gr,
            'validation_accel_rms_m_s2':ar,'camera_imu_calibrated':True,'vio_ready':False}


def target_pose(corners,intrinsics):
    w,h=intrinsics['board_size'];square=intrinsics['square_mm']/1000
    obj=np.zeros((w*h,3),np.float64);obj[:,:2]=np.mgrid[0:w,0:h].T.reshape(-1,2)*square
    K=np.array(intrinsics['camera_matrix']);D=np.array(intrinsics['dist_coeffs'])
    image=np.asarray(corners,dtype=float).reshape(-1,1,2)
    if intrinsics['distortion_model']=='equidistant':
        image=cv2.fisheye.undistortPoints(image,K,D);K=np.eye(3);D=None
    ok,r,t=cv2.solvePnP(obj,image,K,D)
    if not ok:raise ValueError("Target pose unavailable")
    Rcw=cv2.Rodrigues(r)[0]
    return Rcw.T,(-Rcw.T@t).ravel()


def export_openvins_chain(intrinsics,alignment):
    """OpenVINS pinned config convention: camera coordinates -> IMU coordinates."""
    import json
    if not alignment.get('independent_validation_passed'):raise ValueError('Validate an independent recording before estimator export')
    Rci=np.asarray(alignment['R_camera_from_imu']);tci=np.asarray(alignment['t_camera_from_imu_m'])
    transform=np.eye(4);transform[:3,:3]=Rci.T;transform[:3,3]=-Rci.T@tci
    K=np.array(intrinsics['camera_matrix']);D=intrinsics['dist_coeffs'][:4]
    lines=['%YAML:1.0','cam0:','  T_imu_cam:']
    lines+=['    - '+json.dumps(row) for row in transform.tolist()]
    lines+=['  camera_model: pinhole','  distortion_model: '+intrinsics['distortion_model'],
            '  distortion_coeffs: '+json.dumps(D),
            '  intrinsics: '+json.dumps([float(K[0,0]),float(K[1,1]),float(K[0,2]),float(K[1,2])]),
            '  resolution: '+json.dumps(intrinsics['image_size']),
            '  timeshift_cam_imu: '+str(alignment['camera_to_imu_time_shift_s']),
            '  cam_overlaps: []','  rostopic: /cam0/image_raw']
    return '\n'.join(lines)+'\n'
