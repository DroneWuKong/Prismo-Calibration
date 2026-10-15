"""Guided metric-target camera/IMU capture and independent validation."""
import hashlib
import json
import time
import cv2
import numpy as np
from wingman_vio import MjpegHttpReader
from calibration_imu import ImuRecorder
from camera_imu_calibration import target_pose,solve_alignment,validate_alignment,export_openvins_chain


def run_motion(owner,validate=False):
    reader=None;imu=None;poses={'times':[],'rotations':[],'positions':[]};previous=None
    duration=30 if validate else 60
    written_bytes=0
    try:
        reader=MjpegHttpReader(owner.camera)
        ok,frame=reader.read()
        if not ok or not reader.timestamp_us:
            raise ValueError('Camera capture timestamps are missing. Update the APB detector before camera/IMU alignment.')
        imu=ImuRecorder();imu.start()
        started=time.monotonic();heartbeat=started
        folder=owner.motion_path
        while not owner.stop_event.is_set():
            elapsed=time.monotonic()-started
            if time.monotonic()-owner.last_client>8:raise ValueError('Browser disconnected: recording stopped')
            if imu.error:raise ValueError(imu.error)
            if elapsed>=duration:break
            if time.monotonic()-heartbeat>10:
                from calibration_app import verify_disarmed
                verify_disarmed();heartbeat=time.monotonic()
            ok,gray=reader.read()
            if not ok:raise ValueError('Camera stream interrupted; keep USB and aircraft power connected')
            stamp=reader.timestamp_us
            if not stamp or abs(time.monotonic_ns()/1000-stamp)>250000:raise ValueError('Camera timestamp is missing or stale')
            if gray.ndim==3:gray=cv2.cvtColor(gray,cv2.COLOR_BGR2GRAY)
            if [gray.shape[1],gray.shape[0]]!=owner.result['image_size']:raise ValueError('Camera format changed; repeat camera calibration')
            found,corners=cv2.findChessboardCorners(gray,(7,5),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
            view=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
            if found:
                corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.01))
                R,p=target_pose(corners,owner.result)
                if previous is not None:
                    angle=np.linalg.norm(cv2.Rodrigues(previous.T@R)[0])
                    if angle>.35:raise ValueError('Checkerboard pose flipped or motion was too fast. Keep the board upright and move gently.')
                previous=R
                poses['times'].append(stamp/1e6);poses['rotations'].append(R);poses['positions'].append(p)
                cv2.drawChessboardCorners(view,(7,5),corners,True)
                # Keep raw corner observations and timestamps for replay/audit.
                with (folder/'corners.jsonl').open('a') as f:f.write(json.dumps({'capture_monotonic_us':stamp,'corners':corners.tolist()})+'\n')
            else:previous=None
            # Raw gray frames allow independent corner re-extraction and replay.
            ok,raw=cv2.imencode('.jpg',gray)
            if not ok:raise ValueError('Camera frame encoding failed')
            written_bytes+=len(raw)
            if written_bytes>32*1024*1024:raise ValueError('Recording storage limit reached')
            (folder/f'frame-{stamp}.jpg').write_bytes(raw.tobytes())
            _,jpeg=cv2.imencode('.jpg',view)
            instruction=('Hold still with the entire board visible' if elapsed<5 else
                         'Gently mix roll, pitch and yaw; vary speed and direction' if elapsed<20 else
                         'Keep the board visible. Move gently left, right, up and down while tilting')
            with owner.lock:
                owner.preview=jpeg.tobytes();owner.motion_seconds=int(elapsed);owner.motion_duration=duration
                owner.message=instruction if found else 'Bring the whole board back into view'
            if len(poses['times'])>1800:raise ValueError('Camera recording limit exceeded')
        if owner.stop_event.is_set():return
        imu.stop();recorded=imu.data();imu=None
        if recorded['timesync_responses']<5:raise ValueError('Not enough verified camera/IMU clock synchronization')
        serial={'times':poses['times'],'rotations':[p.tolist() for p in poses['rotations']],
                'positions':[p.tolist() for p in poses['positions']]}
        (folder/'poses.json').write_text(json.dumps(serial));(folder/'imu.json').write_text(json.dumps(recorded))
        with owner.lock:owner.state='alignment_solving';owner.message='Checking camera/IMU timing, rotation and metric geometry'
        result=validate_alignment(owner.alignment,poses,recorded) if validate else solve_alignment(poses,recorded)
        if owner.stop_event.is_set():return
        result['camera_calibration_id']=owner.result['calibration_id']
        result['imu_intrinsics_source']='FC corrected RAW_IMU; IMU scale/nonorthogonality not independently estimated'
        result['capture_timing_validated']=bool(validate)
        result['alignment_id']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()[:20]
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        if validate:(folder/'kalibr_imucam_chain.yaml').write_text(export_openvins_chain(owner.result,result))
        with owner.lock:
            owner.alignment=result;owner.state='alignment_complete' if validate else 'alignment_candidate'
            owner.message=('Camera/IMU calibration passed the independent movement check. VIO activation still requires estimator validation.' if validate else
                           'Alignment estimated. Run the separate movement check to verify it.')
    except Exception as e:
        with owner.lock:owner.state='error';owner.message=str(e)
    finally:
        if reader:reader.release()
        if imu:imu.stop()
        with owner.lock:
            owner.running=False
            if owner.stop_event.is_set():owner.state='stopped';owner.message='Recording stopped; existing calibration unchanged'
