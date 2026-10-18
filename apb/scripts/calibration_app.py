#!/usr/bin/env python3
"""Guided camera calibration. Does not arm, write FC parameters or activate VIO."""
import argparse
import hashlib
import io
import json
import math
import secrets
import shutil
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
import cv2
import numpy as np
if __package__:
    from .wingman_vio import MjpegHttpReader
    from .calibration_workspace import Workspace
    from .orqa_setup import normalize_setup, capabilities
else:
    from wingman_vio import MjpegHttpReader
    from calibration_workspace import Workspace
    from orqa_setup import normalize_setup, capabilities

BOARD = (7, 5)  # inner corners; displayed board is 8 by 6 squares
STEPS = ["Point the camera at the whole board", "Move a little closer", "Move a little farther away",
         "Let the board appear toward the left of the camera view", "Let it appear toward the right",
         "Let it appear toward the top", "Let it appear toward the bottom",
         "Tilt the drone gently left", "Tilt it gently right", "Tip the nose gently down",
         "Tip the nose gently up", "Rotate slightly and show the whole board"]

def object_points(square_mm):
    points = np.zeros((BOARD[0]*BOARD[1], 3), np.float64)
    points[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * square_mm / 1000
    return points

def fit_camera(objects, images, size, model):
    if model == "equidistant":
        obj = [p.reshape(1,-1,3).astype(np.float64) for p in objects]
        img = [p.reshape(1,-1,2).astype(np.float64) for p in images]
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, D, rs, ts = cv2.fisheye.calibrate(obj, img, size, None, None, flags=flags,
                    criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,1e-7))
    else:
        rms,K,D,rs,ts = cv2.calibrateCamera([p.astype(np.float32) for p in objects],
                         [p.astype(np.float32) for p in images],size,None,None,flags=cv2.CALIB_FIX_K3)
    if not np.isfinite(K).all() or not np.isfinite(D).all() or min(K[0,0], K[1,1]) <= 0:
        raise ValueError("Invalid camera matrix")
    if not 0 < K[0,2] < size[0] or not 0 < K[1,2] < size[1]:
        raise ValueError("Principal point outside the image")
    return float(rms), K, D, rs, ts

def heldout_error(objects, images, K, D, model):
    errors=[]
    for obj,img in zip(objects,images):
        if model == "equidistant":
            normalized=cv2.fisheye.undistortPoints(img.reshape(-1,1,2).astype(np.float64),K,D)
            ok,r,t=cv2.solvePnP(obj,normalized,np.eye(3),None)
            if not ok: raise ValueError("Held-out pose could not be solved")
            projected,_=cv2.fisheye.projectPoints(obj.reshape(-1,1,3),r,t,K,D)
        else:
            ok,r,t=cv2.solvePnP(obj,img,K,D)
            if not ok: raise ValueError("Held-out pose could not be solved")
            projected,_=cv2.projectPoints(obj,r,t,K,D)
        errors.extend(np.linalg.norm(projected.reshape(-1,2)-img.reshape(-1,2),axis=1).tolist())
    return float(np.sqrt(np.mean(np.square(errors))))

def solve(images, size, square_mm, model="auto"):
    if len(images)<24: raise ValueError("Collect at least 24 varied views")
    objects=[object_points(square_mm) for _ in images]
    hold=[i for i in range(len(images)) if i%5==4]
    train=[i for i in range(len(images)) if i not in hold]
    candidates=[]
    for candidate in (["radtan","equidistant"] if model=="auto" else [model]):
        try:
            _,K,D,_,_=fit_camera([objects[i] for i in train],[images[i] for i in train],size,candidate)
            error=heldout_error([objects[i] for i in hold],[images[i] for i in hold],K,D,candidate)
            candidates.append((error,candidate))
        except (cv2.error,ValueError): pass
    if not candidates: raise ValueError("Lens fit failed; collect more varied, sharper views")
    error,model=min(candidates)
    rms,K,D,rs,_=fit_camera(objects,images,size,model)
    normals=[cv2.Rodrigues(r)[0][:,2] for r in rs]
    spread=np.ptp(np.array(normals),axis=0)
    if rms>.8 or error>1.: raise ValueError("Views do not agree well enough; try slower motion and less screen glare")
    if min(spread[0],spread[1])<.2: raise ValueError("More left/right AND up/down tilt needed")
    return {"schema":1,"camera_matrix":K.tolist(),"dist_coeffs":D.ravel().tolist(),
            "image_size":list(size),"distortion_model":model,"rms_error":rms,
            "heldout_rms_px":error,"n_frames":len(images),"square_mm":square_mm,
            "board_size":list(BOARD),"intrinsics_validated":True,
            "camera_imu_calibrated":False,"vio_ready":False,
            "capture_timing_validated":False}

class Session:
    def __init__(self, output, camera):
        self.output=Path(output);self.camera=camera;self.lock=threading.RLock()
        self.running=False;self.preview=None;self.result=None;self.alignment=None;self.motion_phase=None;self.motion_seconds=0;self.motion_duration=0;self.images=[];self.coverage=set()
        self.message="Set the screen size, then start";self.state="idle";self.path=None
        self.device_id=hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()[:16] if Path('/etc/machine-id').exists() else 'unknown-device'
        self.token=secrets.token_urlsafe(32);self.last_attempt=0
        self.worker=None;self.stop_event=threading.Event();self.last_client=time.monotonic()
        self.workspace=Workspace(output)
    def status(self):
        with self.lock:
            self.last_client=time.monotonic()
            return {"state":self.state,"message":self.message,"count":len(self.images),"target":24,
                    "coverage":len(self.coverage),"coverage_target":8,
                    "instruction":STEPS[min(len(self.images)//2,len(STEPS)-1)],"result":self.result,
                    "alignment":self.alignment,"motion_phase":self.motion_phase,"motion_seconds":self.motion_seconds,"motion_duration":self.motion_duration}
    def start(self, config):
        setup=normalize_setup(config.get('setup'),config.get('device_profile','prismo_apb'))
        if not capabilities(setup)['guided_capture']:
            raise ValueError('This Orqa setup uses recording imports. Select a Prismo APB setup for guided capture.')
        config={**config,'setup':setup}
        square=float(config.get("square_mm",0))
        if not math.isfinite(square) or not 10<=square<=40:raise ValueError("Square size must be 10–40 mm")
        if config.get("props_removed") is not True or config.get("screen_verified") is not True:
            raise ValueError("Confirm props removed and screen size checked")
        with self.lock:
            if self.running:raise ValueError("A capture is already running")
            # Fresh FC heartbeat gate from the same tested bench installer.
            verify_disarmed()
            self.output.mkdir(parents=True,exist_ok=True)
            if shutil.disk_usage(self.output).free < 32*1024*1024:
                raise ValueError("Not enough storage. Download existing recordings first.")
            if len(list(self.output.glob("*-*")))>=20:
                raise ValueError("Recording limit reached. Archive previous sessions before continuing.")
            self.path=self.output/(time.strftime("%Y%m%d-%H%M%S")+"-"+secrets.token_hex(3))
            self.path.mkdir(parents=True,exist_ok=False)
            self.config=dict(config);self.motion_phase=None;self.alignment=None;self.images=[];self.coverage=set();self.result=None;self.last_attempt=0
            self.stop_event=threading.Event();self.running=True;self.state="capturing";self.last_client=time.monotonic()
            (self.path/'screen.json').write_text(json.dumps(config,indent=2))
            self.worker=threading.Thread(target=self.capture,daemon=True);self.worker.start()
    def capture(self):
        reader=None;previous=None;last_saved=0;size=None;last_heartbeat=time.monotonic();started=time.monotonic()
        try:
            reader=MjpegHttpReader(self.camera);reader._decode_period_s=.25
            while not self.stop_event.is_set():
                if time.monotonic()-self.last_client>8 or time.monotonic()-started>300:
                    with self.lock:self.message="Capture stopped: browser disconnected or session timed out";self.state="stopped"
                    break
                if time.monotonic()-last_heartbeat>10:
                    verify_disarmed();last_heartbeat=time.monotonic()
                ok,gray=reader.read()
                if not ok:
                    with self.lock:self.message="Waiting for the APB camera stream"
                    reader.release();self.stop_event.wait(1);reader=MjpegHttpReader(self.camera);reader._decode_period_s=.25
                    continue
                if gray.ndim==3:gray=cv2.cvtColor(gray,cv2.COLOR_BGR2GRAY)
                actual=(gray.shape[1],gray.shape[0])
                if size is not None and actual!=size:raise ValueError("Camera resolution changed; start again")
                size=actual
                found,corners=cv2.findChessboardCorners(gray,BOARD,cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
                view=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
                if found:
                    corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.01))
                    cv2.drawChessboardCorners(view,BOARD,corners,True)
                    blur=cv2.Laplacian(gray,cv2.CV_64F).var()
                    motion=float(np.sqrt(np.mean((corners-previous)**2))) if previous is not None else float('inf')
                    novel=not self.images or min(float(np.sqrt(np.mean((corners-p)**2))) for p in self.images)>5
                    with self.lock:
                        self.message=("Hold still briefly" if motion>1.5 else "Move or tilt to a new view" if not novel else "Image too soft—hold still or adjust distance" if blur<35 else "Board found")
                    if motion<=1.5 and novel and blur>=35 and time.monotonic()-last_saved>.65:
                        with self.lock:
                            self.images.append(corners.copy());last_saved=time.monotonic()
                            for x,y in corners.reshape(-1,2):self.coverage.add((min(3,max(0,int(x/size[0]*4))),min(2,max(0,int(y/size[1]*3)))))
                            index=len(self.images)
                            cv2.imwrite(str(self.path/f"view-{index:03}.jpg"),gray)
                            with (self.path/'observations.jsonl').open('a') as f:
                                f.write(json.dumps({"receipt_monotonic_us":time.monotonic_ns()//1000,
                                  "capture_monotonic_us":getattr(reader,'timestamp_us',0),"corners":corners.tolist()})+'\n')
                            self.message="Captured. "+STEPS[min(index//2,len(STEPS)-1)]
                    previous=corners.copy()
                else:
                    previous=None
                    with self.lock:self.message="Show all 8 × 6 squares, with the white border visible"
                success,jpeg=cv2.imencode('.jpg',view)
                if success:
                    with self.lock:self.preview=jpeg.tobytes()
                with self.lock:count=len(self.images)
                if count>=24 and count>=self.last_attempt+4 and len(self.coverage)>=8:
                    self.last_attempt=count
                    with self.lock:self.state="solving";self.message="Checking lens fit against unused views"
                    try:
                        result=solve(self.images,size,float(self.config['square_mm']))
                        if self.stop_event.is_set():break
                        result['camera_source']=self.camera
                        result['device_id']=self.device_id
                        result['setup']=self.config['setup']
                        result['calibration_id']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()[:20]
                        (self.path/'camera-calibration.json').write_text(json.dumps(result,indent=2))
                        with self.lock:self.result=result;self.state="complete";self.message="Camera calibration saved. Camera/IMU alignment is next."
                        break
                    except (cv2.error,ValueError) as e:
                        with self.lock:self.state="capturing";self.message=str(e)
                if count>=48:raise ValueError("More varied views are needed. Restart and include both tilt directions.")
        except Exception as e:
            with self.lock:self.state="error";self.message=str(e)
        finally:
            if reader:reader.release()
            with self.lock:
                self.running=False
                if self.state not in ('complete','error'):self.state="stopped";self.message="Capture stopped; existing calibration unchanged"
    def start_alignment(self,config,validate=False):
        if __package__:
            from .calibration_motion import run_motion
        else:
            from calibration_motion import run_motion
        with self.lock:
            if self.running:raise ValueError("A recording is already running")
            if not self.result or not self.result.get('intrinsics_validated'):
                raise ValueError("Complete camera calibration first")
            if config.get('setup') is not None:
                setup=normalize_setup(config['setup'],config.get('device_profile','prismo_apb'))
                if setup!=self.config.get('setup'):
                    raise ValueError('Equipment setup changed. Return to the calibrated setup or repeat lens calibration.')
            if validate and self.alignment is None:raise ValueError("Record an alignment candidate first")
            if config.get('props_removed') is not True or config.get('screen_verified') is not True:
                raise ValueError("Confirm screen scale and props removed")
            if float(config.get('square_mm',0))!=float(self.result['square_mm']) or abs(float(config.get('pixels_per_mm',0))-float(self.config['pixels_per_mm']))>.001:
                raise ValueError("Board size changed: restore the previous size or repeat camera calibration")
            verify_disarmed()
            if shutil.disk_usage(self.path).free<64*1024*1024:raise ValueError('At least 64 MB free is required for a motion recording')
            self.motion_phase='validation' if validate else 'alignment'
            self.motion_seconds=0;self.motion_duration=30 if validate else 60
            self.motion_path=self.path/(self.motion_phase+'-'+secrets.token_hex(3))
            self.motion_path.mkdir(exist_ok=False)
            self.stop_event=threading.Event();self.last_client=time.monotonic();self.running=True
            self.state='alignment_recording';self.message='Hold still with the entire board visible'
            self.worker=threading.Thread(target=run_motion,args=(self,validate),daemon=True);self.worker.start()

    def stop(self):self.stop_event.set()
    def check_setup(self,config):
        from urllib.request import urlopen
        setup=normalize_setup(config.get('setup'),config.get('device_profile','orqa_apb'))
        available=capabilities(setup)
        checks=[{'name':'Recording imports','status':'ready','message':'All four calibration workflows can use saved recordings.'}]
        if available['guided_capture']:
            try:
                verify_disarmed()
                checks.append({'name':'FC telemetry','status':'ready','message':'Fresh disarmed heartbeat received.'})
            except ValueError as e:
                checks.append({'name':'FC telemetry','status':'unavailable','message':str(e)})
            try:
                with urlopen(self.camera,timeout=2) as response:
                    if 'multipart/' not in response.headers.get('Content-Type',''):
                        raise ValueError('Expected the APB MJPEG camera service')
                checks.append({'name':'APB camera service','status':'ready','message':'Camera service reachable; frame quality and timing are checked during capture.'})
            except (OSError,ValueError) as e:
                checks.append({'name':'APB camera service','status':'unavailable','message':str(e)})
        return {'setup':setup,'capabilities':available,'checks':checks,
                'guided_ready':available['guided_capture'] and all(c['status']=='ready' for c in checks),
                'firmware_identity':'Operator-selected; not detected from the controller'}
    def export(self):
        with self.lock:
            if self.path is None:raise ValueError("No recording yet")
            buf=io.BytesIO()
            with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
                for p in self.path.rglob("*"):
                    if p.is_file():z.write(p,str(p.relative_to(self.path)))
            return buf.getvalue()

def verify_disarmed():
    # Read the existing APB telemetry consumer. Do not claim a GCS UDP endpoint.
    from urllib.request import urlopen
    try:
        with urlopen('http://127.0.0.1:8088/status',timeout=2) as response:
            status=json.load(response)
        with urlopen('http://127.0.0.1:8088/hangar/preflight',timeout=2) as response:
            report=json.load(response)
        connected=next((c for c in report.get('checks',[]) if c.get('id')=='fc_connected'),{})
        if connected.get('status')!='PASS':
            raise ValueError("No fresh flight-controller heartbeat. Check aircraft power and telemetry.")
        if status.get('flight',{}).get('armed') is not False:
            raise ValueError("Disarm the aircraft before calibration")
    except (OSError, json.JSONDecodeError) as e:
        raise ValueError("APB telemetry is unavailable. Check aircraft power and telemetry.") from e


class Handler(BaseHTTPRequestHandler):
    def send(self,code,data,kind='application/json'):
        if isinstance(data,dict):data=json.dumps(data).encode()
        self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        route=urlsplit(self.path).path
        if route=='/':self.send(200,Path(__file__).with_name('calibration_home.html').read_bytes(),'text/html; charset=utf-8')
        elif route=='/camera':self.send(200,self.server.ui.read_bytes(),'text/html; charset=utf-8')
        elif route=='/assets/calibration_home.js':self.send(200,Path(__file__).with_name('calibration_home.js').read_bytes(),'text/javascript; charset=utf-8')
        elif route=='/api/workspace/catalog':self.send(200,self.server.session.workspace.catalog())
        elif route=='/api/workspace/history':self.send(200,{'sessions':self.server.session.workspace.history()})
        elif route.startswith('/api/workspace/export/'):
            try:self.send(200,self.server.session.workspace.export(route.rsplit('/',1)[-1]),'application/zip')
            except ValueError as e:self.send(404,{'error':str(e)})
        elif route.startswith('/api/workspace/example/'):
            if __package__:
                from .calibration_examples import example
            else:
                from calibration_examples import example
            try:self.send(200,example(route.rsplit('/',1)[-1]))
            except ValueError as e:self.send(404,{'error':str(e)})
        elif route=='/api/status':self.send(200,self.server.session.status())
        elif route=='/api/token':self.send(200,{'token':self.server.session.token})
        elif route=='/api/preview':
            image=self.server.session.preview
            self.send(200,image,'image/jpeg') if image else self.send(404,{'error':'No frame yet'})
        elif route=='/api/export':
            try:self.send(200,self.server.session.export(),'application/zip')
            except ValueError as e:self.send(409,{'error':str(e)})
        else:self.send(404,{'error':'Not found'})
    def do_POST(self):
        origin=self.headers.get('Origin','')
        if urlsplit(origin).netloc!=self.headers.get('Host') or self.headers.get('X-Calibration-Token')!=self.server.session.token:
            self.send(403,{'error':'Open this tool directly in your browser'});return
        try:
            route=urlsplit(self.path).path
            length=int(self.headers.get('Content-Length','0'))
            limit=8*1024*1024 if route=='/api/workspace/calibrate' else 8192
            if not 0<length<=limit:raise ValueError("Invalid request size")
            config=json.loads(self.rfile.read(length))
            if not isinstance(config,dict):raise ValueError("Expected settings object")
            if route=='/api/workspace/calibrate':
                result=self.server.session.workspace.calibrate(config.get('recording'),config.get('device_profile'),config.get('setup'))
                self.send(200,{'ok':True,'result':result});return
            elif route=='/api/workspace/check':
                self.send(200,self.server.session.check_setup(config));return
            elif route=='/api/start':self.server.session.start(config)
            elif route=='/api/align':self.server.session.start_alignment(config)
            elif route=='/api/validate':self.server.session.start_alignment(config,validate=True)
            elif route=='/api/stop':self.server.session.stop()
            else:self.send(404,{'error':'Not found'});return
            self.send(200,{'ok':True})
        except (ValueError,TypeError,OSError) as e:self.send(400,{'error':str(e)})
    def log_message(self,*args):pass

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--bind',default='127.0.0.1');parser.add_argument('--port',type=int,default=8091)
    parser.add_argument('--output',default='./sessions');parser.add_argument('--camera',default='http://127.0.0.1:8082/')
    parser.add_argument('--open',action='store_true',help='Open the calibration app in your browser')
    parser.add_argument('--ui',default=str(Path(__file__).with_name('calibration_app.html')));args=parser.parse_args()
    cv2.setNumThreads(1)
    server=ThreadingHTTPServer((args.bind,args.port),Handler);server.session=Session(args.output,args.camera);server.ui=Path(args.ui)
    url=f'http://{args.bind}:{server.server_port}/'
    print(f'Orqa sensor calibration: {url}',flush=True)
    if args.open:
        import webbrowser
        webbrowser.open(url)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.session.stop()
        if server.session.worker:server.session.worker.join(timeout=5)
        server.server_close()
if __name__=='__main__':main()
