import importlib.util
import json
from pathlib import Path
import sys
import threading
import urllib.request
import urllib.error
import cv2
import numpy as np
import pytest

SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
import calibration_app as app


def views(model="radtan",count=30):
    rng=np.random.default_rng(5)
    K=np.array([[290.,0.,160.],[0.,295.,90.],[0.,0.,1.]])
    D=np.array([-.09,.015,.001,-.001,0.]) if model=="radtan" else np.array([-.02,.002,0.,0.])
    obj=app.object_points(25)
    images=[]
    for i in range(count):
        r=np.array([-.5+(i%6)*.2,-.5+(i//6)*.22,.15*np.sin(i)])
        t=np.array([-.075+.04*np.sin(i),-.05+.03*np.cos(i),.4+.07*(i%4)])
        if model=="radtan":pts,_=cv2.projectPoints(obj,r,t,K,D)
        else:pts,_=cv2.fisheye.projectPoints(obj.reshape(-1,1,3),r,t,K,D)
        images.append(pts+rng.normal(0,.025,pts.shape))
    return images,K


@pytest.mark.parametrize("model",["radtan","equidistant"])
def test_recovers_camera_from_diverse_synthetic_views(model):
    images,K=views(model)
    result=app.solve(images,(320,180),25,model)
    assert result["heldout_rms_px"]<.15
    np.testing.assert_allclose(np.array(result["camera_matrix"])[[0,1],[0,1]], K[[0,1],[0,1]], rtol=.03)
    assert result["intrinsics_validated"]
    assert not result["vio_ready"] and not result["camera_imu_calibrated"]


def test_rejects_too_few_views():
    with pytest.raises(ValueError,match="24"):app.solve(views()[0][:5],(320,180),25)


def test_rejects_repeated_flat_views():
    obj=app.object_points(25);K=np.array([[290.,0,160],[0,290.,90],[0,0,1.]])
    pts,_=cv2.projectPoints(obj,np.zeros(3),np.array([-.075,-.05,.5]),K,None)
    with pytest.raises((ValueError,cv2.error)):app.solve([pts.copy() for _ in range(24)],(320,180),25)


def test_rejects_inconsistent_corner_data():
    images,_=views()
    rng=np.random.default_rng(3)
    images=[p+rng.normal(0,5,p.shape) for p in images]
    with pytest.raises((ValueError,cv2.error)):app.solve(images,(320,180),25)


@pytest.mark.parametrize("square",[0,-1,100,float('nan'),float('inf')])
def test_invalid_scale_cannot_start_capture(tmp_path,square):
    session=app.Session(tmp_path,"unused")
    with pytest.raises(ValueError):session.start({"square_mm":square,"props_removed":True,"screen_verified":True})
    assert not session.running


def test_requires_human_setup_confirmations(tmp_path):
    session=app.Session(tmp_path,"unused")
    with pytest.raises(ValueError,match="Confirm"):session.start({"square_mm":25})


def test_no_fc_heartbeat_prevents_capture(tmp_path,monkeypatch):
    def fail():raise ValueError("No fresh flight-controller heartbeat")
    monkeypatch.setattr(app,"verify_disarmed",fail)
    session=app.Session(tmp_path,"unused")
    with pytest.raises(ValueError,match="heartbeat"):session.start({"square_mm":25,"props_removed":True,"screen_verified":True})
    assert list(tmp_path.iterdir())==[]


def test_http_rejects_cross_origin_mutations(tmp_path):
    server=app.ThreadingHTTPServer(('127.0.0.1',0),app.Handler)
    server.session=app.Session(tmp_path,"unused");server.ui=SCRIPTS/'calibration_app.html'
    worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        assert b'ASUS ProArt PX13' in urllib.request.urlopen(base).read()
        token=json.load(urllib.request.urlopen(base+'/api/token'))['token']
        req=urllib.request.Request(base+'/api/stop',data=b'{}',headers={'Origin':'http://other.example','X-Calibration-Token':token})
        with pytest.raises(urllib.error.HTTPError) as e:urllib.request.urlopen(req)
        assert e.value.code==403
        req=urllib.request.Request(base+'/api/stop',data=b'{}',headers={'Origin':base,'X-Calibration-Token':token})
        assert json.load(urllib.request.urlopen(req))['ok']
    finally:server.shutdown();server.server_close()


def test_export_does_not_include_other_files(tmp_path):
    import io,zipfile
    session=app.Session(tmp_path,"unused");session.path=tmp_path/'session';session.path.mkdir()
    (session.path/'screen.json').write_text('{}');(tmp_path/'private.txt').write_text('private')
    with zipfile.ZipFile(io.BytesIO(session.export())) as z:assert z.namelist()==['screen.json']


@pytest.mark.parametrize("connected,armed,expected",[("PASS",False,True),("WARN",False,False),("FAIL",False,False),("PASS",True,False),("PASS",None,False)])
def test_disarmed_check_requires_fresh_telemetry(monkeypatch,connected,armed,expected):
    import io
    def response(url,timeout):
        data={"flight":{"armed":armed}} if url.endswith('/status') else {"checks":[{"id":"fc_connected","status":connected}]}
        return io.BytesIO(json.dumps(data).encode())
    monkeypatch.setattr(urllib.request,"urlopen",response)
    if expected:app.verify_disarmed()
    else:
        with pytest.raises(ValueError):app.verify_disarmed()
