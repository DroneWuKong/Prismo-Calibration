import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import calibration_imu as imu
import calibration_motion as motion


def frame(mid,payload,extra):
    h=bytes([253,len(payload),0,0,0,1,1])+mid.to_bytes(3,'little')
    return h+payload+struct.pack('<H',imu.checksum(h[1:]+payload+bytes([extra])))


def test_decode_generated_pymavlink_fixture_and_crc_rejection():
    # Generated independently by pymavlink.dialects.v20.ardupilotmega.
    packet=bytes.fromhex('fd1400000001011b000040e2010000000000640038ff18fc2c0170fef401df0d')
    decoded=list(imu.messages(packet))
    assert len(decoded)==1 and decoded[0][0]==27
    values=struct.unpack('<Qhhhhhh',decoded[0][1])
    assert values==(123456,100,-200,-1000,300,-400,500)
    assert list(imu.messages(packet[:-1]+bytes([packet[-1]^1])))==[]
    assert list(imu.messages(packet[:-4]))==[]


def test_router_clock_sync_rate_restore_and_si_units():
    recorder=imu.ImuRecorder()
    router=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);router.bind(('127.0.0.1',0));router.settimeout(.005)
    commands=[];stop=threading.Event();clock_offset=123456789000
    def serve():
        while not stop.is_set():
            router.sendto(frame(0,struct.pack('<IBBBBB',0,2,3,0,3,3),50),('127.0.0.1',14568))
            stamp=(time.monotonic_ns()-clock_offset)//1000
            router.sendto(frame(27,struct.pack('<Qhhhhhhhhh',stamp,0,0,-1000,0,0,0,0,0,0),144),('127.0.0.1',14568))
            try:data,peer=router.recvfrom(1024)
            except socket.timeout:continue
            mid=int.from_bytes(data[7:10],'little');payload=data[10:10+data[1]]
            if mid==111:
                _,echo=struct.unpack('<qq',payload)
                router.sendto(frame(111,struct.pack('<qq',time.monotonic_ns()-clock_offset,echo),34),peer)
            elif mid==76:
                params=struct.unpack('<7fHBBB',payload)
                commands.append(params)
                if params[7]==510:router.sendto(frame(244,struct.pack('<iH',200000,27),95),peer)
    server=threading.Thread(target=serve,daemon=True);server.start();recorder.start()
    try:
        end=time.monotonic()+3
        while time.monotonic()<end and not recorder.rate_changed:time.sleep(.02)
        assert recorder.rate_changed
    finally:
        recorder.stop();time.sleep(.05);stop.set();server.join(timeout=1);router.close()
    data=recorder.data()
    assert data['timesync_responses']>=1 and len(data['times'])>30
    np.testing.assert_allclose(data['accel'][-1],[0,0,-9.80665])
    assert all(t2>t1 for t1,t2 in zip(data['times'],data['times'][1:]))
    interval=[c[1] for c in commands if c[7]==511]
    assert interval==[10000,200000]


def test_missing_capture_timestamp_cannot_start_imu_recording(tmp_path,monkeypatch):
    class Reader:
        timestamp_us=0
        def __init__(self,*args):pass
        def read(self):return True,np.zeros((180,320),np.uint8)
        def release(self):pass
    monkeypatch.setattr(motion,'MjpegHttpReader',Reader)
    def forbidden():raise AssertionError('IMU should not start without camera timestamps')
    monkeypatch.setattr(motion,'ImuRecorder',forbidden)
    owner=SimpleNamespace(camera='unused',lock=threading.RLock(),running=True,stop_event=threading.Event())
    motion.run_motion(owner)
    assert owner.state=='error' and 'timestamps' in owner.message
    assert not owner.running
