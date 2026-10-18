"""Local-router IMU recorder with TIMESYNC, CRC checks and bounded memory.
Uses its own loopback endpoint (:14568); never takes over a GCS endpoint.
Temporary RAW_IMU stream requests are restored when recording ends.
"""
import socket
import struct
import threading
import time
if __package__:
    from .wingman_vio_native import ClockSync
else:
    from wingman_vio_native import ClockSync

CRC={0:50,27:144,111:34,244:95}

def checksum(data):
    c=65535
    for v in data:
        v^=c&255;v^=(v<<4)&255;c=((c>>8)^(v<<8)^(v<<3)^(v>>4))&65535
    return c


def messages(data):
    i=0
    while i+12<=len(data):
        if data[i]!=253:i+=1;continue
        n=data[i+1];flags=data[i+2];length=12+n+(13 if flags&1 else 0)
        if i+length>len(data):break
        frame=data[i:i+length];i+=length
        msg=int.from_bytes(frame[7:10],'little')
        if flags&~1 or frame[5:7]!=bytes([1,1]) or msg not in CRC:continue
        if struct.unpack_from('<H',frame,10+n)[0]!=checksum(frame[1:10+n]+bytes([CRC[msg]])):continue
        yield msg,frame[10:10+n]

class ImuRecorder:
    def __init__(self):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',14568));self.sock.settimeout(.1)
        self.sync=ClockSync();self.seq=0;self.peer=None;self.error=None;self.samples=[]
        self.stopped=threading.Event();self.thread=None;self.original_interval=None;self.rate_changed=False
        self.sync_count=0;self.duplicates=0
    def send(self,msg,payload,extra):
        if self.peer is None:return
        header=bytes([253,len(payload),0,0,self.seq,191,198])+msg.to_bytes(3,'little');self.seq=(self.seq+1)%256
        self.sock.sendto(header+payload+struct.pack('<H',checksum(header[1:]+payload+bytes([extra]))),self.peer)
    def command(self,command,*params):
        values=tuple(params)+(0.,)*(7-len(params))
        self.send(76,struct.pack('<7fHBBB',*values,command,1,1,0),152)
    def start(self):
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        started=time.monotonic();next_sync=0;queried=False
        try:
            while not self.stopped.is_set():
                now=time.monotonic()
                if now-started>90:raise ValueError('IMU recording timed out')
                try:data,peer=self.sock.recvfrom(65535)
                except socket.timeout:data=b'';peer=None
                decoded=list(messages(data))
                if decoded and self.peer is None:self.peer=peer
                if self.peer and now>=next_sync:
                    self.sync.pending_ns=time.monotonic_ns()
                    self.send(111,struct.pack('<qq',0,self.sync.pending_ns),34);next_sync=now+1
                    if not queried:self.command(510,27);queried=True
                if self.peer and not self.rate_changed and now-started>2:
                    self.command(511,27,10000);self.rate_changed=True
                for msg,payload in decoded:
                    if msg==0:
                        hb=payload.ljust(9,b'\0')
                        if hb[6]&128:raise ValueError('Aircraft armed: recording stopped')
                    elif msg==244:
                        interval,mid=struct.unpack('<iH',payload.ljust(6,b'\0')[:6])
                        if mid==27 and not self.rate_changed:self.original_interval=interval
                    elif msg==111:
                        remote,echo=struct.unpack('<qq',payload.ljust(16,b'\0')[:16])
                        if self.sync.response(remote,echo,time.monotonic_ns()):self.sync_count+=1
                    elif msg==27:
                        raw=payload.ljust(29,b'\0')
                        if raw[26]!=0:continue
                        stamp,ax,ay,az,gx,gy,gz=struct.unpack_from('<Qhhhhhh',raw)
                        mapped=self.sync.map(stamp)
                        if mapped is None:continue
                        if self.samples and mapped<=self.samples[-1]['ts']:
                            self.duplicates+=1;continue
                        if len(self.samples)>=15000:raise ValueError('IMU recording limit reached')
                        self.samples.append({'ts':mapped,'gyro':[gx*.001,gy*.001,gz*.001],
                                             'accel':[ax*.00980665,ay*.00980665,az*.00980665]})
                if now-started>6 and not self.samples:raise ValueError('No synchronized IMU stream: calibration router endpoint and FC data required')
        except Exception as e:self.error=str(e)
        finally:
            if self.rate_changed:
                # Restore the reported original interval; 0 requests the FC default
                # if the firmware did not implement GET_MESSAGE_INTERVAL.
                try:self.command(511,27,self.original_interval if self.original_interval is not None else 0)
                except OSError:pass
    def stop(self):
        self.stopped.set()
        if self.thread:self.thread.join(timeout=2)
        self.sock.close()
    def data(self):
        return {'times':[s['ts']/1e6 for s in self.samples], 'gyro':[s['gyro'] for s in self.samples],
                'accel':[s['accel'] for s in self.samples], 'clock':'apb_monotonic','timesync_responses':self.sync_count}
