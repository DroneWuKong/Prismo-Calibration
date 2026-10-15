#!/usr/bin/env python3
"""Install a verified, staged calibration bundle on the APB; --check-only is offline."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import time
import urllib.request
HERE=Path(__file__).resolve().parent

def run(*args):subprocess.run(args,check=True)

def main():
    manifest=json.loads((HERE/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        if Path(name).is_absolute() or '..' in Path(name).parts:raise ValueError('Invalid package path')
        if hashlib.sha256((HERE/name).read_bytes()).hexdigest()!=digest:raise ValueError('Package hash mismatch: '+name)
    binary=(HERE/'detector/wingman-detector').read_bytes()
    if binary[:4]!=b'\x7fELF' or int.from_bytes(binary[18:20],'little')!=183:raise ValueError('Detector must be an aarch64 ELF')
    if '--check-only' in sys.argv:print('Package checks passed');return
    if os.geteuid()!=0:raise ValueError('Installation requires the APB administrator')
    sys.path[:0]=[str(HERE),'/data/wingman']
    from calibration_app import verify_disarmed
    verify_disarmed()
    router=Path('/etc/mavlink-router/main.conf');config=router.read_text()
    endpoint='[UdpEndpoint calibration]'
    if endpoint not in config:
        updated=config+'\n'+endpoint+'\nMode = Normal\nAddress = 127.0.0.1\nPort = 14568\n'
    else:
        block=config.split(endpoint,1)[1].split('[',1)[0]
        if '14568' not in block or '127.0.0.1' not in block:raise ValueError('Existing calibration endpoint has a different configuration')
        updated=config
    target=Path('/data/wingman');unit=Path('/etc/systemd/system/wingman-calibration.service')
    backup=target/'rollback'/('guided-calibration-'+secrets.token_hex(5));backup.mkdir(parents=True)
    destinations={name:(unit if name=='wingman-calibration.service' else target/name) for name in manifest['files']}
    originals={}
    for name,dest in destinations.items():
        if dest.exists():
            original=backup/name;original.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dest,original);originals[name]=original
        else:originals[name]=None
    shutil.copy2(router,backup/'router.conf')
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    had_vio=subprocess.run(['systemctl','is-active','--quiet','wingman-vio']).returncode==0
    had_calibration=subprocess.run(['systemctl','is-active','--quiet','wingman-calibration']).returncode==0
    subprocess.run(['systemctl','stop','wingman-calibration'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    run('systemctl','stop','wingman-detector')
    try:
        for name,dest in destinations.items():
            dest.parent.mkdir(parents=True,exist_ok=True);pending=dest.with_name(dest.name+'.calibration-pending')
            shutil.copyfile(HERE/name,pending);pending.chmod(0o755 if name.startswith('detector/') else 0o644);os.replace(pending,dest)
        router.write_text(updated)
        run('systemctl','daemon-reload');run('systemctl','restart','mavlink-router');run('systemctl','start','wingman-detector')
        if had_vio:run('systemctl','start','wingman-vio')
        from wingman_vio import MjpegHttpReader
        deadline=time.monotonic()+45
        while True:
            reader=None
            try:
                verify_disarmed();reader=MjpegHttpReader('http://127.0.0.1:8082/')
                ok,frame=reader.read()
                if ok and reader.timestamp_us and 0<=time.monotonic_ns()//1000-reader.timestamp_us<250000:break
            except (ValueError,OSError):pass
            finally:
                if reader:reader.release()
            if time.monotonic()>deadline:raise ValueError('Timestamped camera feed or fresh FC telemetry did not return')
            time.sleep(1)
        run('systemctl','enable','--now','wingman-calibration')
        deadline=time.monotonic()+15
        while True:
            try:
                with urllib.request.urlopen('http://127.0.0.1:8091/api/status',timeout=2) as response:state=json.load(response)
                if state.get('state')=='idle':break
            except (OSError,ValueError):pass
            if time.monotonic()>deadline:raise ValueError('Calibration application failed to start')
            time.sleep(1)
        for name,digest in manifest['files'].items():
            if hashlib.sha256(destinations[name].read_bytes()).hexdigest()!=digest:raise ValueError('Installed hash mismatch')
        receipt={**manifest,'installed':True,'backup':str(backup),'timestamped_camera_verified':True,'flight_qualified':False}
        (target/'guided-calibration-install.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
    except BaseException:
        subprocess.run(['systemctl','stop','wingman-calibration','wingman-detector'])
        for name,dest in destinations.items():
            if originals[name]:shutil.copy2(originals[name],dest)
            elif dest.exists():dest.unlink()
        shutil.copy2(backup/'router.conf',router)
        if originals['wingman-calibration.service'] is None:
            subprocess.run(['systemctl','disable','wingman-calibration'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        run('systemctl','daemon-reload');run('systemctl','restart','mavlink-router');run('systemctl','start','wingman-detector')
        if had_vio:run('systemctl','start','wingman-vio')
        if had_calibration:run('systemctl','start','wingman-calibration')
        raise
if __name__=='__main__':main()
