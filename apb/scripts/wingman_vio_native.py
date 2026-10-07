#!/usr/bin/env python3
"""Bounded shared-camera/raw-IMU adapter for a pinned headless OpenVINS candidate.

Requires an actual estimator YAML and validated frame/timing manifest. No ROS
runtime, guessed intrinsics, barometric translation scale, or FC pose feedback.
"""
import argparse
import collections
import json
import math
import os
import socket
import subprocess
import threading
import time
import uuid
from pathlib import Path
import numpy as np
from fusion_control import FusionControl
from wingman_vio import MjpegHttpReader


class ClockSync:
    def __init__(self):
        self.offset_us = None
        self.last_sync = 0.0
        self.pending_ns = None

    def response(self, remote_ns, echoed_ns, received_ns):
        if echoed_ns != self.pending_ns or remote_ns <= 0 or not 0 <= received_ns - echoed_ns <= 10_000_000:
            return False
        offset = ((echoed_ns + received_ns) / 2 - remote_ns) / 1000
        if self.offset_us is not None and abs(offset - self.offset_us) > 20_000:
            raise ValueError("FC clock reset or discontinuity")
        self.offset_us = offset if self.offset_us is None else self.offset_us * 0.9 + offset * 0.1
        self.last_sync = time.monotonic()
        self.pending_ns = None
        return True

    def map(self, remote_us):
        if self.offset_us is None or time.monotonic() - self.last_sync > 5:
            return None
        mapped = int(remote_us + self.offset_us)
        age = time.monotonic_ns() // 1000 - mapped
        return mapped if 0 <= age < 150_000 else None


def quaternion_matrix(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(matrix):
    # Stable branch across 180-degree rotations.
    k = np.array([[matrix[0,0]-matrix[1,1]-matrix[2,2], matrix[1,0]+matrix[0,1], matrix[2,0]+matrix[0,2], matrix[2,1]-matrix[1,2]],
                  [matrix[1,0]+matrix[0,1], matrix[1,1]-matrix[0,0]-matrix[2,2], matrix[2,1]+matrix[1,2], matrix[0,2]-matrix[2,0]],
                  [matrix[2,0]+matrix[0,2], matrix[2,1]+matrix[1,2], matrix[2,2]-matrix[0,0]-matrix[1,1], matrix[1,0]-matrix[0,1]],
                  [matrix[2,1]-matrix[1,2], matrix[0,2]-matrix[2,0], matrix[1,0]-matrix[0,1], np.trace(matrix)]]) / 3
    values, vectors = np.linalg.eigh(k)
    xyzw = vectors[:, np.argmax(values)]
    q = xyzw[[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def transform_estimate(estimate, manifest, reset):
    aligned = manifest.get("north_alignment_validated") is True
    rotation = np.asarray(manifest["world_to_ned"] if aligned else np.eye(3), dtype=float)
    if rotation.shape != (3,3) or not np.isfinite(rotation).all() or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError("invalid calibrated world-to-NED rotation")
    position = rotation @ np.array([estimate[k] for k in ("px", "py", "pz")])
    velocity = rotation @ np.array([estimate[k] for k in ("vx", "vy", "vz")])
    quaternion = matrix_quaternion(rotation @ quaternion_matrix([estimate[k] for k in ("qw", "qx", "qy", "qz")]))
    covariance = np.zeros((6,6))
    indices = np.triu_indices(6)
    covariance[indices] = estimate["covariance"]
    covariance += np.triu(covariance, 1).T
    transform = np.zeros((6,6))
    transform[:3,:3] = rotation
    transform[3:,3:] = rotation
    covariance = transform @ covariance @ transform.T
    if not np.isfinite(covariance).all() or np.min(np.linalg.eigvalsh(covariance)) < -1e-8:
        raise ValueError("invalid estimator covariance")
    variance = max(covariance[0,0], covariance[1,1])
    result = {**estimate, "version": 2, "kind": "odometry", "fields": ["position", "velocity", "attitude"],
              "clock": "apb_monotonic", "frame": 1 if aligned else 4, "velocity_frame": 1 if aligned else 4,
              "origin": manifest["origin_id"], "calibration": manifest["calibration_id"],
              "metric_validated": all(manifest.get(k) is True for k in ("metric_validated", "capture_timing_validated", "north_alignment_validated")),
              "dependencies": ["camera", "raw_imu"], "reset": reset, "tier": 2, "src": "OPENVINS_NATIVE",
              "conf": min(.95, 1 / (1 + math.sqrt(max(0, variance)))), "covariance": covariance[indices].tolist()}
    result.update(zip(("px", "py", "pz"), position.tolist()))
    result.update(zip(("vx", "vy", "vz"), velocity.tolist()))
    result.update(zip(("qw", "qx", "qy", "qz"), quaternion.tolist()))
    return result


def bootstrap_alignment(estimate, gps, attitude, manifest, now_us, armed, sensors):
    """Gauge initialization only: fresh raw GNSS plus healthy compass heading, while disarmed."""
    if armed or gps is None or attitude is None or (sensors & 0x24) != 0x24:
        return None
    if abs(estimate["ts"] - gps["ts"]) > 250_000 or abs(estimate["ts"] - attitude["ts"]) > 100_000:
        return None
    if not 0 <= now_us - gps["ts"] < 500_000 or not 0 <= now_us - attitude["ts"] < 250_000:
        return None
    if gps["fix"] < 3 or not 6 <= gps["satellites"] < 255 or gps["h_acc_m"] <= 0:
        return None
    try:
        lat0, lon0, alt0 = (float(manifest[k]) for k in ("origin_lat", "origin_lon", "origin_alt_m"))
    except (KeyError,TypeError,ValueError):
        return None
    if not (-85 <= lat0 <= 85 and -180 <= lon0 <= 180 and math.isfinite(alt0)):
        return None
    base = np.array([[0,1,0],[1,0,0],[0,0,-1]], dtype=float)
    local_body = base @ quaternion_matrix([estimate[k] for k in ("qw","qx","qy","qz")])
    local_roll = math.atan2(local_body[2,1],local_body[2,2])
    local_pitch = math.asin(max(-1,min(1,-local_body[2,0])))
    if abs(local_roll-attitude["roll"]) > .15 or abs(local_pitch-attitude["pitch"]) > .15:
        return None
    yaw = attitude["yaw"] - math.atan2(local_body[1,0],local_body[0,0])
    rotation = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]]) @ base
    n = math.radians(gps["lat"]-lat0)*6371000
    e = math.radians(gps["lon"]-lon0)*6371000*math.cos(math.radians(lat0))
    d = alt0-gps["alt_m"]
    if math.hypot(n,e) > 100_000: return None
    offset = np.array([n,e,d])-rotation @ np.array([estimate[k] for k in ("px","py","pz")])
    return {"north_alignment_validated":True,"origin_id":manifest["origin_id"],
            "calibration_id":manifest["calibration_id"],"world_to_ned":rotation.tolist(),
            "position_offset_ned_m":offset.tolist(),
            "anchor_variance_xy_m2":gps["h_acc_m"]**2 + (20*abs(estimate["ts"]-gps["ts"])/1e6)**2 + 4.0,
            "anchor_variance_z_m2":max(gps["v_acc_m"],10.0)**2,
            "anchor_yaw_variance_rad2":.15**2, "anchor_source":"raw_gnss_and_compass_initialization"}

class NativeSession:
    def __init__(self, args, manifest, reset, link):
        self.args, self.manifest, self.reset, self.link = args, manifest, reset, link
        self.session_id = str(uuid.uuid4())
        Path(args.session_status).write_text(json.dumps({"session_id": self.session_id, "calibration_id": manifest["calibration_id"]}))
        self.process = subprocess.Popen([args.binary, args.config], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self.frame = None
        self.stopped = threading.Event()
        self.reader = None
        self.sync = ClockSync()
        self.sample_times = collections.deque(maxlen=512)
        self.last_imu = 0
        self.selected_imu = None
        self.fc_system = None
        self.armed = True
        self.gps = self.attitude = None
        self.gps_history = collections.deque(maxlen=20)
        self.attitude_history = collections.deque(maxlen=200)
        self.sensor_health = 0
        self.sensor_health_time = 0
        self.alignment = None
        self.started = time.monotonic()
        self.error = None
        self.recording = None
        if args.recording:
            requested = Path(args.recording)
            recording = requested.with_name(requested.stem + "." + self.session_id + requested.suffix)
            self.recording = open(recording, "xb")
            recording.with_suffix(recording.suffix + ".json").write_text(json.dumps({
                "session_id": self.session_id, "calibration": manifest,
                "openvins_commit": "69488123ed9362dd44b6f28e7f4680abbff1442b",
                "camera": args.camera, "timestamp_kind": "dequeue", "protocol": "native-I-C-v1"
            }))
        self.output = threading.Thread(target=self._publish, daemon=True)
        self.camera = threading.Thread(target=self._capture, daemon=True)
        self.output.start(); self.camera.start()

    def _write(self, payload):
        self.process.stdin.write(payload)
        if self.recording is not None:
            if self.recording.tell() + len(payload) <= 128 * 1024 * 1024:
                self.recording.write(payload)
            else:
                self.recording.close()
                self.recording = None
                print("[native-vio] recording cap reached; navigation continues", flush=True)

    def _capture(self):
        while not self.stopped.is_set():
            try:
                self.reader = MjpegHttpReader(self.args.camera)
                while not self.stopped.is_set():
                    ok, image = self.reader.read()
                    if not ok: break
                    ts = self.reader.timestamp_us
                    if ts and 0 <= time.monotonic_ns() // 1000 - ts < 200_000:
                        with self.frame_lock:
                            if self.frame is None:
                                self.frame = (ts, image)
            finally:
                if self.reader: self.reader.release()
            self.stopped.wait(1)

    def _publish(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for line in self.process.stdout:
                if self.stopped.is_set(): break
                marker = line.find(b"PRISMO_ODOMETRY ")
                if marker < 0: continue
                line = line[marker+len(b"PRISMO_ODOMETRY "):]
                try:
                    if len(self.sample_times) < 2: continue
                    rate = (len(self.sample_times)-1)*1e6 / (self.sample_times[-1]-self.sample_times[0])
                    if rate < self.manifest.get("min_imu_hz", 100): continue
                    # Global alignment is per initialization, never reused from an old calibration file.
                    manifest = {**self.manifest, "north_alignment_validated": False, "origin_id": "local:" + self.session_id}
                    try:
                        alignment = json.loads(Path(self.args.alignment).read_text())
                        if alignment.get("session_id") == self.session_id and alignment.get("calibration_id") == self.manifest["calibration_id"]:
                            manifest.update(alignment)
                    except (OSError, ValueError):
                        pass
                    local = json.loads(line)
                    if self.alignment is None and time.monotonic()-self.sensor_health_time < 3:
                        gps = min(list(self.gps_history),key=lambda p:abs(p["ts"]-local["ts"]),default=None)
                        attitude = min(list(self.attitude_history),key=lambda p:abs(p["ts"]-local["ts"]),default=None)
                        self.alignment = bootstrap_alignment(local,gps,attitude,self.manifest,
                            time.monotonic_ns()//1000,self.armed,self.sensor_health)
                    if self.alignment is not None:
                        manifest.update(self.alignment)
                    estimate = transform_estimate(local, manifest, self.reset)
                    if manifest.get("north_alignment_validated") is True:
                        shift = np.asarray(manifest.get("position_offset_ned_m", [0,0,0]), dtype=float)
                        if shift.shape != (3,) or not np.isfinite(shift).all(): raise ValueError("invalid origin offset")
                        for key, value in zip(("px","py","pz"),shift):
                            estimate[key] += float(value)
                        for index,key in ((0,"anchor_variance_xy_m2"),(6,"anchor_variance_xy_m2"),(11,"anchor_variance_z_m2"),(20,"anchor_yaw_variance_rad2")):
                            variance = float(manifest.get(key,0))
                            if not math.isfinite(variance) or variance < 0: raise ValueError("invalid anchor uncertainty")
                            estimate["covariance"][index] += variance
                    sock.sendto(json.dumps(estimate, allow_nan=False).encode(), ("127.0.0.1", 14565))
                    Path(self.args.marker).write_text(str(time.monotonic()))
                except (ValueError, KeyError, OSError, TypeError) as error:
                    self.error = str(error)

    def ingest(self, message):
        kind = message.get_type()
        if kind == "HEARTBEAT" and message.autopilot == 3:
            if self.fc_system is None: self.fc_system = message.get_srcSystem()
            if message.get_srcSystem() == self.fc_system: self.armed = bool(message.base_mode & 128)
        if self.fc_system is None or message.get_srcSystem() != self.fc_system:
            return
        if kind == "SYS_STATUS":
            self.sensor_health = message.onboard_control_sensors_enabled & message.onboard_control_sensors_health
            self.sensor_health_time = time.monotonic()
            return
        if kind == "GPS_RAW_INT":
            stamp = self.sync.map(message.time_usec)
            if stamp:
                self.gps = {"ts":stamp,"lat":message.lat/1e7,"lon":message.lon/1e7,"alt_m":message.alt/1000,
                            "fix":message.fix_type,"satellites":message.satellites_visible,
                            "h_acc_m":getattr(message,"h_acc",0)/1000,"v_acc_m":getattr(message,"v_acc",0)/1000}
                self.gps_history.append(self.gps)
            return
        if kind == "ATTITUDE":
            stamp = self.sync.map(message.time_boot_ms*1000)
            if stamp:
                self.attitude = {"ts":stamp,"roll":message.roll,"pitch":message.pitch,"yaw":message.yaw}
                self.attitude_history.append(self.attitude)
            return
        if message.get_type() == "TIMESYNC" and message.tc1 > 0:
            self.sync.response(message.tc1, message.ts1, time.monotonic_ns())
            return
        kind = message.get_type()
        if kind not in ("HIGHRES_IMU", "RAW_IMU", "SCALED_IMU"): return
        # Bind one raw sensor stream per session; never integrate duplicates from multiple messages.
        if self.selected_imu is not None and kind != self.selected_imu: return
        remote_us = message.time_boot_ms * 1000 if kind == "SCALED_IMU" else message.time_usec
        stamp = self.sync.map(remote_us)
        if stamp is None or stamp <= self.last_imu: return
        if kind == "HIGHRES_IMU" and (message.fields_updated & 0x3F) != 0x3F: return
        if self.last_imu and stamp-self.last_imu > 100_000: raise ValueError("raw IMU gap")
        self.selected_imu = kind
        self.last_imu = stamp
        self.sample_times.append(stamp)
        if kind == "HIGHRES_IMU":
            values = (message.xacc, message.yacc, message.zacc, message.xgyro, message.ygyro, message.zgyro)
        else:
            values = (message.xacc*.00980665, message.yacc*.00980665, message.zacc*.00980665,
                      message.xgyro*.001, message.ygyro*.001, message.zgyro*.001)
        if not all(math.isfinite(x) for x in values): raise ValueError("invalid IMU")
        with self.lock:
            self._write(("I " + str(stamp/1e6) + " " + " ".join(map(str,values)) + "\n").encode())
            with self.frame_lock:
                frame = self.frame
                if frame and stamp >= frame[0] + 100_000:
                    self.frame = None
                else: frame = None
            if frame and 0 <= stamp-frame[0] < 200_000:
                ts, image = frame
                self._write(f"C {ts/1e6} {image.shape[0]} {image.shape[1]}\n".encode() + image.tobytes())
            self.process.stdin.flush()

    def close(self):
        self.stopped.set()
        if self.reader: self.reader.release()
        self.process.terminate()
        try: self.process.wait(timeout=2)
        except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        self.output.join(timeout=1)
        self.camera.join(timeout=1)
        self.process.stdin.close(); self.process.stdout.close()
        if self.recording is not None: self.recording.close()
        Path(self.args.marker).unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="/data/wingman/wingman-vio-native")
    parser.add_argument("--config", default="/nvram/vio/estimator_config.yaml")
    parser.add_argument("--manifest", default="/nvram/vio/manifest.json")
    parser.add_argument("--camera", default="http://127.0.0.1:8082/")
    parser.add_argument("--recording", help="Optional bounded native input recording (128 MiB cap)")
    parser.add_argument("--alignment", default="/run/wingman-vio-alignment.json")
    parser.add_argument("--session-status", default="/run/wingman-vio-session.json")
    parser.add_argument("--marker", default="/run/wingman-vio-native.timestamp")
    args = parser.parse_args()
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    control = FusionControl()
    reset = 0
    while True:
        if not control.enabled("vio") or not all(Path(p).exists() for p in (args.binary, args.config, args.manifest)):
            time.sleep(.5); continue
        session = None
        link = None
        try:
            manifest = json.loads(Path(args.manifest).read_text())
            if not manifest.get("calibration_id") or not manifest.get("origin_id"):
                raise ValueError("calibration and origin identity required")
            link = mavutil.mavlink_connection("udpin:127.0.0.1:14566", source_system=191, source_component=197)
            session = NativeSession(args, manifest, reset, link)
            next_sync = 0
            while control.enabled("vio") and session.process.poll() is None:
                now = time.monotonic()
                if now >= next_sync:
                    session.sync.pending_ns = time.monotonic_ns()
                    link.mav.timesync_send(0, session.sync.pending_ns)
                    next_sync = now + 1
                message = link.recv_match(blocking=True, timeout=.02)
                if message is not None: session.ingest(message)
                if now-session.started > 5 and not session.sample_times:
                    raise ValueError("waiting for synchronized raw IMU")
        except (OSError, ValueError, BrokenPipeError) as error:
            print(f"[native-vio] waiting/restarting: {error}", flush=True)
        finally:
            if session: session.close()
            if link: link.close()
            reset = (reset+1) % 256
        time.sleep(1)


if __name__ == "__main__": main()
