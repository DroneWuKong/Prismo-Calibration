#!/usr/bin/env python3
"""
wingman_vio.py — AI Wingman VIO process for Orqa DTK APB

Tier 1 optical flow visual odometry. Runs on the APB's i.MX8M Plus Linux side.
Reads camera frames via V4L2 (OpenCV VideoCapture), runs FAST corner detection
and Lucas-Kanade sparse optical flow, fuses with IMU from PX4 HIGHRES_IMU via
MAVLink UDP, and publishes OdometryEstimate as JSON UDP to wingman-apb on :14560.

Architecture:
  Camera (V4L2) ─────► FAST corners ─► LK optical flow ─┐
                                                          ├─► Complementary filter ─► UDP :14560
  PX4 MAVLink (UDP:14541) ─► HIGHRES_IMU pre-integration ┘

Two-thread design:
  Thread 1 (camera): capture → detect → track → compute frame-to-frame translation
  Thread 2 (mavlink): receive HIGHRES_IMU → pre-integrate → notify camera thread

Complementary filter:
  Camera handles translation estimate (low freq, absolute).
  Gyroscope handles rotation (high freq, relative).
  Camera-derived yaw corrects gyro yaw drift when feature count is sufficient.

Usage:
  python3 wingman_vio.py [--camera /dev/video0] [--imu-port 14541] [--out-port 14560]

Dependencies (Yocto Kirkstone apt / pip):
  python3-opencv  (cv2)     — FAST, pyrLK, VideoCapture
  python3-numpy             — array math
  (no other deps — runs on bare i.MX8M Plus Linux)

"It just runs programs." — Number 5, Short Circuit
"""

import argparse
import json
import math
import socket
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────

DEFAULT_CAMERA   = "/dev/video2"  # MIPI downward-facing; /dev/video0 = analog capture
DEFAULT_APB_URL  = "http://127.0.0.1:8088"  # for baro altitude poll
DEFAULT_IMU_PORT = 14541          # PX4 HIGHRES_IMU stream (mavlink-router output)
DEFAULT_OUT_PORT = 14560          # wingman-apb ExternalVioReceiver
DEFAULT_OUT_HOST = "127.0.0.1"

CAMERA_WIDTH     = 320            # QVGA — fast enough for FAST+LK on Cortex-A53
CAMERA_HEIGHT    = 240
CAMERA_FPS       = 20

MAX_FEATURES     = 50             # FAST corners to track
MIN_FEATURES     = 8              # below this → re-detect, confidence drops
LK_WIN_SIZE      = (15, 15)       # Lucas-Kanade window
LK_MAX_LEVEL     = 3              # pyramid levels
LK_CRITERIA      = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)
FAST_THRESHOLD   = 20
FAST_GRID        = (4, 4)         # spatial grid for uniform feature distribution

# Camera focal length profiles (pixels). Calibrate with checkerboard for accuracy.
CAMERA_PROFILES = {
    "generic_wide": 165,   # 1/4" ~2.1mm 160° — generic downward cam
    "runcam_nano":  180,   # RunCam Nano 3
    "ov9281":       420,   # ArduCam OV9281 global shutter (recommended)
    "orqa_csi":     250,   # Orqa DTK CSI estimate
}
DEFAULT_CAMERA_PROFILE = "generic_wide"

COMP_ALPHA       = 0.98           # complementary filter: gyro weight (high freq)
G_MPS2           = 9.80665

# ── MAVLink HIGHRES_IMU (msg 105) minimal parser ──────────────────────────────

MAVLINK_STX_V2   = 0xFD
MSG_HIGHRES_IMU  = 105
MSG_HEARTBEAT    = 0

def _parse_mavlink_v2(buf: bytes):
    """
    Parse one MAVLink v2 frame from a UDP datagram.
    Returns list of (msgid, payload_bytes) tuples found in the buffer.
    """
    results = []
    i = 0
    while i < len(buf) - 10:
        if buf[i] != MAVLINK_STX_V2:
            i += 1
            continue
        payload_len = buf[i+1]
        frame_len   = 10 + payload_len + 2
        if i + frame_len > len(buf):
            break
        msgid = buf[i+7] | (buf[i+8] << 8) | (buf[i+9] << 16)
        payload = buf[i+10: i+10+payload_len]
        results.append((msgid, payload))
        i += frame_len
    return results

def parse_highres_imu(payload: bytes):
    """
    HIGHRES_IMU payload layout (62 bytes):
      uint64 time_usec
      float  xacc, yacc, zacc
      float  xgyro, ygyro, zgyro
      float  xmag, ymag, zmag
      float  abs_pressure, diff_pressure, pressure_alt, temperature
      uint16 fields_updated
      uint8  id
    """
    if len(payload) < 62:
        return None
    ts, xacc, yacc, zacc, xgyro, ygyro, zgyro = struct.unpack_from('<Qffffff', payload, 0)
    return {
        "ts": ts,
        "gyro": (xgyro, ygyro, zgyro),   # rad/s, filtered
        "accel": (xacc, yacc, zacc),      # m/s²
    }

# ── Quaternion math ───────────────────────────────────────────────────────────

def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ], dtype=np.float64)   # w,x,y,z

def quat_mul(q, r):
    w0,x0,y0,z0 = q
    w1,x1,y1,z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

def quat_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-8 else np.array([1.,0.,0.,0.])

def quat_rotate(q, v):
    """Rotate vector v by quaternion q (Hamilton convention w,x,y,z)."""
    qv = np.array([0., v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    r  = quat_mul(quat_mul(q, qv), qc)
    return r[1:]

def quat_to_euler(q):
    w,x,y,z = q
    sinr = 2*(w*x + y*z)
    cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)
    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

# ── FAST feature detection with spatial grid ──────────────────────────────────

def detect_features_gridded(gray, max_features=MAX_FEATURES, grid=FAST_GRID,
                              threshold=FAST_THRESHOLD):
    """
    Detect FAST corners distributed across a spatial grid to avoid clustering.
    Returns (N, 1, 2) float32 array suitable for calcOpticalFlowPyrLK.
    """
    h, w = gray.shape
    gh, gw = grid
    cell_h, cell_w = h // gh, w // gw
    per_cell = max(1, max_features // (gh * gw))
    pts = []
    detector = cv2.FastFeatureDetector_create(threshold=threshold, nonmaxSuppression=True)
    for r in range(gh):
        for c in range(gw):
            roi = gray[r*cell_h:(r+1)*cell_h, c*cell_w:(c+1)*cell_w]
            kps = detector.detect(roi)
            kps = sorted(kps, key=lambda k: -k.response)[:per_cell]
            for kp in kps:
                pts.append([[kp.pt[0] + c*cell_w, kp.pt[1] + r*cell_h]])
    if not pts:
        return None
    return np.array(pts, dtype=np.float32)

# ── Camera intrinsics (APB — calibrate empirically) ──────────────────────────

# Approximate HFOV ~70° for a typical 1/4" sensor at QVGA
# fx = (W/2) / tan(HFOV/2)
HFOV_RAD = math.radians(70.0)
FX = (CAMERA_WIDTH / 2) / math.tan(HFOV_RAD / 2)
FY = FX   # assume square pixels
CX = CAMERA_WIDTH  / 2.0
CY = CAMERA_HEIGHT / 2.0

def pixel_flow_to_angular(dx_px, dy_px):
    """Convert pixel flow to angular velocity approximation (rad/frame)."""
    return dx_px / FX, dy_px / FY

# ── IMU thread ────────────────────────────────────────────────────────────────

class ImuReceiver(threading.Thread):
    """
    Receives HIGHRES_IMU from PX4 via MAVLink UDP on a dedicated port.
    Maintains a rolling buffer of recent samples for VIO fusion.
    """
    def __init__(self, port=DEFAULT_IMU_PORT, maxlen=500):
        super().__init__(daemon=True, name="imu-recv")
        self.port   = port
        self.lock   = threading.Lock()
        self.buf    = deque(maxlen=maxlen)
        self.latest = None

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.5)
        print(f"[ImuReceiver] listening on UDP:{self.port}")
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                for msgid, payload in _parse_mavlink_v2(data):
                    if msgid == MSG_HIGHRES_IMU:
                        imu = parse_highres_imu(payload)
                        if imu:
                            with self.lock:
                                self.buf.append(imu)
                                self.latest = imu
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[ImuReceiver] error: {e}")

    def drain_since(self, since_us):
        """Return samples newer than since_us in timestamp order."""
        with self.lock:
            return [s for s in self.buf if s["ts"] > since_us]

    def get_latest(self):
        with self.lock:
            return self.latest

# ── VIO main loop ─────────────────────────────────────────────────────────────

class MjpegHttpReader:
    """
    Minimal MJPEG-over-HTTP client. Drop-in replacement for cv2.VideoCapture
    when the source is a `multipart/x-mixed-replace; boundary=…` stream.

    Matches the cv2.VideoCapture API surface that wingman_vio.py uses:
      - isOpened() -> bool
      - read() -> (ret: bool, frame: ndarray)
      - set(prop, value) -> bool   (no-op; HTTP source is fixed-format)
      - release() -> None
    """
    def __init__(self, url: str):
        import urllib.request
        self._url = url
        self._closed = False
        self._stream = None
        self._buf = b""
        self._boundary = None
        try:
            req = urllib.request.Request(url)
            self._stream = urllib.request.urlopen(req, timeout=5)
            ctype = self._stream.headers.get("Content-Type", "")
            if "boundary=" in ctype:
                self._boundary = b"--" + ctype.split("boundary=", 1)[1].strip().encode()
            else:
                self._boundary = b"--frame"
        except Exception as e:
            print(f"[MjpegHttpReader] open failed: {e}", flush=True)
            self._stream = None

    def isOpened(self) -> bool:
        return self._stream is not None and not self._closed

    def set(self, prop, value) -> bool:
        return False  # source format is fixed by the producer

    def read(self):
        import numpy as _np
        if not self.isOpened():
            return False, None
        try:
            # Find next boundary, then Content-Length header, then exactly that
            # many bytes for the JPEG payload.
            while self._boundary not in self._buf:
                chunk = self._stream.read(8192)
                if not chunk:
                    return False, None
                self._buf += chunk
            i = self._buf.find(self._boundary)
            self._buf = self._buf[i + len(self._boundary):]
            # Read headers up to the blank line.
            while b"\r\n\r\n" not in self._buf:
                chunk = self._stream.read(2048)
                if not chunk:
                    return False, None
                self._buf += chunk
            hdr_end = self._buf.find(b"\r\n\r\n")
            headers = self._buf[:hdr_end].decode(errors="ignore")
            self._buf = self._buf[hdr_end + 4:]
            length = None
            for line in headers.split("\r\n"):
                if line.lower().startswith("content-length:"):
                    length = int(line.split(":", 1)[1].strip())
                    break
            if length is None:
                return False, None
            while len(self._buf) < length:
                chunk = self._stream.read(min(65536, length - len(self._buf)))
                if not chunk:
                    return False, None
                self._buf += chunk
            jpeg = self._buf[:length]
            self._buf = self._buf[length:]
            arr = _np.frombuffer(jpeg, dtype=_np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return False, None
            return True, frame
        except Exception as e:
            print(f"[MjpegHttpReader] read error: {e}", flush=True)
            return False, None

    def release(self):
        self._closed = True
        if self._stream is not None:
            try: self._stream.close()
            except Exception: pass


class WingmanVio:
    def __init__(self, camera_dev, imu_port, out_port, out_host):
        # Yocto Scarthgap's cv2 is GStreamer-only (no FFmpeg backend) and the
        # souphttpsrc/multipartdemux path raced with detector-side encoder
        # readiness, throwing "Internal data stream error" depending on the
        # caller's import graph. Use a small in-process MJPEG reader for HTTP
        # sources — same multipart-boundary parse cv2 would do, deterministic,
        # no GStreamer pipeline magic.
        if camera_dev.startswith(("http://", "https://")):
            self.cap = MjpegHttpReader(camera_dev)
        else:
            self.cap = cv2.VideoCapture(camera_dev)
        self.imu         = ImuReceiver(imu_port)
        self.out_sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out_addr    = (out_host, out_port)

        self.prev_gray   = None
        self.prev_pts    = None
        self.prev_ts_us  = 0

        # Baro altitude for optical flow scale (polled from wingman-apb)
        self.baro_alt_m: float = 2.0
        self._baro_lock = threading.Lock()

        # State — NED frame
        self.pos         = np.zeros(3)  # metres
        self.vel         = np.zeros(3)
        self.orientation = quat_from_euler(0., 0., 0.)  # w,x,y,z

        # Complementary filter state
        self.gyro_euler  = np.zeros(3)  # roll, pitch, yaw

        self.feature_count = 0
        self.last_imu_ts   = 0
        self.frame_count   = 0

        self._configure_camera()

    def _configure_camera(self):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
        # Prefer grayscale (GREY / Y800 — saves colour conversion overhead)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB,  0)
        print(f"[WingmanVio] camera {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}")

    def _imu_pre_integrate(self, from_us, to_us):
        """
        Pre-integrate IMU samples between two camera frame timestamps.
        Returns (delta_orientation_quat, delta_velocity_ned, avg_gyro_rad_s).
        """
        samples = self.imu.drain_since(from_us)
        samples = [s for s in samples if s["ts"] <= to_us]
        if not samples:
            return None, np.zeros(3), np.zeros(3)

        dq = np.array([1., 0., 0., 0.])
        dv = np.zeros(3)
        prev_ts = from_us
        gyro_sum = np.zeros(3)

        for s in sorted(samples, key=lambda x: x["ts"]):
            dt = (s["ts"] - prev_ts) / 1e6
            if dt <= 0 or dt > 0.1:
                prev_ts = s["ts"]
                continue

            gx, gy, gz = s["gyro"]
            gyro_sum += np.array([gx, gy, gz])

            # Integrate rotation
            omega = np.array([gx, gy, gz])
            angle = np.linalg.norm(omega) * dt
            if angle > 1e-8:
                axis = omega / np.linalg.norm(omega)
                sa, ca = math.sin(angle/2), math.cos(angle/2)
                dq_step = np.array([ca, axis[0]*sa, axis[1]*sa, axis[2]*sa])
                dq = quat_norm(quat_mul(dq, dq_step))

            # Integrate velocity (gravity-removed, rotated to NED)
            ax, ay, az = s["accel"]
            g_body = quat_rotate(
                np.array([self.orientation[0], -self.orientation[1],
                          -self.orientation[2], -self.orientation[3]]),
                np.array([0., 0., G_MPS2])
            )
            a_body = np.array([ax - g_body[0], ay - g_body[1], az - g_body[2]])
            a_ned  = quat_rotate(self.orientation, a_body)
            dv    += a_ned * dt
            prev_ts = s["ts"]

        n = len(samples)
        avg_gyro = gyro_sum / n if n > 0 else np.zeros(3)
        return dq, dv, avg_gyro

    def step(self, gray, ts_us):
        """Process one camera frame. Returns OdometryEstimate dict or None."""
        now_us = ts_us if ts_us > 0 else int(time.monotonic() * 1e6)
        dt_s   = (now_us - self.prev_ts_us) / 1e6 if self.prev_ts_us > 0 else 1/CAMERA_FPS

        # ── IMU pre-integration ────────────────────────────────────────────────
        dq_imu, dv_imu, avg_gyro = self._imu_pre_integrate(self.prev_ts_us, now_us)

        # Apply IMU rotation update
        if dq_imu is not None:
            self.orientation = quat_norm(quat_mul(self.orientation, dq_imu))
        self.vel += dv_imu

        # ── Optical flow ───────────────────────────────────────────────────────
        flow_translation = None
        n_tracked = 0

        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) >= MIN_FEATURES:
            # Track previous features
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None,
                winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL, criteria=LK_CRITERIA
            )
            mask    = status.ravel() == 1
            good_p0 = self.prev_pts[mask]
            good_p1 = next_pts[mask]
            n_tracked = int(mask.sum())

            if n_tracked >= MIN_FEATURES:
                # Compute median flow vector (robust to outliers)
                flow_vecs  = good_p1 - good_p0
                median_flow = np.median(flow_vecs, axis=0)
                dx_px, dy_px = float(median_flow[0][0]), float(median_flow[0][1])

                # Angular equivalent
                dtheta_x, dtheta_y = pixel_flow_to_angular(dx_px, dy_px)

                # Scale: v_ms = (pixel_delta/focal_px) × baro_alt_m × fps
                # Use baro altitude — dead-reckoned pos[2] drifts and
                # causes scale error that compounds over time.
                with self._baro_lock:
                    alt_m = max(0.3, self.baro_alt_m)
                dn =  dtheta_y * alt_m   # forward (N) component
                de = -dtheta_x * alt_m   # rightward (E) component

                # Rotate from body to NED
                euler = quat_to_euler(self.orientation)
                yaw   = euler[2]
                flow_translation = np.array([
                    dn * math.cos(yaw) - de * math.sin(yaw),
                    dn * math.sin(yaw) + de * math.cos(yaw),
                    0.
                ])

                # Complementary filter: blend camera yaw with gyro yaw
                # Camera gives us yaw indirectly via heading-compensated flow
                # Gyro gives high-freq rate
                gyro_euler = quat_to_euler(self.orientation)
                r_cam, p_cam, y_cam = gyro_euler  # after IMU integration above
                # For roll/pitch, trust gyro. For yaw, slight camera correction if many features.
                if n_tracked > 20:
                    # No direct yaw from flow — just trust gyro for now
                    pass
                self.orientation = quat_norm(quat_from_euler(r_cam, p_cam, y_cam))

            self.prev_pts = good_p1.reshape(-1, 1, 2) if n_tracked > MIN_FEATURES else None

        # Re-detect if features dropped
        if self.prev_pts is None or len(self.prev_pts) < MIN_FEATURES:
            pts = detect_features_gridded(gray)
            self.prev_pts = pts
            n_tracked = len(pts) if pts is not None else 0

        # ── Position integration ───────────────────────────────────────────────
        if flow_translation is not None:
            # Complementary: camera translation (position) + IMU velocity (for smoothing)
            cam_vel_estimate = flow_translation / dt_s if dt_s > 0 else np.zeros(3)
            self.vel[:2] = COMP_ALPHA * self.vel[:2] + (1 - COMP_ALPHA) * cam_vel_estimate[:2]
        self.pos += self.vel * dt_s

        # ── Confidence ────────────────────────────────────────────────────────
        conf = min(1.0, n_tracked / MAX_FEATURES)
        if self.imu.get_latest() is None:
            conf *= 0.5   # no IMU = lower confidence even if camera is fine

        # ── Store state ───────────────────────────────────────────────────────
        self.prev_gray    = gray.copy()
        self.prev_ts_us   = now_us
        self.feature_count = n_tracked
        self.frame_count  += 1

        return {
            "ts":   now_us,
            "px":   round(float(self.pos[0]),  4),
            "py":   round(float(self.pos[1]),  4),
            "pz":   0.0,  # Do NOT publish optical-flow altitude — let baro handle it.
            # With EKF2_EV_CTRL=2 (PX4) or EK3_SRC1_POSZ=1 (ArduPilot),
            # the EKF ignores our z entirely. Publishing drifting z here
            # actively fights altitude hold if params are not set correctly.
            "qw":   round(float(self.orientation[0]), 6),
            "qx":   round(float(self.orientation[1]), 6),
            "qy":   round(float(self.orientation[2]), 6),
            "qz":   round(float(self.orientation[3]), 6),
            "vx":   round(float(self.vel[0]),  4),
            "vy":   round(float(self.vel[1]),  4),
            "vz":   round(float(self.vel[2]),  4),
            "wx":   round(float(avg_gyro[0]),  5),
            "wy":   round(float(avg_gyro[1]),  5),
            "wz":   round(float(avg_gyro[2]),  5),
            "conf": round(conf,                4),
            "tier": 1,
            "nf":   n_tracked,
            "src":  "LK_OPTICAL_FLOW"
        }

    def publish(self, est: dict):
        msg = (json.dumps(est) + "\n").encode()
        try:
            self.out_sock.sendto(msg, self.out_addr)
        except Exception as e:
            print(f"[WingmanVio] publish error: {e}")


    def _poll_baro_alt(self, apb_url: str):
        """Poll wingman-apb /status for baro altitude AGL every 500ms."""
        import urllib.request as _req, json as _j
        while True:
            try:
                with _req.urlopen(f"{apb_url}/status", timeout=2) as r:
                    d = _j.loads(r.read().decode())
                    agl = d.get("guidance",{}).get("altitudeAglM")
                    if agl and float(agl) > 0.3:
                        with self._baro_lock: self.baro_alt_m = max(0.3, float(agl))
                        time.sleep(0.5); continue
                    pos_d = d.get("odometry",{}).get("posD")
                    if pos_d and abs(float(pos_d)) > 0.3:
                        with self._baro_lock: self.baro_alt_m = abs(float(pos_d))
            except Exception: pass
            time.sleep(0.5)
    def run(self, apb_url: str = DEFAULT_APB_URL):
        if not self.cap.isOpened():
            print(f"[WingmanVio] ERROR: camera not opened — check device")
            return

        threading.Thread(target=self._poll_baro_alt, args=(apb_url,),
                         daemon=True, name="vio-baro").start()
        print(f"[WingmanVio] baro alt polling → {apb_url}")
        self.imu.start()
        print(f"[WingmanVio] running — publishing to {self.out_addr}")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("[WingmanVio] camera read failed — retrying")
                time.sleep(0.1)
                continue

            # Convert to grayscale if colour frame
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # Get approximate frame timestamp from monotonic clock
            ts_us = int(time.monotonic() * 1e6)

            # Use PX4 IMU timestamp if available (better sync)
            imu = self.imu.get_latest()
            if imu:
                ts_us = imu["ts"]

            est = self.step(gray, ts_us)
            if est:
                self.publish(est)

                # Periodic status
                if self.frame_count % (CAMERA_FPS * 5) == 0:
                    print(f"[WingmanVio] frame={self.frame_count} "
                          f"nf={self.feature_count} conf={est['conf']:.2f} "
                          f"pos=({est['px']:.1f},{est['py']:.1f},{est['pz']:.1f})m")

    def cleanup(self):
        self.cap.release()
        self.out_sock.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Wingman VIO — APB optical flow odometry")
    parser.add_argument("--camera",   default=DEFAULT_CAMERA,   help="V4L2 device (default: /dev/video0)")
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT, help="MAVLink HIGHRES_IMU UDP port")
    parser.add_argument("--out-port", type=int, default=DEFAULT_OUT_PORT, help="Output UDP port for wingman-apb")
    parser.add_argument("--out-host", default=DEFAULT_OUT_HOST, help="Output UDP host")
    args = parser.parse_args()

    print(f"[wingman-vio] starting camera={args.camera} imu={args.imu_port} out={args.out_host}:{args.out_port}")
    vio = WingmanVio(args.camera, args.imu_port, args.out_port, args.out_host)
    try:
        vio.run(apb_url=getattr(args,"apb",DEFAULT_APB_URL))
    except KeyboardInterrupt:
        print("[wingman-vio] shutting down")
    finally:
        vio.cleanup()


if __name__ == "__main__":
    main()
