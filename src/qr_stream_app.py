import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, cast
import cv2
import numpy as np
import json
import socket
import requests


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray


@dataclass
class TagPose:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    bearing_x_deg: float
    bearing_y_deg: float
    rvec: Optional[np.ndarray]
    tvec: Optional[np.ndarray]


# === NEW ===
class SnapshotCapture:
    """Polls an HTTP JPEG endpoint (e.g., http://<ip>/capture)."""
    def __init__(self, url: str, fps: float = 10.0, timeout: float = 2.0):
        self.url = url
        self.delay = 1.0 / max(0.5, float(fps))
        self.timeout = timeout
        self.sess = requests.Session()

    def isOpened(self):
        return True

    def read(self):
        t0 = time.time()
        try:
            r = self.sess.get(self.url, timeout=self.timeout)
            r.raise_for_status()
            arr = np.frombuffer(r.content, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                time.sleep(self.delay)
                return False, None
            dt = time.time() - t0
            if dt < self.delay:
                time.sleep(self.delay - dt)
            return True, img
        except Exception:
            time.sleep(self.delay)
            return False, None

    def get(self, prop_id: int):
        if prop_id == cv2.CAP_PROP_FPS:
            return 10.0
        return 0.0

    def set(self, prop_id: int, value: float):
        return True

    def release(self):
        self.sess.close()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time AprilTag tracker and recorder")
    p.add_argument("--url", required=False, default=0, help="HTTP/RTSP URL or /capture endpoint")
    p.add_argument("--out", required=False, default="tag_debug.mp4", help="Output video file")
    p.add_argument("--fov", type=float, default=75.0, help="Horizontal field of view (deg)")
    p.add_argument("--tag-size", type=float, default=0.1, help="AprilTag size (m)")
    p.add_argument("--tag-family", type=str, default="auto", help="AprilTag family")
    p.add_argument("--display", action="store_true", help="Show window")
    p.add_argument("--no-display", dest="display", action="store_false", help="No window")
    p.add_argument("--record", action="store_true", help="Record video")
    p.add_argument("--no-record", dest="record", action="store_false", help="No record")
    p.add_argument("--max-width", type=int, default=1280, help="Resize width")
    p.add_argument("--mirror", action="store_true", help="Mirror display")
    p.add_argument("--font-scale", type=float, default=0.6, help="Text scale")
    p.add_argument("--auto-exposure", choices=["on", "off"], default="on", help="Auto exposure")
    p.add_argument("--exposure", type=float, default=-5.0, help="Manual exposure")
    p.add_argument("--brightness", type=float, default=0.5, help="Brightness")
    p.add_argument("--smooth-alpha", type=float, default=0.2, help="Smoothing")
    p.add_argument("--poll-fps", type=float, default=10.0, help="Polling rate for /capture streams")
    # === NEW ===
    p.add_argument("--udp", type=str, default=None,
                   help="Optional UDP host:port to send pose JSON (e.g., 127.0.0.1:5005)")
    p.set_defaults(display=True, record=True)
    args = p.parse_args(argv)
    if isinstance(args.url, str):
        try:
            args.url = int(args.url)
        except ValueError:
            pass
    return args


def compute_intrinsics(w, h, fov):
    hfov = math.radians(fov)
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    dist = np.zeros((5, 1), dtype=np.float32)
    return CameraIntrinsics(fx, fy, cx, cy, dist)


def poly_area_signed(pts):
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def order_corners(pts):
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    ordered = np.array([tl, tr, br, bl], dtype=np.float32)
    if poly_area_signed(ordered) > 0:
        ordered = ordered[::-1].copy()
    return ordered


def refine_corners(gray, pts):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    pts_ref = pts.astype(np.float32).reshape(-1, 1, 2)
    try:
        cv2.cornerSubPix(gray, pts_ref, (5, 5), (-1, -1), criteria)
    except cv2.error:
        return pts
    return pts_ref.reshape(-1, 2)


def solve_tag_pose(img_pts, intr, tag_size_m, last_rvec=None, last_tvec=None, smooth_alpha=0.2):
    s = tag_size_m / 2.0
    obj_pts = np.array([
        [-s, -s, 0.0],
        [s, -s, 0.0],
        [s, s, 0.0],
        [-s, s, 0.0],
    ], dtype=np.float32)

    K = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], dtype=np.float32)
    img_pts_ud = cv2.undistortPoints(img_pts.reshape(-1, 1, 2), K, intr.dist, P=K).reshape(-1, 2)

    success, rvec, tvec = False, None, None
    try:
        if hasattr(cv2, 'solvePnPGeneric') and hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE'):
            ret = cv2.solvePnPGeneric(obj_pts, img_pts_ud, K, intr.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if isinstance(ret, tuple) and len(ret) >= 3:
                rvecs = ret[1]
                tvecs = ret[2]
                if rvecs and tvecs:
                    candidates = []
                    for rv, tv in zip(rvecs, tvecs):
                        if tv[2] > 0:
                            candidates.append((rv, tv))
                    if candidates:
                        rvec, tvec = candidates[0]
                        success = True
    except cv2.error:
        pass

    if not success:
        flags = cv2.SOLVEPNP_ITERATIVE
        try:
            success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts_ud, K, intr.dist, flags=flags)
        except cv2.error:
            success = False

    if not success or rvec is None or tvec is None:
        return None

    if last_rvec is not None:
        rvec = (1.0 - smooth_alpha) * last_rvec + smooth_alpha * rvec
    if last_tvec is not None:
        tvec = (1.0 - smooth_alpha) * last_tvec + smooth_alpha * tvec

    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        pitch = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[1, 0], R[0, 0])
    else:
        pitch = math.atan2(-R[1, 2], R[1, 1])
        yaw = math.atan2(-R[2, 0], sy)
        roll = 0.0

    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)

    cx_img = img_pts[:, 0].mean()
    cy_img = img_pts[:, 1].mean()
    Kinv = np.linalg.inv(K)
    ray = Kinv @ np.array([cx_img, cy_img, 1.0], dtype=np.float32)
    bearing_x = math.degrees(math.atan2(float(ray[0]), 1.0))
    bearing_y = math.degrees(math.atan2(float(-ray[1]), 1.0))

    return TagPose(yaw_deg, pitch_deg, roll_deg, bearing_x, bearing_y, rvec, tvec)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    # === NEW: UDP setup ===
    udp_sock = None
    udp_target = None
    if args.udp:
        try:
            host, port = args.udp.split(":")
            udp_target = (host, int(port))
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[UDP] Sending pose data to {udp_target[0]}:{udp_target[1]}")
        except Exception as e:
            print(f"[UDP] Invalid target ({args.udp}): {e}")

    # === Capture ===
    if isinstance(args.url, str) and args.url.startswith("http") and "/capture" in args.url:
        cap = SnapshotCapture(args.url, fps=args.poll_fps)
        print(f"[INFO] Using SnapshotCapture for {args.url}")
    else:
        cap = cv2.VideoCapture(args.url)

    if not cap.isOpened():
        print(f"ERROR: Unable to open video source: {args.url}")
        return 2

    grabbed, frame = cap.read()
    if not grabbed or frame is None:
        print("ERROR: Could not read first frame.")
        return 3

    if args.max_width and frame.shape[1] > args.max_width:
        scale = args.max_width / frame.shape[1]
        frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))

    h, w = frame.shape[:2]
    intr = compute_intrinsics(w, h, args.fov)

    fam_map = {
        "36h11": cv2.aruco.DICT_APRILTAG_36h11,
        "25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    }
    dictionary = cv2.aruco.getPredefinedDictionary(fam_map["36h11"])
    detector_params = cv2.aruco.DetectorParameters()
    print("AprilTag tracker initialized.")

    last_rvec, last_tvec = None, None
    frames = 0
    t0 = time.time()

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        if args.max_width and frame.shape[1] > args.max_width:
            scale = args.max_width / frame.shape[1]
            frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))
            h, w = frame.shape[:2]
            intr = compute_intrinsics(w, h, args.fov)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)
        pose = None

        if ids is not None and len(corners) > 0:
            pts = corners[0][0].astype(np.float32)
            pts = order_corners(pts)
            pts = refine_corners(gray, pts)
            pose = solve_tag_pose(pts, intr, args.tag_size, last_rvec, last_tvec, args.smooth_alpha)
            if pose:
                last_rvec, last_tvec = pose.rvec, pose.tvec
                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 255), 2)
                cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
                cv2.circle(frame, (cx, cy), 4, (255, 0, 255), -1)
                cv2.putText(frame, f"Yaw={pose.yaw_deg:+.1f}", (cx-50, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # === NEW: UDP send pose ===
        if udp_sock and udp_target:
            msg = {"timestamp": time.time(), "seen": bool(pose is not None)}
            if pose:
                msg.update({
                    "yaw_deg": pose.yaw_deg,
                    "pitch_deg": pose.pitch_deg,
                    "roll_deg": pose.roll_deg,
                    "bearing_x_deg": pose.bearing_x_deg,
                    "bearing_y_deg": pose.bearing_y_deg,
                })
            try:
                udp_sock.sendto(json.dumps(msg).encode("utf-8"), udp_target)
            except Exception:
                pass

        if args.display:
            cv2.imshow("AprilTag Tracker", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

        frames += 1
        if frames % 30 == 0:
            fps = frames / (time.time() - t0)
            print(f"FPS: {fps:.1f}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
