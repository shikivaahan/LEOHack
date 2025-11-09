# qr_stream_app.py
import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, cast

import cv2
import numpy as np
import requests  # NEW: used for polling /capture


# ------------------------- Data classes -------------------------

@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray  # distortion coefficients, shape (1,5) or (5,)


@dataclass
class TagPose:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    bearing_x_deg: float  # left/right from center (+ right)
    bearing_y_deg: float  # up/down from center (+ up)
    rvec: Optional[np.ndarray]
    tvec: Optional[np.ndarray]


# ------------------------- Args -------------------------

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time AprilTag tracker and recorder")
    p.add_argument("--url", required=False, default=0,
                   help="HTTP/RTSP URL, file path, or camera index (default 0). "
                        "Use http://<ip>/capture for snapshot polling if /stream is unreliable.")
    p.add_argument("--out", required=False, default="tag_debug.mp4",
                   help="Output video file (mp4/avi). Set empty to disable recording.")
    p.add_argument("--fov", type=float, default=75.0,
                   help="Horizontal field of view in degrees (default 75)")
    p.add_argument("--tag-size", type=float, default=0.1,
                   help="Physical size of AprilTag (edge length) in meters (default 0.1 m ~ 10 cm).")
    p.add_argument("--tag-family", type=str, default="auto",
                   help="AprilTag family (auto, 36h11, 25h9, 16h5). 'auto' tries to detect on initial frames.")
    p.add_argument("--display", action="store_true", help="Show live window")
    p.add_argument("--no-display", dest="display", action="store_false", help="Disable live window")
    p.add_argument("--record", action="store_true", help="Enable recording to --out file")
    p.add_argument("--no-record", dest="record", action="store_false", help="Disable recording")
    p.add_argument("--max-width", type=int, default=1280,
                   help="Resize frames to this width to keep real-time (0 = no resize)")
    p.add_argument("--mirror", action="store_true", help="Mirror display horizontally (selfie view)")
    p.add_argument("--font-scale", type=float, default=0.6, help="Overlay text scale")
    # Exposure / brightness controls
    p.add_argument("--auto-exposure", choices=["on", "off"], default="on",
                   help="Toggle camera auto-exposure (default on)")
    p.add_argument("--exposure", type=float, default=-5.0,
                   help="Manual exposure value (driver-specific, default -5.0)")
    p.add_argument("--brightness", type=float, default=0.5,
                   help="Manual brightness value in [0,1] typical range (driver-specific)")
    # Temporal smoothing
    p.add_argument("--smooth-alpha", type=float, default=0.2,
                   help="Exponential smoothing factor for rvec/tvec (0..1, default 0.2)")
    # Snapshot polling FPS (used only if --url points to /capture)
    p.add_argument("--poll-fps", type=float, default=10.0,
                   help="Polling FPS for /capture snapshot sources (default 10.0)")

    p.set_defaults(display=True, record=True)
    args = p.parse_args(argv)

    # If url looks like an int, convert for VideoCapture index
    if isinstance(args.url, str):
        try:
            args.url = int(args.url)
        except ValueError:
            pass
    return args


# ------------------------- Geometry helpers -------------------------

def compute_intrinsics(w: int, h: int, horizontal_fov_deg: float) -> CameraIntrinsics:
    hfov = math.radians(horizontal_fov_deg)
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    dist = np.zeros((5, 1), dtype=np.float32)
    return CameraIntrinsics(fx, fy, cx, cy, dist)


def poly_area_signed(pts: np.ndarray) -> float:
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def order_corners(pts: np.ndarray) -> np.ndarray:
    s = pts.sum(axis=1)  # x + y
    diff = np.diff(pts, axis=1)  # y - x
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    ordered = np.array([tl, tr, br, bl], dtype=np.float32)
    # Enforce clockwise ordering (image y down): flip if area > 0
    if poly_area_signed(ordered) > 0:
        ordered = ordered[::-1].copy()
    return ordered


def refine_corners(gray: np.ndarray, pts: np.ndarray) -> np.ndarray:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    pts_ref = pts.astype(np.float32).reshape(-1, 1, 2)
    try:
        cv2.cornerSubPix(gray, pts_ref, (5, 5), (-1, -1), criteria)
    except cv2.error:
        return pts
    return pts_ref.reshape(-1, 2)


# ------------------------- Pose -------------------------

def solve_tag_pose(
    img_pts: np.ndarray,
    intr: CameraIntrinsics,
    tag_size_m: float,
    last_rvec: Optional[np.ndarray] = None,
    last_tvec: Optional[np.ndarray] = None,
    smooth_alpha: float = 0.2,
):
    # 3D points of square in its own coord frame, centered at origin
    s = tag_size_m / 2.0
    obj_pts = np.array(
        [[-s, -s, 0.0],
         [ s, -s, 0.0],
         [ s,  s, 0.0],
         [-s,  s, 0.0]], dtype=np.float32
    )

    K = np.array([[intr.fx, 0, intr.cx],
                  [0, intr.fy, intr.cy],
                  [0, 0, 1]], dtype=np.float32)

    # Undistort image points to pixel space (P=K keeps pixel coords)
    img_pts_ud = cv2.undistortPoints(img_pts.reshape(-1, 1, 2), K, intr.dist, P=K).reshape(-1, 2)

    def reproj_err(rv: np.ndarray, tv: np.ndarray) -> float:
        proj, _ = cv2.projectPoints(obj_pts, rv, tv, K, intr.dist)
        proj = proj.reshape(-1, 2)
        return float(np.mean(np.linalg.norm(proj - img_pts, axis=1)))

    success = False
    rvec = None
    tvec = None

    # Try IPPE_SQUARE via solvePnPGeneric
    try:
        if hasattr(cv2, 'solvePnPGeneric') and hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE'):
            ret = cv2.solvePnPGeneric(obj_pts, img_pts_ud, K, intr.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if isinstance(ret, tuple) and len(ret) >= 3:
                rvecs = ret[1]
                tvecs = ret[2]
                if rvecs is not None and tvecs is not None and len(rvecs) > 0:
                    candidates = []
                    for rv, tv in zip(rvecs, tvecs):
                        rv = rv.reshape(3, 1)
                        tv = tv.reshape(3, 1)
                        if tv[2] > 0:
                            candidates.append((rv, tv, reproj_err(rv, tv)))
                    if candidates:
                        candidates.sort(key=lambda x: x[2])
                        rvec, tvec, _ = candidates[0]
                        success = True
    except cv2.error:
        success = False

    # Fallback to ITERATIVE PnP
    if not success:
        flags = cv2.SOLVEPNP_ITERATIVE
        if last_rvec is not None and last_tvec is not None:
            try:
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts_ud, K, intr.dist,
                    rvec=last_rvec, tvec=last_tvec, useExtrinsicGuess=True, flags=flags
                )
            except cv2.error:
                success = False
        if not success:
            try:
                success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts_ud, K, intr.dist, flags=flags)
            except cv2.error:
                success = False

    if not success or rvec is None or tvec is None:
        return None, None

    # Temporal smoothing
    if last_rvec is not None:
        rvec = (1.0 - smooth_alpha) * last_rvec + smooth_alpha * rvec
    if last_tvec is not None:
        tvec = (1.0 - smooth_alpha) * last_tvec + smooth_alpha * tvec

    R, _ = cv2.Rodrigues(rvec)

    # Angles
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        pitch = math.atan2(R[2, 1], R[2, 2])  # X
        yaw = math.atan2(-R[2, 0], sy)        # Y
        roll = math.atan2(R[1, 0], R[0, 0])   # Z
    else:
        pitch = math.atan2(-R[1, 2], R[1, 1])
        yaw = math.atan2(-R[2, 0], sy)
        roll = 0.0

    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)

    # Bearing via normalized camera ray from pixel to optical axis
    cx_img = img_pts[:, 0].mean()
    cy_img = img_pts[:, 1].mean()
    Kinv = np.linalg.inv(K)
    ray = Kinv @ np.array([cx_img, cy_img, 1.0], dtype=np.float32)
    bearing_x = math.degrees(math.atan2(float(ray[0]), 1.0))
    bearing_y = math.degrees(math.atan2(float(-ray[1]), 1.0))

    pose = TagPose(
        yaw_deg=yaw_deg,
        pitch_deg=pitch_deg,
        roll_deg=roll_deg,
        bearing_x_deg=bearing_x,
        bearing_y_deg=bearing_y,
        rvec=rvec,
        tvec=tvec,
    )
    return pose, R


# ------------------------- Drawing -------------------------

def draw_crosshair(frame: np.ndarray, color=(0, 255, 0)) -> None:
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    arm = max(10, min(w, h) // 20)
    t = 2
    cv2.line(frame, (cx - arm, cy), (cx + arm, cy), color, t, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - arm), (cx, cy + arm), color, t, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 3, color, -1, cv2.LINE_AA)


def ensure_writer(out_path: str, size: Tuple[int, int], fps_hint: float) -> Tuple[Optional[cv2.VideoWriter], str]:
    if not out_path:
        return None, ""
    w, h = size
    fourccs = [("mp4v", ".mp4"), ("avc1", ".mp4"), ("XVID", ".avi"), ("MJPG", ".avi")]
    for fourcc_str, ext in fourccs:
        fourcc_func = getattr(cv2, 'VideoWriter_fourcc', None)
        if callable(fourcc_func):
            fourcc: int = cast(int, fourcc_func(*fourcc_str))
        else:
            fourcc = 0
        path = out_path if out_path.lower().endswith(ext) else (out_path + ext)
        writer = cv2.VideoWriter(path, fourcc, fps_hint if fps_hint > 0 else 30.0, (w, h))
        if writer.isOpened():
            return writer, path
        writer.release()
    return None, ""


def put_text_lines(frame: np.ndarray, lines: List[str], origin=(10, 30), scale=0.6, color=(255, 255, 255)) -> None:
    x, y = origin
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (x, y + int(i * 22 * scale)), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (x, y + int(i * 22 * scale)), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


# ------------------------- Snapshot poller -------------------------

class SnapshotCapture:
    """
    Polls an HTTP JPEG endpoint (e.g., http://<ip>/capture) and mimics cv2.VideoCapture.
    """
    def __init__(self, url: str, fps: float = 10.0, timeout_open: float = 2.0, timeout_read: float = 2.0):
        self.url = url
        self.fps = max(0.5, float(fps))
        self._delay = 1.0 / self.fps
        self._sess = requests.Session()
        self._open = True
        self._t_open = timeout_open
        self._t_read = timeout_read
        self._last_w = None
        self._last_h = None

    def isOpened(self) -> bool:
        return self._open

    def read(self):
        t0 = time.time()
        try:
            r = self._sess.get(self.url, timeout=self._t_read)
            r.raise_for_status()
            img_arr = np.frombuffer(r.content, np.uint8)
            frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return False, None
            self._last_h, self._last_w = frame.shape[:2]
            # throttle to target fps
            elapsed = time.time() - t0
            if elapsed < self._delay:
                time.sleep(self._delay - elapsed)
            return True, frame
        except Exception:
            return False, None

    def get(self, prop_id: int):
        if prop_id == cv2.CAP_PROP_FPS:
            return float(self.fps)
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH and self._last_w is not None:
            return float(self._last_w)
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT and self._last_h is not None:
            return float(self._last_h)
        if prop_id in (cv2.CAP_PROP_EXPOSURE, cv2.CAP_PROP_BRIGHTNESS):
            return 0.0
        return 0.0

    def set(self, prop_id: int, value: float) -> bool:
        # No-op; return True so caller doesn't warn
        return True

    def release(self):
        try:
            self._sess.close()
        finally:
            self._open = False


# ------------------------- Main -------------------------

def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    # Decide capture source:
    # - If URL is a string and contains "/capture", use snapshot polling
    # - Else use OpenCV VideoCapture (prefer FFmpeg for URLs)
    if isinstance(args.url, str) and args.url.startswith(("http://", "https://")) and ("/capture" in args.url):
        cap = SnapshotCapture(args.url, fps=float(args.poll_fps))
        source_desc = f"SnapshotCapture({args.url}, fps={args.poll_fps})"
    else:
        cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG) if isinstance(args.url, str) else cv2.VideoCapture(args.url)
        source_desc = f"VideoCapture({args.url})"

    if not cap.isOpened():
        print(f"ERROR: Unable to open video source: {args.url}", file=sys.stderr)
        return 2

    # Exposure / brightness controls
    # For SnapshotCapture these are no-ops and will silently succeed
    if args.auto_exposure == "off":
        ae_supported = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 manual, 1.0 auto (backend-dependent)
        if not ae_supported:
            print("WARNING: AUTO_EXPOSURE property unsupported; manual exposure may fail.")
        exp_ok = cap.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
        br_ok = cap.set(cv2.CAP_PROP_BRIGHTNESS, args.brightness)
        if not exp_ok:
            print("WARNING: Exposure setting not supported by this camera/source.")
        if not br_ok:
            print("WARNING: Brightness setting not supported by this camera/source.")
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        print("Auto-exposure enabled (camera may override manual brightness/exposure).")

    # Report effective values (best-effort)
    try:
        eff_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        eff_bright = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        print(f"{source_desc}")
        print(f"Camera exposure={eff_exp:.2f} brightness={eff_bright:.2f} (auto={args.auto_exposure})")
    except Exception:
        pass

    # AprilTag dictionary selection (auto option)
    family_map = {
        "36h11": cv2.aruco.DICT_APRILTAG_36h11,
        "25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    }
    requested_family = str(args.tag_family).lower()
    detector_params = cv2.aruco.DetectorParameters()
    ippe_available = hasattr(cv2, 'solvePnPGeneric') and hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE')

    # Determine frame size and intrinsics from first frame
    grabbed, frame = cap.read()
    if not grabbed or frame is None:
        print("ERROR: Could not read first frame.", file=sys.stderr)
        return 3

    # Optional resize
    if args.max_width and frame.shape[1] > args.max_width:
        scale = args.max_width / frame.shape[1]
        frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))

    # Auto family detection
    if requested_family == 'auto':
        gray_auto = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_family = None
        best_count = 0
        for fam_key_try, dict_id in family_map.items():
            dict_try = cv2.aruco.getPredefinedDictionary(dict_id)
            corners_try, ids_try, _ = cv2.aruco.detectMarkers(gray_auto, dict_try, parameters=detector_params)
            count = 0 if ids_try is None else len(ids_try)
            if count > best_count:
                best_count = count
                detected_family = fam_key_try
            if best_count > 0:
                break
        if detected_family is None:
            detected_family = '36h11'
            print("AprilTag auto-detect: No tags found on initial frame; defaulting to 36h11")
        else:
            print(f"AprilTag auto-detect selected family: {detected_family} (detections={best_count})")
        fam_key_runtime = detected_family
    else:
        if requested_family not in family_map:
            print(f"WARNING: Unknown tag family '{args.tag_family}', falling back to 36h11")
            fam_key_runtime = '36h11'
        else:
            fam_key_runtime = requested_family

    dictionary = cv2.aruco.getPredefinedDictionary(family_map[fam_key_runtime])
    print(f"PnP method: {'IPPE_SQUARE' if ippe_available else 'ITERATIVE'} | AprilTag family={fam_key_runtime} | auto-exposure={args.auto_exposure}")

    h, w = frame.shape[:2]
    intr = compute_intrinsics(w, h, args.fov)

    fps_stream = cap.get(cv2.CAP_PROP_FPS)
    if fps_stream is None or fps_stream <= 0 or math.isnan(fps_stream):
        # for SnapshotCapture this will be args.poll_fps
        fps_stream = 30.0

    writer = None
    out_path_actual = ""
    if args.record and args.out:
        writer, out_path_actual = ensure_writer(args.out, (w, h), fps_stream)
        if writer is None:
            print("WARNING: Failed to create VideoWriter; recording disabled.")
            args.record = False
        else:
            print(f"Recording to {out_path_actual}")

    if args.display:
        cv2.namedWindow("AprilTag Tracker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag Tracker", min(w, 1280), min(h, 720))

    # Main loop
    t0 = time.time()
    frames = 0
    last_fps_report = t0

    last_rvec: Optional[np.ndarray] = None
    last_tvec: Optional[np.ndarray] = None

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            # Attempt to re-read once for network hiccups
            time.sleep(0.05)
            ok, frame = cap.read()
            if not ok or frame is None:
                break

        if args.max_width and frame.shape[1] > args.max_width:
            scale = args.max_width / frame.shape[1]
            frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))
            h, w = frame.shape[:2]
            intr = compute_intrinsics(w, h, args.fov)

        frame_proc = frame
        gray = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2GRAY)

        # AprilTag detection
        tag_ids: List[int] = []
        bbox_list: List[np.ndarray] = []
        corners, ids, _ = cv2.aruco.detectMarkers(frame_proc, dictionary, parameters=detector_params)
        if ids is not None and len(corners) > 0:
            for corner, id_arr in zip(corners, ids):
                pts = corner[0].astype(np.float32)  # TL,TR,BR,BL
                ordered = order_corners(pts)
                ordered = refine_corners(gray, ordered)
                bbox_list.append(ordered)
                tag_ids.append(int(id_arr[0]))

        # Prepare overlay
        overlay_proc = frame_proc.copy()
        draw_crosshair(overlay_proc, color=(0, 200, 0))

        info_lines = [f"Frame: {frames}"]
        for idx, pts in enumerate(bbox_list):
            pts_int = pts.astype(int)
            cv2.polylines(overlay_proc, [pts_int], True, (0, 255, 255), 2, cv2.LINE_AA)
            cxi = int(pts[:, 0].mean())
            cyi = int(pts[:, 1].mean())
            cv2.circle(overlay_proc, (cxi, cyi), 4, (255, 0, 255), -1, cv2.LINE_AA)

            pose, R = solve_tag_pose(pts, intr, args.tag_size, last_rvec, last_tvec, args.smooth_alpha)
            tag_id = tag_ids[idx] if idx < len(tag_ids) else -1

            if pose is not None:
                last_rvec = pose.rvec
                last_tvec = pose.tvec
                K = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], dtype=np.float32)
                axis_len = float(args.tag_size * 0.75)
                axis = np.array(
                    [[0.0, 0.0, 0.0],
                     [axis_len, 0.0, 0.0],
                     [0.0, axis_len, 0.0],
                     [0.0, 0.0, axis_len]], dtype=np.float32
                )
                if pose.rvec is not None and pose.tvec is not None:
                    try:
                        imgpts, _ = cv2.projectPoints(axis, pose.rvec, pose.tvec, K, intr.dist)
                        imgpts = imgpts.reshape(-1, 2).astype(int)
                        origin = tuple(imgpts[0])
                        cv2.line(overlay_proc, origin, tuple(imgpts[1]), (0, 0, 255), 2)  # X - red
                        cv2.line(overlay_proc, origin, tuple(imgpts[2]), (0, 255, 0), 2)  # Y - green
                        cv2.line(overlay_proc, origin, tuple(imgpts[3]), (255, 0, 0), 2)  # Z - blue
                    except cv2.error:
                        pass

                txt = (f"Tag id={tag_id} yaw={pose.yaw_deg:+.1f} pitch={pose.pitch_deg:+.1f} roll={pose.roll_deg:+.1f} | "
                       f"bearing x={pose.bearing_x_deg:+.1f} y={pose.bearing_y_deg:+.1f}")
                cv2.putText(overlay_proc, txt, (max(5, cxi - 120), max(15, cyi - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, args.font_scale, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(overlay_proc, txt, (max(5, cxi - 120), max(15, cyi - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, args.font_scale, (255, 255, 255), 1, cv2.LINE_AA)
                info_lines.append(
                    f"Tag[{tag_id}]: yaw={pose.yaw_deg:+.1f} pitch={pose.pitch_deg:+.1f} roll={pose.roll_deg:+.1f} | "
                    f"bearing x={pose.bearing_x_deg:+.1f} y={pose.bearing_y_deg:+.1f}"
                )
            else:
                info_lines.append(f"Tag[{tag_id}]: pose=N/A")

        put_text_lines(overlay_proc, info_lines, origin=(10, 28), scale=args.font_scale)

        frame_out_proc = cv2.addWeighted(frame_proc, 0.8, overlay_proc, 0.7, 0)
        frame_out = cv2.flip(frame_out_proc, 1) if args.mirror else frame_out_proc

        if args.display:
            cv2.imshow("AprilTag Tracker", frame_out)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

        if args.record and writer is not None:
            writer.write(frame_out)

        frames += 1
        now = time.time()
        if now - last_fps_report >= 1.0:
            elapsed = now - t0
            fps = frames / elapsed if elapsed > 0 else 0.0
            print(f"FPS: {fps:.1f} | frames={frames}")
            last_fps_report = now

    cap.release()
    if writer is not None:
        writer.release()
    if args.display:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
