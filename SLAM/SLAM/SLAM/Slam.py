import numpy as np
import cv2

from frame import Frame, match_frames
from map import Map, cam_to_world_point
from pose import rt_matrix


def _reprojection_errors(object_points, image_points, Rcw, tcw, K):
    rvec, _ = cv2.Rodrigues(Rcw)
    proj, _ = cv2.projectPoints(object_points, rvec, tcw.reshape(3, 1), K, None)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - image_points, axis=1)
    return err



def _solve_pnp_ransac_refine(
        object_points,
        image_points,
        K,
        reproj_err_px=3.0,
        iters=300,
        confidence=0.999,
        min_pts=20,
        Rcw_prior=None,
        tcw_prior=None,
        dbg=False,
):
    if object_points is None or image_points is None:
        if dbg:
            print("[PnP] FAIL: object_points or image_points is None")
        return False, None, None, None

    n = len(object_points)
    if n < min_pts:
        if dbg:
            print(f"[PnP] FAIL: not enough points: {n} < min_pts={min_pts}")
        return False, None, None, None

    use_prior = (Rcw_prior is not None) and (tcw_prior is not None)


    if use_prior:
        rvec0, _ = cv2.Rodrigues(Rcw_prior)
        tvec0 = tcw_prior.reshape(3, 1)
    else:
        rvec0 = None
        tvec0 = None

    try:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            K,
            None,
            rvec=rvec0,
            tvec=tvec0,
            useExtrinsicGuess=use_prior,
            iterationsCount=iters,
            reprojectionError=reproj_err_px,
            confidence=confidence,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error as e:
        if dbg:
            print(f"[PnP] FAIL: solvePnPRansac cv2.error: {e}")
        return False, None, None, None

    if not ok:
        if dbg:
            print(f"[PnP] FAIL: solvePnPRansac returned ok=False (n={n}, use_prior={use_prior})")
        return False, None, None, None

    if inliers is None:
        if dbg:
            print(f"[PnP] FAIL: solvePnPRansac inliers=None (ok={ok}, n={n}, use_prior={use_prior})")
        return False, None, None, None

    if len(inliers) < min_pts:
        if dbg:
            print(f"[PnP] FAIL: too few inliers: {len(inliers)} < min_pts={min_pts} (n={n}, use_prior={use_prior})")
        return False, None, None, None

    inl = inliers.reshape(-1)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 50, 1e-7)
    try:
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points[inl],
            image_points[inl],
            K,
            None,
            rvec,
            tvec,
            criteria=criteria,
        )
    except cv2.error as e:

        if dbg:
            print(f"[PnP] WARN: solvePnPRefineLM failed (keeping RANSAC pose): {e}")

    Rcw, _ = cv2.Rodrigues(rvec)
    tcw = tvec.reshape(3, 1)

    if dbg:
        print(f"[PnP] OK: n={n}, inl={len(inl)}, use_prior={use_prior}, reproj_err_px={reproj_err_px}, iters={iters}")

    return True, Rcw, tcw, inl


def solve_pnp_ransac_refine_2pass(
        obj_pts,
        img_pts,
        K,
        reproj_err_px_ransac=2.0,
        reproj_err_px_gate=1.6,
        iters=500,
        confidence=0.999,
        min_pts=20,
        Rcw_prior=None,
        tcw_prior=None,
        dbg=False,
):
    ok, Rcw, tcw, inl = _solve_pnp_ransac_refine(
        obj_pts,
        img_pts,
        K,
        reproj_err_px=reproj_err_px_ransac,
        iters=iters,
        confidence=confidence,
        min_pts=min_pts,
        Rcw_prior=Rcw_prior,
        tcw_prior=tcw_prior,
        dbg=dbg,
    )

    if (not ok) or (inl is None) or (len(inl) < min_pts):
        if dbg:
            n = 0 if obj_pts is None else len(obj_pts)
            linl = -1 if inl is None else len(inl)
            print(f"[PnP2] FAIL: first pass failed (ok={ok}, n={n}, inl={linl}, min_pts={min_pts})")
        return False, None, None, None

    obj_inl = obj_pts[inl]
    img_inl = img_pts[inl]

    try:
        err = _reprojection_errors(obj_inl, img_inl, Rcw, tcw, K)
    except cv2.error as e:
        if dbg:
            print(f"[PnP2] FAIL: reprojection error computation failed: {e}")
        return False, None, None, None

    keep = err < reproj_err_px_gate
    n_keep = int(keep.sum())

    if n_keep < min_pts:
        if dbg:
            print(
                f"[PnP2] FAIL: reproj gate removed too many: keep={n_keep}/{len(inl)} < min_pts={min_pts} (gate_px={reproj_err_px_gate})")
        return False, None, None, None

    obj2 = obj_inl[keep]
    img2 = img_inl[keep]

    rvec, _ = cv2.Rodrigues(Rcw)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 50, 1e-7)

    try:
        rvec, tvec = cv2.solvePnPRefineLM(obj2, img2, K, None, rvec, tcw, criteria=criteria)
        Rcw, _ = cv2.Rodrigues(rvec)
        tcw = tvec.reshape(3, 1)
    except cv2.error as e:
        if dbg:
            print(f"[PnP2] WARN: second-pass solvePnPRefineLM failed (keeping pass1 pose): {e}")

    inl2 = inl[keep]

    if dbg:
        print(f"[PnP2] OK: inl(pass1)={len(inl)}, keep(after gate)={len(inl2)}, gate_px={reproj_err_px_gate}")

    return True, Rcw, tcw, inl2

from scipy.spatial.transform import Rotation as R_scipy

def imu_rot_predict(R_prev, ang_vel, dt=0.033):  
    delta_rot = R_scipy.from_rotvec(ang_vel * dt)
    return (delta_rot * R_scipy.from_matrix(R_prev)).as_matrix()

class SLAM:
    def __init__(self, cam_module, feature_extactor=3, zed=True):
        
        self.cam_module = cam_module
        self.map = Map()

        self._next_id = 0
        self.initialized = False

        self.reference_frame = None
        self.last_frame = None
        self.current_frame = None

        self.feature_extractor = feature_extactor
        self.zed = bool(zed)

        self.last_R = np.eye(3)
        self.last_t = np.zeros((3, 1))
        self.zed_cam_T_cw = np.eye(4, dtype=np.float64)
        self.min_pts = 20

    def _next_frame_id(self):
        self._next_id += 1
        return self._next_id - 1

    def estimate_motion(self, idx1, idx2, ref_frame=None):
        ref_frame = self.reference_frame if ref_frame is None else ref_frame
        cur_frame = self.current_frame

        if ref_frame is None or cur_frame is None:
            print("No frames for PnP")
            return None, None, [None]

        if len(idx1) == 0:
            print("No keypoints matched for PnP")
            return None, None, [None]

        obj_cam = ref_frame.kps3d[idx1]
        dst_cam = cur_frame.kps3d[idx2]
        ok3d = np.isfinite(obj_cam).all(1) & np.isfinite(dst_cam).all(1)
        ok3d = ok3d & (np.abs(obj_cam[:, 2] - dst_cam[:, 2]) < 0.5)  
        idx1 = idx1[ok3d]
        idx2 = idx2[ok3d]

        obj_pts = ref_frame.kps3d[idx1]
        img_pts = cur_frame.kps[idx2]
        obj_pts = cam_to_world_point(ref_frame.T_wc, obj_pts)

        if len(obj_pts) < self.min_pts:
            print("Not enough valid 3D-2D points for PnP:", len(obj_pts))
            return None, None, [None]

        obj_pts = np.asarray(obj_pts, dtype=np.float32)
        img_pts = np.asarray(img_pts, dtype=np.float32)

        pose = self.cam_module.pose
        if pose.valid:
            ang_vel = pose.twist[3:6]
            self.lastR = 0.8 * imu_rot_predict(self.last_R, ang_vel) + 0.2 * self.last_R

        ok, Rcw, tcw, inl = solve_pnp_ransac_refine_2pass(
            obj_pts, img_pts, self.cam_module.K,
            Rcw_prior=self.last_R, tcw_prior=self.last_t
        )


        if not ok:
            print("PnP failed")
            return None, None, [None]


        pose = self.cam_module.pose

        if ok:
            self.last_R = Rcw
        print(
            f"F.{self.current_frame.id} extracted: {cur_frame.kps.shape[0]} in {cur_frame.feature_extraction_time:.3f}s  inl: {0 if inl is None else len(inl)} / {len(obj_pts)} tcw: ~ {self.last_frame.t_cw} ")
        return Rcw, tcw, [] if inl is None else inl

    def should_be_keyframe(self, frame, n_inl, max_frames=20):
        if self.reference_frame is None:
            return True
        if (frame.id - self.reference_frame.id) > int(max_frames):
            return True

        dt = np.linalg.norm(frame.t_cw - self.reference_frame.t_cw)
        dR = np.linalg.norm(frame.R_cw - self.reference_frame.R_cw)

        if n_inl < 80:
            return True
        if dt > 0.25 or dR > 0.15:
            return True
        return False

    def set_pose(self, frame, T_cw):
        frame.update_pose(T_cw)

    def update(self, R_delta, t_delta):
        """ ZED pose (delta) WORLD accumulation"""
        if self.last_frame is None:
            return np.eye(4, dtype=np.float64)

        T_delta = rt_matrix(R_delta, t_delta)
        T_last = self.last_frame.T_cw

        return T_delta @ T_last

    def get_pose(self):

        idx1, idx2 = match_frames(self.reference_frame, self.current_frame, mode="bf")
        if not self.zed:
            Rcw, tcw, inl = self.estimate_motion(idx1, idx2)
            if (Rcw is not None) and (tcw is not None) and (len(inl) >= self.min_pts):
                return rt_matrix(Rcw, tcw), inl, idx1, idx2

            print("fallback to last frame")
            if len(self.map.frames) > 0:
                idx1, idx2 = match_frames(self.last_frame, self.current_frame, mode="bf")
                Rcw, tcw, inl = self.estimate_motion(idx1, idx2, ref_frame=self.map.frames[-1])
                if (Rcw is not None) and (tcw is not None) and (len(inl) >= self.min_pts):
                    return rt_matrix(Rcw, tcw), inl, idx1, idx2

            print("PnP fallback fail, fallback to ZED")

        R_delta, t_delta = self.cam_module.get_pose_camera()
        if self.zed:
            # r,t, ok = self.cam_module.get_pose_world()
            # mx = rt_matrix(r,t)
            # return  np.linalg.inv(mx) , np.empty(100), idx1, idx2
            return self.update(R_delta, t_delta), np.empty(100), idx1, idx2

        return self.update(R_delta, t_delta), [], idx1, idx2

    def track(self, img, full_pc_raw, normals):
        img_gray = img

        if not self.initialized:
            self.reference_frame = Frame(
                img_gray,
                full_pc=full_pc_raw,
                normals=normals,
                matcher=self.feature_extractor,
                id=self._next_frame_id(),
            )
            self.reference_frame.update_pose(np.eye(4, dtype=np.float64))
            self.map.add_frame(self.reference_frame, as_keyframe=True)
            self.map.points_to_place_on_map(self.reference_frame)

            self.initialized = True
            self.last_frame = self.reference_frame
            return self.reference_frame

        self.current_frame = Frame(
            img_gray,
          
            full_pc=full_pc_raw,
            normals=normals,
            matcher=self.feature_extractor,
            id=self._next_frame_id(),
        )

        T_wc, inl, idx1, idx2 = self.get_pose()
        self.set_pose(self.current_frame, T_wc)

        # print(f"t_cw := {T_cw[:3, 3]}, t_wc = {self.current_frame.t_wc}")

        # self.map.update_map(self.current_frame, idx1,idx2)
        self.map.add_frame(self.current_frame, as_keyframe=False)

        if self.should_be_keyframe(self.current_frame, len(inl)):
            self.map.add_frame(self.current_frame, as_keyframe=True)
            self.reference_frame = self.current_frame
            # self.map.points_to_place_on_map(self.current_frame)
            # self.map.flush_buffer(self.current_frame, min_obs=4)
            print(f"KEYFRAME {self.current_frame.kid}")

        self.last_frame = self.current_frame
        return self.current_frame


