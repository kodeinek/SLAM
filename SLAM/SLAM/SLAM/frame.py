import time
import numpy as np
import cv2

from pose import CameraPose



class Frame:
    def __init__(self, img_gray, full_pc=None, normals=None, pose=None, id=0, matcher=3):
        
        self.id = id
        self.kid = -1  # >=0 -> keyframe

        self._pose = CameraPose(pose)

        self.img = img_gray
        self.kps = np.empty((0, 2), dtype=np.float32)
        self.kps3d = np.empty((0, 3), dtype=np.float64)
        self.des = None

        self.points = []
        self.normals = normals
        self.full_pc = full_pc
        self.matcher = matcher

        start = time.time()
        self.extract_features(img_gray, method=matcher)
        self.feature_extraction_time = time.time() - start






    def extract_features(self, img_gray, method=2, max_features=5000):

        mask = None

        valid_dist = np.isfinite(self.full_pc).all(axis=2)
        mask = valid_dist.astype(np.uint8) * 255



        kps, des = None, None

        if method == 0:
            corners = cv2.goodFeaturesToTrack(
                img_gray, maxCorners=max_features, qualityLevel=0.02,
                blockSize=5, minDistance=3, mask=mask
            )
            kps = []
            if corners is not None:
                for c in corners:
                    x, y = float(c[0][0]), float(c[0][1])
                    kps.append(cv2.KeyPoint(x=x, y=y, size=7))

            orb = cv2.ORB_create(nfeatures=len(kps), scoreType=cv2.ORB_HARRIS_SCORE)
            kps, des = orb.compute(img_gray, kps)

        elif method == 1:
            fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
            kps = fast.detect(img_gray, mask) or []
            orb = cv2.ORB_create(nfeatures=len(kps), scoreType=cv2.ORB_HARRIS_SCORE)
            kps, des = orb.compute(img_gray, kps)

        elif method == 2:
            sift = cv2.SIFT_create(nfeatures=max_features, contrastThreshold=0.04, edgeThreshold=10, sigma=1.6)
            kps, des = sift.detectAndCompute(img_gray, mask)

            if kps is None or len(kps) == 0:
                return [], None

        elif method == 3:


            orb = cv2.ORB_create(edgeThreshold=31, patchSize=31, nlevels=12, fastThreshold=20,scoreType=cv2.ORB_HARRIS_SCORE, nfeatures = 1200)
            kps, des = orb.detectAndCompute(img_gray,mask = mask)

            # orb = cv2.ORB_create(nfeatures = max_features ,scoreType=cv2.ORB_HARRIS_SCORE)
            # kps, des = orb.detectAndCompute(img_gray, mask)
            #

            # H, W = img_gray.shape[:2]
            
            # grid_x, grid_y = 2,2
            # overlap = 25
            
            
            # per_tile = max(50, max_features // (grid_x * grid_y)) if max_features is not None else None
            # per_tile = None
            
            # orb = cv2.ORB_create(
            #     nfeatures=per_tile,
            #     scaleFactor = 1.2,
            #     nlevels = 12,
            #     scoreType=cv2.ORB_HARRIS_SCORE,
            
            #     edgeThreshold=25,
            #     patchSize=25,
            #     fastThreshold=8,
            # )
            
            # kps_all = []
            # des_list = []
            
            # for gy in range(grid_y):
            #     for gx in range(grid_x):
            #         x0 = int(gx * W / grid_x)
            #         x1 = int((gx + 1) * W / grid_x)
            #         y0 = int(gy * H / grid_y)
            #         y1 = int((gy + 1) * H / grid_y)
            
            #         # overlap (clamp do obrazu)
            #         xa = max(0, x0 - overlap)
            #         xb = min(W, x1 + overlap)
            #         ya = max(0, y0 - overlap)
            #         yb = min(H, y1 + overlap)
            
            #         roi = img_gray[ya:yb, xa:xb]
            #         mroi = None if mask is None else mask[ya:yb, xa:xb]
            
            #         kps_roi= orb.detect(roi, mroi)
            #         if len(kps_roi) == 0:
            #             continue
            
            
            #         for kp in kps_roi:
            #             kp.pt = (kp.pt[0] + xa, kp.pt[1] + ya)
            
            #         kps_all.extend(kps_roi)
            
            
            # kps = kps_all
            
            
            
            #     #deduplikacja
            # uniq = {}
            # for i, kp in enumerate(kps):
            #     u = int(round(kp.pt[0])); v = int(round(kp.pt[1]))
            #     key = (u, v)
            #     if (key not in uniq) or (kp.response > kps[uniq[key]].response):
            #         uniq[key] = i
            # keep_idx = np.fromiter(uniq.values(), dtype=np.int32)
            # kps = [kps[i] for i in keep_idx]
            
            # kps, des = orb.compute(img_gray, kps)




        elif method == 5:
            # KAZE
            kaze = cv2.KAZE_create(
                extended=False,
                upright=False,
                threshold=0.001,
                nOctaves=2,
                nOctaveLayers=3
            )
            kps, des = kaze.detectAndCompute(img_gray, mask)
            if kps is None or len(kps) == 0:
                return [], None

        elif method == 6:
            # AKAZE
            akaze = cv2.AKAZE_create(
                descriptor_type=cv2.AKAZE_DESCRIPTOR_MLDB,
                descriptor_size=0,
                descriptor_channels=3,
                threshold=0.001,
                nOctaves=3,
                nOctaveLayers=3,
            )
            kps, des = akaze.detectAndCompute(img_gray, mask)
            if kps is None or len(kps) == 0:
                return [], None

        elif method == 7:
            # BRISK
            brisk = cv2.BRISK_create(
                thresh=30,
                octaves=3,
                patternScale=1.0
            )
            kps, des = brisk.detectAndCompute(img_gray, mask)
            if kps is None or len(kps) == 0:
                return [], None
        else:
            raise ValueError(f"Unknown feature method: {method}")

        if kps is None or len(kps) == 0:
            self.kps = np.empty((0, 2), dtype=np.float32)
            self.kps3d = np.empty((0, 3), dtype=np.float64)
            self.des = None
            self.points = []
            return


        kps_xy = np.array([kp.pt for kp in kps], dtype=np.float32)
        self.kps = kps_xy

        if self.full_pc is not None:
            H, W = self.full_pc.shape[:2]
            r = 2

            u = kps_xy[:, 0]
            v = kps_xy[:, 1]

            # najbliższy piksel
            u0 = np.rint(u).astype(np.int32)
            v0 = np.rint(v).astype(np.int32)


            offs = np.arange(-r, r + 1, dtype=np.int32)
            du, dv = np.meshgrid(offs, offs, indexing="xy")
            du = du.reshape(-1) 
            dv = dv.reshape(-1) 
            K = du.size

  
            uu = np.clip(u0[:, None] + du[None, :], 0, W - 1)
            vv = np.clip(v0[:, None] + dv[None, :], 0, H - 1)


            cand = self.full_pc[vv, uu, :]

            ok = np.isfinite(cand).all(axis=2)


            d2 = (uu.astype(np.float32) - u[:, None])**2 + (vv.astype(np.float32) - v[:, None])**2

            d2 = np.where(ok, d2, np.inf)

            j = np.argmin(d2, axis=1) 

            has = np.isfinite(d2[np.arange(d2.shape[0]), j])

            pts3d = cand[np.arange(cand.shape[0]), j, :].astype(np.float64)  

            has = has & (np.linalg.norm(pts3d, axis=1) > 0.1)


            self.kps = self.kps[has]
            self.kps3d = pts3d[has]

            if des is not None:
                des = des[has]

        else:
            self.kps3d = np.full((len(self.kps), 3), np.nan, dtype=np.float64)

        self.des = des
        self.points = [None] * len(self.kps)
    @property
    def T_wc(self): return self._pose.T_wc.copy()

    @property
    def R_wc(self): return self._pose.R_wc.copy()

    @property
    def t_wc(self): return self._pose.t_wc.copy()

    @property
    def T_cw(self): return self._pose.T_cw.copy()

    @property
    def R_cw(self): return self._pose.R_cw.copy()

    @property
    def t_cw(self): return self._pose.t_cw.copy()

    def update_pose(self, pose):
        self._pose.set(pose)

    def set_point_match(self, mp, idx):
        if self.points is None or len(self.points) != len(self.kps):
            self.points = [None] * len(self.kps)
        if 0 <= idx < len(self.points):
            self.points[idx] = mp


def _fundamental_filter(f1, f2, idx1, idx2):
    if len(idx1) < 8:
        return idx1, idx2
    F, mask = cv2.findFundamentalMat(f1.kps[idx1], f2.kps[idx2], cv2.FM_RANSAC, 1.0, 0.99)
    if mask is None:
        return idx1, idx2
    m = mask.ravel().astype(bool)
    return idx1[m], idx2[m]

def sample_pc_nearest_valid(full_pc, u, v, r=2):
    h, w = full_pc.shape[:2]
    u0 = int(round(u))
    v0 = int(round(v))

    ua = max(0, u0 - r); ub = min(w - 1, u0 + r)
    va = max(0, v0 - r); vb = min(h - 1, v0 + r)

    patch = full_pc[va:vb+1, ua:ub+1, :]  
    ok = np.isfinite(patch).all(axis=2)

    if not np.any(ok):
        return None

    ys, xs = np.where(ok)

    du = (xs + ua) - u
    dv = (ys + va) - v
    d2 = du*du + dv*dv
    j = np.argmin(d2)

    return patch[ys[j], xs[j], :]

def match_frames_bf(f1: Frame, f2: Frame, ratio_test=0.75):
    if f1.des is None or f2.des is None:
        return np.array([], dtype=np.int32), np.array([], dtype=np.int32)

    if f1.matcher in (2,5):
        matcher = cv2.BFMatcher_create(cv2.NORM_L2, crossCheck=False)
    else:
        matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=False)

    knn = matcher.knnMatch(f1.des, f2.des, k=2)
    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio_test * n.distance:
            good.append(m)

    if len(good) < 10:
        return np.array([], dtype=np.int32), np.array([], dtype=np.int32)

    idx1 = np.array([m.queryIdx for m in good], dtype=np.int32)
    idx2 = np.array([m.trainIdx for m in good], dtype=np.int32)
    return idx1, idx2


def match_frames_flann(f1: Frame, f2: Frame, ratio_test=0.75):
    if f1.des is None or f2.des is None:
        return np.array([], dtype=np.int32), np.array([], dtype=np.int32)

    if f1.matcher in (2,5):
        index_params = dict(algorithm=1, trees=5) 
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        des1, des2 = f1.des, f2.des
    else:
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)  
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
       
        des1, des2 = f1.des.astype(np.uint8), f2.des.astype(np.uint8)

    knn = matcher.knnMatch(des1, des2, k=2)
    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio_test * n.distance:
            good.append(m)

    if len(good) < 10:
        return np.array([], dtype=np.int32), np.array([], dtype=np.int32)

    idx1 = np.array([m.queryIdx for m in good], dtype=np.int32)
    idx2 = np.array([m.trainIdx for m in good], dtype=np.int32)
    return _fundamental_filter(f1, f2, idx1, idx2)


def match_frames(f1: Frame, f2: Frame, mode="bf"):
    if mode == "flann":
        return match_frames_flann(f1, f2)
    return match_frames_bf(f1, f2)

