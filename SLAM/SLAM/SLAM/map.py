from collections import deque
import numpy as np

from frame import match_frames
from pose import cam_to_world_point


class MapPoint:
    def __init__(self, pt3d_world):
        self.pt = np.array(pt3d_world, dtype=np.float64)
        self.pt_cov = np.eye(3) * 0.1
        self.id = -1
        self.observations = []  # (frame_id, kp_idx)
        self.is_bad = False
        self.last_seen_frame = -1
        self.des = None

    def add_observation(self, frame_id, kp_idx):
        self.observations.append((frame_id, kp_idx))

    def update_position(self, new_pt3d, T_wc):
        new_pt_world = cam_to_world_point(T_wc, new_pt3d)
        if np.any(np.isnan(new_pt_world)):
            return
        n_obs = len(self.observations)
        self.pt = (self.pt * n_obs + new_pt_world) / (n_obs + 1)

    @property
    def num_observations(self):
        return len(self.observations)

    def set_bad(self):
        self.is_bad = True


class Map:
    def __init__(self, max_frames=100, recent_kf=5, max_buffer=15000):
        self.frames = deque(maxlen=max_frames)
        self.keyframes = []
        self.recent_kf = recent_kf

        self.points = []
        self.point_buffer = deque(maxlen=max_buffer)

        self._next_id = {"keyframe": 0, "point": 0}

    def _next_kf_id(self):
        self._next_id["keyframe"] += 1
        return self._next_id["keyframe"] - 1

    def _next_point_id(self):
        self._next_id["point"] += 1
        return self._next_id["point"] - 1

    def add_frame(self, frame, as_keyframe=False):
        self.frames.append(frame)
        if as_keyframe:
            frame.kid = self._next_kf_id()
            self.keyframes.append(frame)
        return frame.id

    def flush_buffer(self, current_frame, min_obs=6, max_dist=4.0):
        cam_pos = current_frame.t_wc.reshape(-1)
        good = []
        rest = []
        
        for mp in self.point_buffer:
            if mp.num_observations >= min_obs and np.linalg.norm(mp.pt - cam_pos) < max_dist:
                good.append(mp)
            else:
                rest.append(mp)

        self.point_buffer = deque(rest, maxlen=self.point_buffer.maxlen)
        self.points.extend(good)
        return len(good)

    def points_to_place_on_map(self, kf, every_nth=1):
        if kf.kps3d is None or len(kf.kps3d) == 0:
            return 0
        Xw = cam_to_world_point(kf.T_wc, kf.kps3d)


        added = 0
        for i in range(len(Xw)):

            mp = MapPoint(Xw[i])
            mp.id = self._next_point_id()
            self.point_buffer.append(mp)
            kf.set_point_match(mp, i)
            #mp.last_seen_frame = kf.id
            mp.add_observation(kf.id, i)
            added += 1
        return added

    def update_map(self, current_frame,idx1,idx2):
        matches = 0
        kf_ref = self.keyframes[-1]
        for i1, i2 in zip(idx1, idx2):
            mp = kf_ref.points[i1] 
            if mp is None:
                continue
            mp.add_observation(current_frame.id, i2)
            #mp.last_seen_frame = current_frame.id
            current_frame.set_point_match(mp, i2)


            #mp.update_position(current_frame.kps3d[i2], current_frame.T_wc)

            matches += 1
        return matches

    def get_local_keyframes(self, n=4):
        return self.keyframes[-n:] if len(self.keyframes) > 0 else []

    def get_local_points(self, n_kf=5, min_obs=2):
        pts = []
        for kf in self.get_local_keyframes(n_kf):
            for mp in kf.points:
                if mp is None:
                    continue
                if mp.num_observations < min_obs:
                    continue
                pts.append(mp)

        uniq = {}
        for p in pts:
            uniq[p.id] = p
        return list(uniq.values())

    def cull_bad_points(self, current_frame_id, max_age=50, min_obs=2):
        new_buf = deque(maxlen=self.point_buffer.maxlen)
        for mp in self.point_buffer:
            if mp.is_bad:
                continue
            if mp.num_observations < 1 and (current_frame_id - mp.last_seen_frame) > 10:
                continue
            new_buf.append(mp)
        self.point_buffer = new_buf

        kept = []
        for mp in self.points:
            if mp.is_bad:
                continue
            if mp.num_observations < min_obs:
                continue
            if mp.last_seen_frame >= 0 and (current_frame_id - mp.last_seen_frame) > max_age:
                continue
            kept.append(mp)
        self.points = kept