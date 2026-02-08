import numpy as np

class CameraPose:
    """
    T_cw: X_c = R_cw * X_w + t_cw
    """

    def __init__(self, pose=None):
        self.set(np.eye(4, dtype=np.float64) if pose is None else pose)

    def set(self, pose):
        self.T_cw = pose
        self.R_cw = self.T_cw[:3, :3]
        self.t_cw = self.T_cw[:3, 3]

        self.T_wc = np.linalg.inv(self.T_cw)
        self.R_wc = self.T_wc[:3, :3]
        self.t_wc = self.T_wc[:3, 3]

    def set_wc(self,pose):
        self.T_wc = pose.copy()
        self.R_wc = self.T_wc[:3, :3]
        self.t_wc = self.T_wc[:3, 3]
        self.T_cw = np.linalg.inv(self.T_wc)
        self.R_cw = self.T_cw[:3, :3]
        self.t_cw = self.T_cw[:3, 3]



def rt_matrix(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(-1)
    return T



def cam_to_world_point(T_wc, kps3d):
    P = np.atleast_2d(kps3d)
    pts3d_hom = np.hstack([P, np.ones((P.shape[0], 1))]).T
    Xw = (T_wc @ pts3d_hom)[:3].T
    return Xw[0] if np.asarray(kps3d).ndim == 1 else Xw