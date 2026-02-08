import numpy as np
import cv2
from collections import deque

class Visualizer:
    def __init__(self):
        self.traj = deque() 

    def push_pose(self, T_wc):
        t = T_wc[:3, 3].astype(np.float64)
        self.traj.append(t)  


    def draw_map_view(self, map_obj, window_size=(600, 600)):
        """
        rzut z góry mapę
        """
        h, w = window_size
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = 30 

        if len(map_obj.points) == 0 or len(map_obj.frames) == 0:
            return img


        pts = np.array([p.pt for p in map_obj.points])  
        xs = pts[:, 0]
        zs = pts[:, 2]

        # Ow = -R_cw^T * t_cw
        cam_positions = []
        for f in map_obj.frames:
            T_wc = f.T_wc
            twc = T_wc[:3, 3]
            cam_positions.append(twc)
        cam_positions = np.array(cam_positions)  
        cx = cam_positions[:, 0]
        cz = cam_positions[:, 2]

        
        all_x = np.concatenate([xs, cx])
        all_z = np.concatenate([zs, cz])

        
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_z, max_z = np.min(all_z), np.max(all_z)
        if max_x - min_x < 1e-6 or max_z - min_z < 1e-6:
            return img


        margin = 20
        sx = (w - 2 * margin) / (max_x - min_x)
        sz = (h - 2 * margin) / (max_z - min_z)
        s = min(sx, sz)


        def world_to_img(x, z):
            u = int(margin + (x - min_x) * s)
            v = int((margin + (z - min_z) * s)) 
            return u, v



        for x, z in zip(xs, zs):
            u, v = world_to_img(x, z)
            cv2.circle(img, (u, v), 0.4, (0, 255, 0), -1)


        for i in range(1, len(cam_positions)):
            x1, z1 = cx[i-1], cz[i-1]
            x2, z2 = cx[i], cz[i]
            u1, v1 = world_to_img(x1, z1)
            u2, v2 = world_to_img(x2, z2)
            cv2.line(img, (u1, v1), (u2, v2), (255, 0, 0), 2)
        # last pose 
        u_last, v_last = world_to_img(cx[-1], cz[-1])
        cv2.circle(img, (u_last, v_last), 4, (0, 0, 255), -1)

        return img


    def draw_trajectory(self, window_size=(600, 600), margin=20):
        h, w = window_size
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = 30

        if len(self.traj) == 0:
            return img

        cam_positions = np.asarray(self.traj)
        cx = cam_positions[:, 0]
        cz = cam_positions[:, 2]

        min_x, max_x = np.min(cx), np.max(cx)
        min_z, max_z = np.min(cz), np.max(cz)

        span_x = max(1e-3, max_x - min_x)
        span_z = max(1e-3, max_z - min_z)

        sx = (w - 2 * margin) / span_x
        sz = (h - 2 * margin) / span_z
        s = min(sx, sz)

        def world_to_img(x, z):
            u = int(margin + (x - min_x) * s)
            v = int(margin + (z - min_z) * s)
            return u, v

        u0, v0 = world_to_img(0.0, 0.0)

       
        u0 = max(0, min(w - 1, u0))
        v0 = max(0, min(h - 1, v0))  

        
        cv2.arrowedLine(img, (0, v0), (w - 1, v0), (50, 50, 0), 2, tipLength=0.002) 

       
        cv2.arrowedLine(img, (u0, h - 1), (u0, 0), (50, 50, 0), 2, tipLength=0.002)  


        pts2d = np.array([world_to_img(x, z) for x, z in zip(cx, cz)], np.int32).reshape(-1, 1, 2)
        if len(pts2d) >= 2:
            cv2.polylines(img, [pts2d], False, (255, 0, 0), 2) 
        u_last, v_last = world_to_img(cx[-1], cz[-1])
        cv2.circle(img, (u_last, v_last), 4, (0, 0, 255), -1) 

        return img