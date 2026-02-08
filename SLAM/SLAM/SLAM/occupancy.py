
import numpy as np
import matplotlib.pyplot as plt
import cv2




class OccupancyGrid:
    def __init__(self, size=10.0, resolution=0.05):
        self.size = size
        self.res = resolution # meters per cell
        self.n = int(size / resolution)

        self.log_odds = np.full((self.n, self.n),0.2, dtype=np.float32)
        self.hit_count = np.zeros((self.n, self.n), dtype=np.uint8)

        self.l_occ = np.log(0.65 / 0.35)
        self.l_free = np.log(0.35 / 0.65)

        self.cam_pos = None

        self.origin = np.array([self.n // 2, self.n // 2])
        self.world_origin = np.array([-self.size/2, -self.size/2], dtype=np.float32)
        self.T_wc = None
        self.l_min = -6
        self.l_max =  6

    def world_to_grid(self, x, z):
        gx = np.floor((x - self.world_origin[0]) / self.res).astype(int)
        gz = np.floor((z - self.world_origin[1]) / self.res).astype(int)
        return gx, gz

    def grid_to_world(self, gx, gz):
        x = (gx + 0.5) * self.res + self.world_origin[0]
        z = (gz + 0.5) * self.res + self.world_origin[1]
        return x, z

    def _bresenham(self, x0, y0, x1, y1):
        x0 = int(x0); y0 = int(y0); x1 = int(x1); y1 = int(y1)
        dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _clip_log_odds(self):
        np.clip(self.log_odds, self.l_min, self.l_max, out=self.log_odds)

    def update_from_frame(self, frame, full_pc, normals, max_range=1.5):
        self.T_wc = frame.T_wc
        cam_pos = self.T_wc[:3, 3]
        self.cam_pos = cam_pos

        h, w = full_pc.shape[:2]

        h0 = int(h * 0.7)
        h1 = int(h * 0.8)
        pc = full_pc[h0:h1:].reshape(-1, 3)
        n  = normals[h0:h1:].reshape(-1, 3)

        valid = (
           
            np.isfinite(n).all(axis=1) &
            (np.abs(n[:, 1]) < 0.1) &
            (np.linalg.norm(pc, axis=1) < max_range)
        )
        pc = pc[valid]

        if len(pc) == 0:
            return

        #points -> world
        pc_w = (self.T_wc[:3, :3] @ pc.T).T + cam_pos

        m = pc_w.shape[0]
        
        idx = np.random.choice(m, size=min(12000, m), replace=False)
        pc_w = pc_w[idx]
        

        #camera cell
        cgx, cgz = self.world_to_grid(cam_pos[0], cam_pos[2])
        if not (0 <= cgx < self.n and 0 <= cgz < self.n):
            return

        tmp = np.zeros((self.n, self.n), dtype=np.uint8)
        gx_all = np.floor((pc_w[:, 0] - self.world_origin[0]) / self.res).astype(np.int32)
        gz_all = np.floor((pc_w[:, 2] - self.world_origin[1]) / self.res).astype(np.int32)

        mask = (gx_all >= 0) & (gx_all < self.n) & (gz_all >= 0) & (gz_all < self.n)
        gx_all = gx_all[mask]
        gz_all = gz_all[mask]
        for gx, gz in zip(gx_all, gz_all):


            if not (0 <= gx < self.n and 0 <= gz < self.n):
                continue

            cv2.line(tmp, (int(cgx), int(cgz)), (int(gx), int(gz)), 255, 1)


        tmp[gz_all, gx_all] = 0
        ys, xs = np.where(tmp)
        self.log_odds[ys, xs] += self.l_free

        np.add.at(self.hit_count, (gz_all, gx_all), 1)

        np.minimum(self.hit_count, 255, out=self.hit_count)
        mask_occ = self.hit_count[gz_all, gx_all] >= 5
        np.add.at(self.log_odds, (gz_all[mask_occ], gx_all[mask_occ]), self.l_occ)

        self._clip_log_odds()



    # def update_from_frame(self, frame, full_pc, normals, max_range=5.0):
    #     self.T_wc = frame.T_wc
    #     cam_pos = self.T_wc[:3, 3]
    #     self.cam_pos = cam_pos


    #     if self.decay > 0:
    #         self.log_odds += (self.l0 - self.log_odds) * self.decay

    #     h, w = full_pc.shape[:2]

    #     h0 = int(h * 0.60)
    #     h1 = int(h * 0.70)
    #     pc = full_pc[h0:h1:2, ::2].reshape(-1, 3)
    #     n  = normals[h0:h1:2, ::2].reshape(-1, 3)

    #     valid = (
    #         np.isfinite(pc).all(axis=1) &
    #         np.isfinite(n).all(axis=1) &
    #         (np.abs(n[:, 1]) < 0.5) &
    #         (np.linalg.norm(pc, axis=1) < max_range)
    #     )
    #     pc = pc[valid]
    #     if len(pc) == 0:
    #         return

    #     # points -> world
    #     pc_w = (self.T_wc[:3, :3] @ pc.T).T + cam_pos

    #     # camera cell
    #     cgx, cgz = self.world_to_grid(cam_pos[0], cam_pos[2])
    #     if not (0 <= cgx < self.n and 0 <= cgz < self.n):
    #         return

    #     for p in pc_w:
    #         # endpoint cell
    #         gx, gz = self.world_to_grid(p[0], p[2])
    #         if not (0 <= gx < self.n and 0 <= gz < self.n):
    #             continue

    #         for x, y in self._bresenham(cgx, cgz, gx, gz):
    #             if x == gx and y == gz:
    #                 break
    #             self.log_odds[y, x] += self.l_free

    #         self.hit_count[gz, gx] = np.uint8(min(255, int(self.hit_count[gz, gx]) + 1))
    #         if self.hit_count[gz, gx] >= 3:
    #             self.log_odds[gz, gx] += self.l_occ

    #     self._clip_log_odds()

        

    def get_grid(self, thr=0.5):
        p = 1 - 1 / (1 + np.exp(self.log_odds))
        return (p > thr).astype(np.uint8) *200, self.world_to_grid(self.cam_pos[0], self.cam_pos[2])



    def inflate_grid(self, occ, radius_cells):
        r = radius_cells
        y, x = np.ogrid[-r:r+1, -r:r+1]
        mask = (x*x + y*y) <= r*r   

        inflated = occ.copy()
        h, w = occ.shape

        ys, xs = np.where(occ)
        for y0, x0 in zip(ys, xs):
            y1 = max(0, y0 - r)
            y2 = min(h, y0 + r + 1)
            x1 = max(0, x0 - r)
            x2 = min(w, x0 + r + 1)

            my1 = y1 - (y0 - r)
            my2 = my1 + (y2 - y1)
            mx1 = x1 - (x0 - r)
            mx2 = mx1 + (x2 - x1)

            inflated[y1:y2, x1:x2] |= mask[my1:my2, mx1:mx2]

        return inflated

    def get_inflated_grid(self, thr=0.55, robot_radius=0.14):
        p = 1 - 1 / (1 + np.exp(self.log_odds))
        occ = (p > thr)

        r_cells = int(np.ceil(robot_radius / self.res))
        occ_inflated = self.inflate_grid(occ, r_cells)

        cam_gx, cam_gz = self.world_to_grid(self.cam_pos[0], self.cam_pos[2])
        return occ_inflated.astype(np.uint8)*200, (cam_gx, cam_gz)



    def plot_plane(self):
        grid, _ = self.get_inflated_grid(thr=0.55, robot_radius=0.18)
        grid = grid.astype(np.uint8)

        extent = [
            self.world_origin[0], self.world_origin[0] + self.size,  # x_min, x_max
            self.world_origin[1], self.world_origin[1] + self.size   # z_min, z_max
        ]

        plt.figure(figsize=(12, 6))
        plt.subplot(1, 2, 1)
        plt.imshow(grid * 255, cmap="gray", origin="lower", extent=extent)
        if self.cam_pos is not None:
            plt.plot(self.cam_pos[0], self.cam_pos[2], "ro")  
        plt.title("Inflated obstacles (world coords)")
        plt.xlabel("x [m]")
        plt.ylabel("z [m]")

        grid_pos = self.world_to_grid(self.cam_pos[0], self.cam_pos[2]) if self.cam_pos is not None else None   

        plt.subplot(1, 2, 2)
        plt.imshow(grid, cmap="gray", origin="lower")
        plt.plot(grid_pos[0], grid_pos[1], "ro")
        plt.title("Inflated obstacles (grid cords)")
        plt.xlabel("grid x [cells]")
        plt.ylabel("grid z [cells]")

        plt.show()
       


        


    def add_obstacle(self, start, end, thickness_cells=1, weight=5.0, hits=10):

        x0, z0 = map(float, start.split(","))
        x1, z1 = map(float, end.split(","))

        gx0, gz0 = self.world_to_grid(x0, z0)
        gx1, gz1 = self.world_to_grid(x1, z1)

        tmp = np.zeros((self.n, self.n), dtype=np.uint8)
        cv2.line(tmp, (int(gx0), int(gz0)), (int(gx1), int(gz1)), 255, int(thickness_cells)) 

        rr, cc = np.where(tmp > 0)
        self.log_odds[rr, cc] += self.l_occ * weight
        self.hit_count[rr, cc] = np.clip(self.hit_count[rr, cc] + hits, 0, 255).astype(np.uint8)








