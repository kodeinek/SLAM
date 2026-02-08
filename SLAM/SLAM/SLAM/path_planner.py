import heapq
import math
from collections import deque
import json
from re import S
import time
import numpy as np


def line_of_sight(grid, a, b):
    """Bresenham LOS na gridzie: 0=free, !=0=blocked."""
    x0, y0 = a
    x1, y1 = b

    H, W = grid.shape
    if not (0 <= x0 < W and 0 <= y0 < H and 0 <= x1 < W and 0 <= y1 < H):
        return False

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while True:
        if grid[y, x] != 0:
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def theta_star(grid, no_buff_grid, start, goal, allow_diag=True):

    H, W = grid.shape
    sx, sy = start
    gx, gy = goal

    if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
        print("Start or goal out of bounds")
        return None
        
    if no_buff_grid[sy, sx] != 0 or no_buff_grid[gy, gx] != 0:
        print("Start or goal blocked")
        return None

    if allow_diag:
        nbrs = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(-1,1),(1,-1),(1,1)]
    else:
        nbrs = [(-1,0),(1,0),(0,-1),(0,1)]

    def cost(a, b):
        ax, ay = a
        bx, by = b
        return math.hypot(bx - ax, by - ay)

    def h(x, y):
        return math.hypot(gx - x, gy - y)

    INF = 1e18
    gscore = {(sx, sy): 0.0}
    parent = {(sx, sy): (sx, sy)}

    openpq = []
    heapq.heappush(openpq, (h(sx, sy), (sx, sy)))
    closed = set()
    TURN_K = 0.05 

    def turn_penalty(prev, cur, nxt):

        dx1, dy1 = cur[0] - prev[0], cur[1] - prev[1]
        dx2, dy2 = nxt[0] - cur[0], nxt[1] - cur[1]

        return TURN_K * abs(dx1 * dy2 - dy1 * dx2)

    def update_vertex(s, sp):
        ps = parent[s]
        if line_of_sight(grid, ps, sp):
            base = gscore[ps] + cost(ps, sp)

            extra = 0.0 if ps == s else turn_penalty(ps, s, sp)
            newg = base + extra
            newp = ps
        else:
            base = gscore[s] + cost(s, sp)
            extra = 0.0 if parent[s] == s else turn_penalty(parent[s], s, sp)
            newg = base + extra
            newp = s

        if newg < gscore.get(sp, INF):
            gscore[sp] = newg
            parent[sp] = newp
            heapq.heappush(openpq, (newg + h(sp[0], sp[1]), sp))

    while openpq:
        _, s = heapq.heappop(openpq)
        if s in closed:
            continue
        if s == (gx, gy):
            path = [s]
            while path[-1] != (sx, sy):
                path.append(parent[path[-1]])
            path.reverse()
            return path

        closed.add(s)
        x, y = s
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= W or ny < 0 or ny >= H:
                continue
            if grid[ny, nx] != 0:
                continue
            sp = (nx, ny)
            if sp in closed:
                continue
            update_vertex(s, sp)
    print("No path found")  
    return None


class motion_controller:

    def __init__(self, socket, occupancy, turn_speed=50, fwd_speed=150):
        self.socket = socket
        self.turn_speed = turn_speed
        self.fwd_speed = fwd_speed

        self.path = None
        self.x_cur = None
        self.y_cur = None
        self.finish_time = 0.0
        self.debug = True
        self.wp = None
        self.first_update = True
        self.occupancy = occupancy

        self.preplan =  self.preplan = deque([
            (0.0, 3.7),
            (-2.34, 3.7),
            (-2.34, 0.0),
            (0.0, 0.0),
        ])

    def _send_json(self, obj):
        pass
        self.socket.sendall((json.dumps(obj) + "\n").encode("utf-8"))

    def send_stop(self):
        
        self.finish_time = time.time() 
        self._send_json({"cmd": "stop"})

    def send_move(self, alpha, t1, t2):


       
        cmd = {
            "cmd": "move",
            "alpha": float(alpha),
            "t1": float(t1),
            "t2": float(t2),
            "turn_speed": float(self.turn_speed),
            "fwd_speed": float(self.fwd_speed),
        }
        self.finish_time = time.time() + float(t1) + float(t2) + 0.2

        print(alpha, t1, t2)
        self._send_json(cmd)

    def _dist(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])


    def subsample_path(self, path_grid, min_dist=20):  
        if len(path_grid) < 2:
            return path_grid
    
        subsampled = [path_grid[0]] 
        for i in range(1, len(path_grid)):
            prev = subsampled[-1]
            curr = path_grid[i]
            dist = int(math.hypot(curr[0] - prev[0], curr[1] - prev[1]))
        
            if dist > min_dist:

                steps = max(1, dist // min_dist)
                for step in range(1, steps + 1):
                    frac = step / steps
                    interp_x = int(prev[0] + frac * (curr[0] - prev[0]))
                    interp_y = int(prev[1] + frac * (curr[1] - prev[1]))
                    subsampled.append((interp_x, interp_y))
            else:
                subsampled.append(curr)
    
        return subsampled[:50]
    def next_step(self, T_wc):

        if self.path is None or len(self.path) < 2:
            return 0.0, 0.0, 0.0
        if len(self.path) < 2:
            return 0,0,0

        a0 = math.atan2(T_wc[2, 2], T_wc[0, 2])
        x0, y0 = self.path.popleft()
        self.x_cur, self.y_cur = x0, y0
        x1, y1 = self.path[0]
        self.wp = (x1, y1)

        x0, _, y0 = self.occupancy.cam_pos
        x1, y1 = self.occupancy.grid_to_world(x1,y1)


        R= np.sqrt((x1-x0)**2+(y1-y0)**2)
        beta = np.arctan2( y1-y0,x1-x0)
        alpha = float(np.arctan2(np.sin(beta - a0), np.cos(beta - a0)))
        rads_per_sec =  self.turn_speed / 235
                                                        #13.2s == 2 pi
        t1 = abs(alpha) / (2*np.pi/13.2)

                                                        #9.1s == 1m
        t2 = R / (1/9.1) 

        print(R)
        print("next_step", alpha , t1, t2)
        

        return alpha, t1, t2

    def move_to(self, img ,R_wc, target_pos= None):


        target_pos = self.occupancy.world_to_grid(target_pos[0], target_pos[1])

        img_with_buffer,  current_pos = self.occupancy.get_inflated_grid(thr=0.55, robot_radius=0.21)
        if current_pos is None:
                                if self.debug:
                                    print("[move_to] current_pos None -> return")
                                self.path = None
                                return None

        new_path = theta_star(img_with_buffer, img, current_pos, target_pos)


        if new_path is None or len(new_path) < 2:
                                if self.debug:
                                    print(f"[move_to] no path: {new_path}")
                                self.send_stop()
                                
                                self.path = None
                                return None
        # if self.path is not None and len(new_path) > 2:
        #     nex, ney =  self.path[1]
        #     if int(math.hypot(current_pos[0] - nex, current_pos[1] - ney)) > 20 :
        #         new_path = self.subsample_path(new_path, min_dist=20)  # ~1m krok
        #         if self.debug:
        #             print(f"[move_to] path len: {len(new_path)} (subsampled)")

        # start lub replan 
        if self.path is None:
            if new_path is not None and len(new_path) >= 2:
                                self.path = deque(new_path)
                                alpha, t1, t2 = self.next_step(R_wc)
                                if self.debug:
                                    print(f"[move_to] start: alpha={alpha:.3f} t1={t1:.3f} t2={t2:.3f} wp={self.wp}")
                                self.send_move(alpha, t1, t2)
                                self.finish_time = time.time() + t1 + t2 + 0.3
                                return self.path
            else:
                                if self.debug:
                                    print(f"[move_to] no valid new_path on start: {new_path}")
                                self.send_stop()
                                self.path = None
                                return None

       
        if time.time() < self.finish_time and line_of_sight(img, (self.x_cur, self.y_cur), self.wp) and math.dist(new_path[1], self.wp) < 2:
                                if self.debug:
                                    print("[move_to] still moving -> return")
                                return self.path

        if self.wp is not None and self.x_cur is not None and self.y_cur is not None:
            ok = line_of_sight(img, (self.x_cur, self.y_cur), self.wp)
            if not ok:
                if self.debug:
                    print(f"[move_to] obstacle appeared wp={self.wp} -> stop+replan")
                self.send_stop()
                self.path = deque(new_path)
                alpha, t1, t2 = self.next_step(R_wc)
                if self.debug:
                    print(f"[move_to] replan: alpha={alpha:.3f} t1={t1:.3f} t2={t2:.3f} wp={self.wp}")
                self.send_move(alpha, t1, t2)
                self.finish_time = time.time() + t1 + t2 + 0.3
                return self.path
        else:

            if self.debug:
                print(f"[move_to] wp/x_cur/y_cur missing: wp={self.wp} x_cur={self.x_cur} y_cur={self.y_cur}")



        if len(self.path) >= 2 and self.x_cur is not None and self.y_cur is not None and line_of_sight(img, (self.x_cur, self.y_cur), self.path[1]):
            alpha, t1, t2 = self.next_step(R_wc)
            if self.debug:
                print(f"[move_to] next step: alpha={alpha:.3f} t1={t1:.3f} t2={t2:.3f} wp={self.wp}")
            self.send_move(alpha, t1, t2)
            self.finish_time = time.time() + t1 + t2 + 0.3
        else:
            if self.debug:
                nxt = self.path[1] if len(self.path) >= 2 else None
                print(f"[move_to] no LOS to next={nxt} -> stop+replan")
            self.send_stop()
            self.path = deque(new_path)
            alpha, t1, t2 = self.next_step(R_wc)
            if self.debug:
                print(f"[move_to] replan2: alpha={alpha:.3f} t1={t1:.3f} t2={t2:.3f} wp={self.wp}")
            self.send_move(alpha, t1, t2)
            self.finish_time = time.time() + t1 + t2 + 0.3

        return self.path


    



