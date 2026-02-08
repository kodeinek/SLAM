# pscp -r -pw user "C:\Users\jaros\source\repos\SLAM\SLAM\Sender.py" user@192.168.123.104:/home/user/slam/SLAM
# pscp -r -pw user "C:\Users\jaros\source\repos\SLAM\SLAM\roomba.py" user@192.168.123.104:/home/user/slam/SLAM
# pscp -r -pw user "C:\Users\jaros\source\repos\SLAM\SLAM\roomba_deamon.py" user@10.86.89.174:/home/user/slam/SLAM
# pscp -r -pw user "C:\Users\jaros\source\repos\SLAM\SLAM\roomba_deamon.py" user@192.168.123.104:/home/user/slam/SLAM

import time
import cv2
import numpy as np
import math



from camera import Camera
from Slam import SLAM
import occupancy
import path_planner
from visualizer import Visualizer
from occupancy import OccupancyGrid
from collections import deque
from path_planner import motion_controller
import socket, json


def init(ip = " "):
    zed = Camera(min_dist=0.3, max_dist=8)

    if not zed.open_from_live(ip, 30000):
        if not zed.open_from_svo("svo/zed.svo"):
            print("FAIL ZED OPEN")      
            raise SystemExit(1)
            #exit(1)




    # vis = Visualizer()
    occupancy = OccupancyGrid()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, 9999))
    motion= motion_controller(s,occupancy,  turn_speed=50, fwd_speed=100)
    slam = SLAM(zed, 3, False) 
    return zed, slam, occupancy, s, motion

    
def show_occupancy(f, full_pc, normals, occupancy):
    occupancy.update_from_frame(f, full_pc, normals)
    img1, pos1 = occupancy.get_grid(thr=0.55)
    img, pos = occupancy.get_inflated_grid(thr=0.55)
    return img, (pos[0], pos[1]), img1

def show_matches(slam, vis, img, f):
    pts2d_img = f.kps.reshape(-1, 2).astype(int)

    for pt in pts2d_img:
        if 0 <= pt[0] < img.shape[1] and 0 <= pt[1] < img.shape[0]:
            cv2.circle(img, tuple(pt), 2, (0, 255, 0), -1)

    pos = f"t_cw X:{f.t_cw[0]:.2f} Y:{f.t_cw[1]:.2f} Z:{f.t_cw[2]:.2f}"
    cv2.putText(img, pos, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    
    # mask = np.abs(f.normals[:,:, 1]) > 0.7
    
    # img[mask] = [100]  

    cv2.imshow("ZED-SLAM", img)
    #view = vis.draw_map_view(slam.map)
    vis.push_pose(f.T_wc)
    view = vis.draw_trajectory()
    cv2.imshow("Map view", view)
    if f.id % 3 == 0:
        cv2.imwrite("path.jpg", view)

def main():
    zed, slam, occupancy, s, motion = init("192.168.123.102")
    
    
    targets = [(0, 3.7)
            ]
    pth =deque(targets)
    #s.sendall((json.dumps(cmd) + "\n").encode("utf-8"))
    target = pth.popleft()
    
    running = True

    vis = Visualizer()
    path = None
    i = 0


    go_start = True
    while running:
        start = time.time()
        dt, ts = zed.grab()
        if dt is None:
            continue
        # if i<520:
        #     i+=1
        #     continue
        zed.get_pose_camera()

        left_img, full_pc, normals = zed.retrieve_measurements()

        f = slam.track(left_img, full_pc, normals)


        if f.id % 10 == 0:
            img, pos, img1= show_occupancy(f, full_pc, normals, occupancy)
            ps = occupancy.grid_to_world(pos[0], pos[1])

            if math.hypot(target[0] - ps[0], target[1] - ps[1]) <0.1:
                if len(pth) >=1:
                    target = pth.popleft()
                else:
                    if go_start:
                        target = (0, 0)
                        go_start = False
                    else:
                        running = False

            path = motion.move_to(img,f.T_wc, target)

            if path is None:
                if motion.debug:
                    print("[main] path is None -> stop loop")
                running = False
            else:
                cv2.circle(img, (int(target[0]), int(target[1])), 5, (0,255,0), -1)
                img[pos[1], pos[0]] = 255
                for p in path:
                    cv2.circle(img, (int(p[0]), int(p[1])), 1, (255,0,0), -1)

            img = cv2.flip(img, 0)
            img = cv2.resize(img, (800, 800), interpolation=cv2.INTER_NEAREST)
            if f.id %50 == 0 :
                cv2.imwrite(f"OCCUPANCY_process{i}.jpg", img)
                i+=1
            cv2.imshow("Occupancy", img)
        #

        print("sum time : ", time.time() - start)
        show_matches(slam, vis, left_img.copy(), f)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    img1 = cv2.flip(img1, 0)
    img1 = cv2.resize(img1, (800, 800), interpolation=cv2.INTER_NEAREST)
    for tr in targets:
        cv2.circle(img1, (int(tr[0]), int(tr[1])), 5, (111,255,111), -1)
        cv2.circle(img, (int(tr[0]), int(tr[1])), 5, (111,255,111), -1)
    cv2.imwrite("OCCUPANCY_RAW.jpg", img1)
    cv2.imwrite("OCCUPANCY_INFLATED.jpg", img)
       

if __name__ == "__main__":
    main()

