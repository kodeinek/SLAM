import cv2
import torch
from yolo_standalone import YoloDetector, opt
import numpy as np
from ttictoc import tic, toc
import logging
import datetime
from collections import defaultdict
from zed_player import ZedPlayer
import math as m
# from connect_roomba import get_set_value
from connect_roomba_3 import RoombaConnectionWiFi
#from connect_roomba_2 import RoombaConnection
import time
from scipy.spatial.distance import cdist
from trajectory import gen_trajectry
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pickle

FROM_VIDEO = False
IMAGE_SCALE = 1
# IMAGE_SCALE = 1.38
WIFI = True
POINT_CLOSENESS_EPSILON = 20
PIXELS_EPS = 50
#POINT_CLOSENESS_EPSILON = 15qqqqqqqq

colors_for_robots = [
    (21, 244, 94),
    (181, 37, 228),
    (12, 220, 220),
    (0, 0, 128)
]

SELECTED_ROBOT = 2
N_ROBOTS = 4

if FROM_VIDEO:
    zp = ZedPlayer(r'D:\zed_video\irobot_zed2i\irobot_zed_02.svo')
else:
    vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280 * 2)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    fps = vid.get(cv2.CAP_PROP_FPS)

    print(f"{fps} frames per second")




#exit()

yd = YoloDetector(**vars(opt))

now = datetime.datetime.now()
filename = now.strftime("%m%d%Y_%H%M%S")
#

out = cv2.VideoWriter(f'{filename}.avi',
                          cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 25, (1280,720))


logging.basicConfig(filename=f'irobot_{filename}.log', level=logging.DEBUG,  # datefmt='%a, %d %b %Y %H:%M:%S',
                    format='%(message)s,', force=True)

logg = logging.getLogger()
fhandler = logging.FileHandler(f'irobot_{filename}.log', mode='a')
logg.addHandler(fhandler)

#PIXELS_EPS = 25


if WIFI:
    rc_wifi = RoombaConnectionWiFi()
    rc_wifi.connect_wifi('192.168.250.55', 1883, 'user2', 'user2')
else:
    rc = RoombaConnection()
    rc.connnect()
BEFORE_FIRST_MOVE = True

logg.info("robot,k,t,robots_state[0],robots_state[1],robots_state[2],pp[0],pp[1],pp[2],v,rad,e_x,e_y,e_al")


def rotate_point(point, angle):
    point2 = point.copy()
    angle_radians = m.radians(angle)
    rotation_matrix = np.array(
        [[m.cos(angle_radians), -m.sin(angle_radians)], [m.sin(angle_radians), m.cos(angle_radians)]])
    point2 = np.dot(rotation_matrix, point2)
    return point2


# N_ROBOTS_del = 2


# def compute_position_and_angles(yolo_pred):
#     x = np.hstack((yolo_pred[:, 0].reshape(-1, 1), yolo_pred[:, 2].reshape(-1, 1)))
#     y = np.hstack((yolo_pred[:, 1].reshape(-1, 1), yolo_pred[:, 3].reshape(-1, 1)))
#
#     x = np.mean(x, axis=1)
#     y = np.mean(y, axis=1)
#
#     angle_vectors = []
#     for i in range(0, yolo_pred.shape[0], 2):
#         v_i = [x[i + 1] - x[i], y[i + 1] - y[i]]
#         angle_vectors.append(v_i)
#         # print(v_i)
#     angle_vectors = np.array(angle_vectors).squeeze()
#     angles = np.arctan2(angle_vectors[:, 0] * -1, angle_vectors[:, 1])
#     angles = np.round(np.degrees(angles),4)
#     resp = np.vstack([x[range(1, N_ROBOTS_del * 2, 2)], y[range(1, N_ROBOTS_del * 2, 2)], angles]).T
#     return resp

def check_detection_correctness(yolo_pred):
    robots_found_state = [False] * N_ROBOTS
    unique_classes, unique_classes_founds = np.unique(yolo_pred[:, -1], return_counts=True)
    classes_counter = defaultdict(lambda: 0, zip(unique_classes, unique_classes_founds))

    all_classes = np.arange(N_ROBOTS * 2).reshape(N_ROBOTS, 2)
    for i, rf in enumerate(robots_found_state):
        if classes_counter[all_classes[i, 0]] == 1 and classes_counter[all_classes[i, 1]] == 1:
            robots_found_state[i] = True
    return robots_found_state


def get_robot_state(yolo_pred, robots_detection_correctness):
    resp = [[None] * 3 for i in range(len(robots_detection_correctness))]
    for i, detection in enumerate(robots_detection_correctness):
        if detection:
            idx_symbol = 2 * i
            idx_center = 2 * i + 1

            pos = yolo_pred[np.logical_or(yolo_pred[:, -1] == idx_center, yolo_pred[:, -1] == idx_symbol)]
            x = np.hstack((pos[:, 0].reshape(-1, 1), pos[:, 2].reshape(-1, 1)))
            y = np.hstack((pos[:, 1].reshape(-1, 1), pos[:, 3].reshape(-1, 1)))
            x = np.mean(x, axis=1)
            y = np.mean(y, axis=1)

            v_i = [x[0] - x[1], y[0] - y[1]]
            angle_vectors = np.array(v_i).squeeze()

            angle = np.arctan2(angle_vectors[1], angle_vectors[0])
            # angle = np.arctan2(angle_vectors[0] * -1, angle_vectors[1])
            angle = np.round(np.degrees(angle), 5)

            resp[i][0] = x[1]
            resp[i][1] = y[1]
            resp[i][2] = angle + 90

    return resp


def modulo2pi(a):
    a = np.mod(a, 2 * np.pi)
    if a > np.pi:
        return -(np.pi * 2 - a)
    else:
        return a


def modulo360(a):
    a = np.mod(a, 360)
    if a > 180:
        return -(360 - a)
    else:
        return a

def get_radius(S_zad, S):
    alpha = np.radians(S[2]) # S[2] w stopniach
    x1, y1 = S[0], S[1]
    x2, y2 = S_zad[0], S_zad[1]
    a = np.array([[-2*(x1-x2), -2*(y1-y2)], [-np.cos(alpha), - np.sin(alpha)]])
    b = np.array([(x2**2) - x1**2 + (y2**2 - y1**2), -x1*np.cos(alpha) - x1*np.sin(alpha)])

    sr = np.linalg.solve(a, b)
    #print(sr)
    r = np.sqrt((x1-sr[0])**2 +(y1-sr[1])**2)
    return r, sr

def control(S_zad, S):
    e_xy = np.array([S_zad[0] - S[0], S_zad[1] - S[1]])
    # en = e_xy / np.linalg.norm(e_xy)
    # r = np.array([np.cos(m.radians(robots_state[0][2])), np.sin(m.radians(robots_state[0][2]))])
    angle_S = np.degrees(np.arctan2(e_xy[1], e_xy[0])).item()  # e - zadany
    e_alfa = modulo360(angle_S - S[2])
    velocity = min(np.linalg.norm(e_xy) * 10, 300)
    # radius = -np.sign(e_alfa) * 2000 / (0.1 * np.abs(e_alfa)**1.5 + 1)
    #rad, sr = get_radius(S_zad, S)
    #radius = -np.sign(e_alfa) * rad * 1
    radius = -np.sign(e_alfa) * max(50, 2000 / (0.1 * np.abs(e_alfa) ** 1.5 + 1))
    #print("::", e_xy, e_alfa, radius, velocity)
    return velocity, radius, e_xy[0], e_xy[1], e_alfa


# PRED_INITIALIZED = False

frame_positions = np.zeros((720, 1280, 3), dtype=np.uint8)

robots_state = [[None] * 3 for i in range(4)]
robots_state_error = [[None] * 3 for i in range(4)]

k = 0
points_counter = [0, 0, 0, 0]

# vi = np.array([[300, 100], [300, 600], [900, 600], [901, 101]])
# X, Y = gen_trajectry(vi, epsilon=40, nodes=200, num_robots=4)
# vi = np.array([[285, 90], [285, 620], [1020, 620], [1021, 101]])
# X, Y = gen_trajectry(vi, epsilon=50, nodes=200, num_robots=4)
# for i in range(N_ROBOTS):
#     plt.plot(X[i], Y[i])
#
#
# plt.show()

with open('X_TRAJECTORY.pickle', 'rb') as handle:
    X = pickle.load(handle)

with open('Y_TRAJECTORY.pickle', 'rb') as handle:
    Y = pickle.load(handle)

# with open(f'X_{filename}.pickle', 'wb') as f:
#     pickle.dump(X, f, pickle.HIGHEST_PROTOCOL)
# with open(f'Y_{filename}.pickle', 'wb') as f:
#     pickle.dump(X, f, pickle.HIGHEST_PROTOCOL)

#exit()



while True:

    k += 1

    # print(k)

    if FROM_VIDEO:
        frame = zp.get_next_frame()[0][:, :, :3]
    else:
        ret, frame = vid.read()

        frame = frame[:, :frame.shape[1] // 2, :]

    # print(frame.dtype)
    # exit()

    # cv2.imshow('ww', frame)
    # cv2.waitKey(0)

    # tic()
    pred = yd.predict(frame).astype(int)
    # print(f'{toc()} s')
    pred = pred[np.argsort(pred[:, -1])]

    robots_detected = check_detection_correctness(pred)
    robots_state_updated = get_robot_state(pred, robots_detected)

    # if not PRED_INITIALIZED:
    #     pred_prev = pred
    #     PRED_INITIALIZED = True

    if pred.shape[0] == 0:
        print('no detections')
    #     break

    # update robot state
    for i in range(len(robots_state)):
        for j in range(len(robots_state[0])):

            if k > 1:

                if robots_state_updated[i][j] and (j == 0 or j == 1): # ktorakolwiek ze wspolrzednych skoczyla mocno
                    if abs(robots_state[i][j] - robots_state_updated[i][j]) > PIXELS_EPS:
                        continue

            if robots_state_updated[i][j] is not None:
                # if (j == 0 or j == 1) and robots_state[i][j] is not None:


                # if abs(robots_state[i][j] - robots_state_updated[i][j]) > PIXELS_EPS:
                #    continue
                #    pass

                robots_state[i][j] = robots_state_updated[i][j]

    # print(robots_state)

    # print('error', robots_state_error)

    for i in range(pred.shape[0]):
        # color = (0, 0, 255)
        c = pred[i, -1]
        color = colors_for_robots[c // 2]
        thickness = 1
        if pred[i, -1] % 2 == 1:
            thickness = 2
        #    color = (100, 255, 100)
        frame = cv2.rectangle(frame, (pred[i, 0], pred[i, 1]), (pred[i, 2], pred[i, 3]), color, thickness)

    for i in range(len(robots_state)):
        if robots_state[i][0] is not None:
            #frame_positions = cv2.circle(frame_positions, (int(robots_state[i][0]), int(robots_state[i][1])), 2,
            #                             colors_for_robots[i], 2)
            p2 = np.array([[30], [0]])
            p2 = rotate_point(p2, robots_state[i][2])
            p2 += np.array(robots_state[i][:2], dtype=int).reshape(2, 1)
            frame = cv2.arrowedLine(frame, (int(robots_state[i][0]), int(robots_state[i][1])), (int(p2[0]), int(p2[1])),
                                    colors_for_robots[i], 3)

            # print('a')

    # pp = get_set_value()

    if BEFORE_FIRST_MOVE:
        print(robots_state)
        # trajectory_points = {'t_list': [0, 15, 30, 45, 60], 'x_list': [robots_state[0][0], 492, 750, 750, 492],
        #                     'y_list': [robots_state[0][1], 255, 255, 500, 500]}
        # t_list = np.linspace(0,40,4)
        # trajectory_points = {'t_list': t_list, 'x_list': [robots_state[SELECTED_ROBOT][0], 492, 750, 750, 492],
        #             'y_list': [robots_state[SELECTED_ROBOT][1], 600, 500, 200, 300]}
        # trajectory_points = {'t_list': t_list, 'x_list': [robots_state[SELECTED_ROBOT][0], 352, 961, 948, 345],
        #             'y_list': [robots_state[SELECTED_ROBOT][1], 173, 173, 553, 553]}
        ##trajectory_points = {'t_list': t_list, 'x_list': [robots_state[SELECTED_ROBOT][0], 352, 961, 352],
        ##             'y_list': [robots_state[SELECTED_ROBOT][1], 173, 173, 173]}



        t_list_1 = np.linspace(0, 20, 4)
        trajectory_points_1 = {'t_list': t_list_1, 'x_list': X[0],
                               'y_list': Y[0]}
        t_list_2 = np.linspace(0, 30, 4)
        trajectory_points_2 = {'t_list': t_list_1, 'x_list': X[1],
                               'y_list': Y[1]}
        t_list_3 = np.linspace(0, 30, 4)
        trajectory_points_3 = {'t_list': t_list_1, 'x_list': X[2],
                               'y_list': Y[2]}
        t_list_4 = np.linspace(0, 60, 4)
        trajectory_points_4 = {'t_list': t_list_1, 'x_list': X[3],
                               'y_list': Y[3]}
        ####################################################
        # t_list_1 = np.linspace(0, 20, 4)
        # trajectory_points_1 = {'t_list': t_list_1, 'x_list': [robots_state[0][0], 352, 961, 352],
        #                        'y_list': [robots_state[0][1], 553, 553, 553]}
        # t_list_2 = np.linspace(0, 30, 4)
        # trajectory_points_2 = {'t_list': t_list_2, 'x_list': [robots_state[1][0], 352, 961, 352],
        #                        'y_list': [robots_state[1][1], 173, 173, 173]}
        # t_list_3 = np.linspace(0, 30, 4)
        # trajectory_points_3 = {'t_list': t_list_3, 'x_list': [robots_state[2][0], 352, 961, 352],
        #                        'y_list': [robots_state[2][1], 173, 173, 173]}
        # t_list_4 = np.linspace(0, 60, 4)
        # trajectory_points_4 = {'t_list': t_list_4, 'x_list': [robots_state[3][0], 352, 961, 352],
        #                        'y_list': [robots_state[3][1], 173, 173, 173]}
        ####################################################
        # trajectory_points = {'t_list': [0, 5, 10], 'x_list': [robots_state[SELECTED_ROBOT][0], 492, 750],
        #              'y_list': [robots_state[SELECTED_ROBOT][1], 600, 500]}

        pp_list = [None] * 4

        if WIFI:
            # for i in range(N_ROBOTS):
            # pp_list[0] = rc_wifi.get_set_value(0, time.time(), **trajectory_points_1)
            # pp_list[1] = rc_wifi.get_set_value(1, time.time(), **trajectory_points_2)
            # pp_list[2] = rc_wifi.get_set_value(2, time.time(), **trajectory_points_3)
            # pp_list[3] = rc_wifi.get_set_value(3, time.time(), **trajectory_points_4)

            pp_list[0] = rc_wifi.get_set_value(0, 0, **trajectory_points_1)
            pp_list[1] = rc_wifi.get_set_value(1, 0, **trajectory_points_2)
            pp_list[2] = rc_wifi.get_set_value(2, 0, **trajectory_points_3)
            pp_list[3] = rc_wifi.get_set_value(3, 0, **trajectory_points_4)

        #        rc_wifi.get_set_value(0, time.time(), **trajectory_points)
        else:
            pass
            #pp = rc.get_set_value(time.time(), **trajectory_points)
        BEFORE_FIRST_MOVE = False
    else:
        if WIFI:
            for i in range(N_ROBOTS):
                if robots_state[i][0] is not None:
                    cur_pos = np.array(robots_state[i][0:2]).reshape(1,2)
                    if cdist(cur_pos, np.array(pp_list[i][0:2]).reshape(1,2)) < POINT_CLOSENESS_EPSILON:
                        points_counter[i] += 1
                # pp_list[i] = rc_wifi.get_set_value(i, time.time())
                pp_list[i] = rc_wifi.get_set_value(i, points_counter[i])
        else:
            pass
           # pp = rc.get_set_value(time.time())

    # frame_positions = cv2.circle(frame_positions, (int(pp_list[SELECTED_ROBOT][0]), int(pp_list[SELECTED_ROBOT][1])), 1,
    #                              (5, 5, 5), 1)

    for i in range(N_ROBOTS):
        if robots_state[i][0] is not None:
            if WIFI:
                # print('aaa', rc_wifi.get_set_value(i, time.time()), robots_state[i])
                robots_state_error[i] = np.round(np.array(rc_wifi.get_set_value(i, points_counter[i])) - robots_state[i],
                                                 4).tolist()
                # robots_state_error[i] = np.round(np.array(rc_wifi.get_set_value(i, time.time())) - robots_state[i],
                #                                  4).tolist()
            else:
                pass
                #robots_state_error[i] = np.round(np.array(rc.get_set_value(time.time())) - robots_state[i], 4).tolist()

    # frame = cv2.arrowedLine(frame, (int(robots_state[SELECTED_ROBOT][0]), int(robots_state[SELECTED_ROBOT][1])),
    #                         (int(pp_list[SELECTED_ROBOT][0]), int(pp_list[SELECTED_ROBOT][1])),
    #                         (255, 0, 0,), 3)

    # e = np.array([pp_list[SELECTED_ROBOT][0] - robots_state[SELECTED_ROBOT][0],
    #               pp_list[SELECTED_ROBOT][1] - robots_state[SELECTED_ROBOT][1]])
    # en = e / np.linalg.norm(e)

    r = np.array(
        [np.cos(m.radians(robots_state[SELECTED_ROBOT][2])), np.sin(m.radians(robots_state[SELECTED_ROBOT][2]))])
    r2 = 50 * r
    # print(r2)
    #
    # p2 = np.array([[0], [-30]])
    # p2 = rotate_point(p2, robots_state[2][2])
    # p2 += np.array(robots_state[2][:2], dtype=int).reshape(2, 1)
    # print(e)
    # angle_e = np.degrees(np.arctan2(e[1], e[0])).item()  # e - zadany
    # angle_r = np.degrees(np.arctan2(r[1], r[0])).item()  # e - zadany
    # # exit()
    # frame = cv2.arrowedLine(frame, (int(robots_state[SELECTED_ROBOT][0]), int(robots_state[SELECTED_ROBOT][1])),
    #                         (
    #                             int(robots_state[SELECTED_ROBOT][0] + r2[0]),
    #                             int(robots_state[SELECTED_ROBOT][1] + r2[1])),
    #                         (0, 0, 0), 2)

    # frame = cv2.circle(frame, (10, 50), 5,
    #                    (255, 255, 0), 5)

   # e_alfa = modulo360(-robots_state[SELECTED_ROBOT][2] + angle_e)
    # print("e=", e)
    # print('*' * 10, angle_e, angle_r, robots_state[SELECTED_ROBOT][2], e_alfa)
    # print(angle_e)

    controls_list = []
    for i in range(N_ROBOTS):
        if robots_state[i][0] is not None:
            v, rad, e_x, e_y, e_al = control(pp_list[i], robots_state[i])
        else:
            v, rad, e_x, e_y, e_al = [None] * 5

        controls_list.append([v, rad, e_x, e_y, e_al])

    # v, rad, e_x, e_y, e_al = control(pp, robots_state[SELECTED_ROBOT])

    for i in range(N_ROBOTS):
        logg.info(
            f"{i},{k},{time.time()},{robots_state[i][0]},{robots_state[i][1]},{robots_state[i][2]},{pp_list[i][0]},{pp_list[i][1]},{pp_list[i][2]},{controls_list[i][0]},{controls_list[i][1]},{controls_list[i][2]},{controls_list[i][3]},{controls_list[i][4]}")

    if k % 2 == 0:
        if WIFI:
            for i in range(N_ROBOTS):
                rc_wifi.move_with_radius(i, controls_list[i][0], controls_list[i][1])
        else:
            rc.move_with_radius(v, rad)

    # print(v, rad, e_x, e_y, e_al)

    #frame = cv2.putText(frame, str(e_alfa), (60, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0))
    # a = np.array(
    # angle_goal_sin = np.arcsin(np.cross())

    # mix cameras image and path of robot
    mask = frame_positions != 0
    np.putmask(frame, mask, frame_positions)

    frame = cv2.resize(frame, (int(frame.shape[1] * IMAGE_SCALE), int(frame.shape[0] * IMAGE_SCALE)))

    cv2.imshow('frame', frame)
    out.write(frame)
    #cv2.waitKey(1)

    # if pred.shape[0] < 2:
    #     pred = pred_prev.copy()
    #     continue

    # cond_a = pred.shape[0] % 2 != 0
    # cond_b = np.unique(pred[:, -1]).shape[0] % 2 != 0
    # print(pred)
    # ww =np.diff(pred[:,-1])
    # print(ww)
    # cond_c = np.diff(pred[:, -1]).max() != 1
    # cond_d = np.diff(pred[:, -1]).min() != 1
    # if cond_a or cond_b or cond_c or cond_d:
    #     print(cond_a, cond_b, cond_c, cond_d)
    #     pred = pred_prev.copy()
    #     # print('b')
    #     continue

    # print(pred)

    # if pred.shape[0] != N_ROBOTS * 2 or np.unique(pred[:, -1]).shape[0] != N_ROBOTS * 2:
    #     pred = pred_prev.copy()
    #     #print('b')
    #     continue
    # if np.abs(pred_prev - pred).max() > PIXELS_EPS:
    #     # print('c')
    #     pred = pred_prev.copy()
    #     continue

    # print(compute_position_and_angles(pred).flatten().tolist())

    # print(pred)
    # exit()

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
    # print('*' * 25)
    # pred_prev = pred.copy()

if WIFI:
    for i in range(N_ROBOTS):
        rc_wifi.stop_robot(i)
else:
    rc.stop_robot()
if WIFI:
    rc_wifi.disconnect_wifi()
else:
    rc.disconnect()
# fname = str(time.time()).replace('.', '')
out.release()
frame = cv2.resize(frame, (int(frame.shape[1] / IMAGE_SCALE), int(frame.shape[0] / IMAGE_SCALE)))
cv2.imwrite(f'irobot_{filename}.jpg', frame)
vid.release()
cv2.destroyAllWindows()
