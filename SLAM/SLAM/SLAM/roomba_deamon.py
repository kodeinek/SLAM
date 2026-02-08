import json, socket
import select

from old_roomba import Roomba  
import time
HOST = "0.0.0.0"
PORT = 9999


def reinit_roomba(r):
    try:
        r.stop()
    except Exception:
        pass
    try:
        r.fullMode()
    except Exception:
        pass


r = Roomba()
r.fullMode()
running = True

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    while running:
        conn, addr = s.accept()
        with conn:
            moving = False
            phase_end = 0.0
            phase = None
            msg = None
            buf = b""
            try:
                while True:
                    now = time.time()
                    ready, _, _ = select.select([conn], [], [], 0.02)
                    if ready:
                        try:
                            data = conn.recv(4096)
                        except (ConnectionResetError, BrokenPipeError, OSError):
                            reinit_roomba(r)   # stop + fullMode
                            break

                        if not data:
                            reinit_roomba(r)
                            break

                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            if not line.strip():
                                continue
                            msg = json.loads(line.decode("utf-8"))

                    if msg is not None:
                        if msg.get("cmd") == "stop":
                            r.stop()
                            moving = False
                            phase = None

                        elif msg.get("cmd") == "move":
                            alpha = msg["alpha"]; t1 = msg["t1"]; t2 = msg["t2"]
                            turn_speed = msg["turn_speed"]; fwd_speed = msg["fwd_speed"]
                            right = int(turn_speed if alpha > 0 else -turn_speed)
                            left  = int(-turn_speed if alpha > 0 else  turn_speed)
                            r.driveDirect(right, left)

                            moving = True
                            phase = "turn"
                            phase_end = time.time() + t1

                        elif msg.get("cmd") == "fin":
                            r.stop()
                            running = False
                            break

                    if moving and now >= phase_end:
                        if phase == "turn":
                            r.driveDirect(int(fwd_speed), int(fwd_speed))
                            phase = "fwd"
                            phase_end = now + t2
                        else:
                            r.stop()
                            moving = False
                            phase = None

                    msg = None
            finally:
                try:
                    r.stop()
                except Exception:
                    pass
                





