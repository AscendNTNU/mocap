from pymavlink import mavutil
import time
import math


master = mavutil.mavlink_connection("udp:127.0.0.1:14553")


def set_pos(x, y, z):
    master.mav.set_position_target_local_ned(
        0,
        master.target_system,
        0,
        1,  # msg.CORDINATE_FRAME,
        4088,
        x,
        y,
        z,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


i = 0
while True:
    time.sleep(1)
    set_pos(1.5 * math.sin(i), 1.5, -(1.5 * math.cos(i)))
    i += 0.01
