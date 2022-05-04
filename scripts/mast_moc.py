#!/usr/bin/env python3

from pymavlink import mavutil
import time
import math


master = mavutil.mavlink_connection("udp:127.0.0.1:14553")

master.wait_heartbeat()


def set_pos(x, y, z, vx, vy):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        0,
        1,  # msg.CORDINATE_FRAME,
        0b110111000111,
        x,
        y,
        z,
        vx,
        vy,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


i = 0
while True:
    time.sleep(.25)
    x = math.cos(i)
    y = math.sin(i)
    print(f"x: {x}, y: {y}")
    vx=-math.sin(i)
    vy=math.cos(i)
    set_pos(x, y, -1, vx, vy)
    i += 0.04
