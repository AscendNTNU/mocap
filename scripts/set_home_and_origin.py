#!/usr/bin/env python3
from pymavlink import mavutil


master = mavutil.mavlink_connection("udp:127.0.0.1:14552")

master.mav.set_gps_global_origin_send(master.target_system, 599168569, 107284696, 0)
master.mav.set_home_position_send(
    master.target_system, 0, 0, 0, 0, 0, 0, [0, 0, 0, 0], 0, 0, 0
)
