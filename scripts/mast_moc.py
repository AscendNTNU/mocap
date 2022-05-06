#!/usr/bin/env python3

from pymavlink import mavutil
import sympy, math, time


master = mavutil.mavlink_connection("udp:127.0.0.1:14553")
print("Waiting for heartbeat")
master.wait_heartbeat()
print("Heartbeat received")

t = sympy.symbols("t")
T = 10
L = 2
THETA_PITCH_MAX = 4 * 2.39 * math.pi / 180
THETA_ROLL_MAX = 4 * 7.13 * math.pi / 180
PHI_PITCH = math.pi / 2
PHI_ROLL = 0

theta_pitch = THETA_PITCH_MAX * sympy.sin((t / T) * 2 * math.pi + PHI_PITCH)
theta_roll = THETA_ROLL_MAX * sympy.sin((t / T) * 2 * math.pi + PHI_ROLL)

s = sympy.Matrix(
    [
        L * sympy.sin(theta_pitch),
        L * sympy.sin(theta_roll),
        L * sympy.cos(theta_pitch) * sympy.cos(theta_roll),
    ]
)
v = sympy.diff(s)


while True:
    t = time.time()
    x, y, z = s.evalf(subs={"t": t}).col(0)
    vx, vy, vz = v.evalf(subs={"t": t}).col(0)
    time.sleep(0.08)
    print(x, y, z, vx, vy, vz)

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        0,
        1,
        0b000111000000,
        x,
        y,
        -z,
        vx,
        vy,
        -vz,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
