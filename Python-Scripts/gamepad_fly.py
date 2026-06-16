"""
pygame gamepad flight for LiteWing/pyDrone with a HOLD-TO-FLY dead-man switch.

Replaces height-hold-joystick.py (DragonRise/raw-HID only). Uses pygame so it
works with the Switch Pro Controller.

Controls (Mode 2), confirmed with gamepad_test.py:
  RIGHT stick (axis 2/3) : forward/back + sideways  (vx / vy)
  LEFT stick  Y (axis 1) : climb / descend (center = hold height)
  LEFT stick  X (axis 0) : yaw
  Button 10              : HOLD to fly. Release = stop. (dead-man switch)

SAFETY:
  - Flies ONLY while you HOLD button 10. Release / Ctrl+C / close = immediate stop.
  - Altitude held by the barometer. Horizontal is OPEN-LOOP on pyDrone (no optical
    flow) -> it WILL drift; you steer it. Fly in a clear, open space.
"""
import time
import pygame
import cflib.crtp
from cflib.crazyflie import Crazyflie

DRONE_URI   = "udp://192.168.43.42"

DEADMAN_BTN = 10
AXIS_VY     = 2      # right stick X -> sideways
AXIS_VX     = 3      # right stick Y -> forward/back
AXIS_YAW    = 0      # left stick X  -> yaw
AXIS_Z      = 1      # left stick Y  -> climb/descend

HOVER_Z   = 0.3      # height when you grab the dead-man, metres
Z_MIN, Z_MAX = 0.2, 1.2
CLIMB_RATE = 0.4     # m/s height change at full left-stick-Y
MAX_VXY    = 0.3     # m/s
MAX_YAW    = 80.0    # deg/s
DEADZONE   = 0.15

pygame.init(); pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No gamepad found."); raise SystemExit(1)
js = pygame.joystick.Joystick(0)
print("Gamepad:", js.get_name())

def dz(v):
    return 0.0 if abs(v) < DEADZONE else v

cflib.crtp.init_drivers()
cf = Crazyflie()
print("Connecting", DRONE_URI, "...")
cf.open_link(DRONE_URI)
time.sleep(1.0)
cf.commander.send_setpoint(0, 0, 0, 0)   # unlock thrust
time.sleep(0.1)
cf.param.set_value('commander.enHighLevel', '1')
print("\nReady. HOLD button %d to fly. Release = stop. Ctrl+C to quit.\n" % DEADMAN_BTN)

flying = False
targetZ = HOVER_Z
last = time.time()
try:
    while True:
        pygame.event.pump()
        now = time.time(); dt = now - last; last = now
        if js.get_button(DEADMAN_BTN):
            vx  = -dz(js.get_axis(AXIS_VX)) * MAX_VXY    # push up = forward
            vy  =  dz(js.get_axis(AXIS_VY)) * MAX_VXY    # push right = right
            yaw =  dz(js.get_axis(AXIS_YAW)) * MAX_YAW   # push right = yaw right
            targetZ += -dz(js.get_axis(AXIS_Z)) * CLIMB_RATE * dt  # push up = climb
            targetZ = max(Z_MIN, min(Z_MAX, targetZ))
            cf.commander.send_hover_setpoint(vx, vy, yaw, targetZ)
            print(f"FLY z={targetZ:.2f} vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.0f}   ", end="\r")
            flying = True
        else:
            cf.commander.send_setpoint(0, 0, 0, 0)
            targetZ = HOVER_Z
            if flying:
                print("\n(released - stopped)"); flying = False
        time.sleep(0.05)   # 20 Hz
except KeyboardInterrupt:
    print("\nquitting")
finally:
    for _ in range(5):
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.02)
    cf.close_link()
    pygame.quit()
    print("stopped, link closed")
