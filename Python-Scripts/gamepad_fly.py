"""
pygame gamepad flight for LiteWing/pyDrone with a HOLD-TO-FLY dead-man switch.

Replaces height-hold-joystick.py (which was hard-wired to a DragonRise pad and
raw-HID parsing). Uses pygame so it works with the Switch Pro Controller.

SAFETY:
  - The drone ONLY flies while you HOLD the dead-man button (DEADMAN_BTN).
    Release it (or Ctrl+C, or close the window) and it immediately stops/lands.
  - Altitude is held by the barometer at HOVER_Z metres. Horizontal motion is
    OPEN-LOOP on pyDrone (no optical flow) -> it WILL drift; you steer it. Fly in
    a clear, open space and keep your thumb on the dead-man button.

Mapping was found with gamepad_test.py (Switch Pro): one stick = axis 2 (X) /
axis 3 (Y); chosen button = 10. Adjust the constants below if needed.
"""
import time
import pygame
import cflib.crtp
from cflib.crazyflie import Crazyflie

DRONE_URI   = "udp://192.168.43.42"

DEADMAN_BTN = 10     # hold this button to fly; release = stop
AXIS_X      = 2      # stick left/right  -> vy (sideways)
AXIS_Y      = 3      # stick up/down     -> vx (forward/back)
HOVER_Z     = 0.3    # hold height, metres
MAX_VXY     = 0.3    # max horizontal speed, m/s (gentle for small spaces)
DEADZONE    = 0.15   # ignore tiny stick noise

# ---- gamepad ----
pygame.init(); pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No gamepad found."); raise SystemExit(1)
js = pygame.joystick.Joystick(0)
print("Gamepad:", js.get_name())

def dz(v):
    return 0.0 if abs(v) < DEADZONE else v

# ---- drone ----
cflib.crtp.init_drivers()
cf = Crazyflie()
print("Connecting", DRONE_URI, "...")
cf.open_link(DRONE_URI)
time.sleep(1.0)
cf.commander.send_setpoint(0, 0, 0, 0)   # unlock thrust
time.sleep(0.1)
cf.param.set_value('commander.enHighLevel', '1')
print("\nReady. HOLD button %d to fly (hover %.2fm). Release = stop. Ctrl+C to quit.\n"
      % (DEADMAN_BTN, HOVER_Z))

flying = False
try:
    while True:
        pygame.event.pump()
        hold = js.get_button(DEADMAN_BTN)
        if hold:
            vx = -dz(js.get_axis(AXIS_Y)) * MAX_VXY   # push up = forward
            vy =  dz(js.get_axis(AXIS_X)) * MAX_VXY   # push right = right
            cf.commander.send_hover_setpoint(vx, vy, 0, HOVER_Z)
            print(f"FLY  vx={vx:+.2f} vy={vy:+.2f} z={HOVER_Z}      ", end="\r")
            flying = True
        else:
            cf.commander.send_setpoint(0, 0, 0, 0)    # not holding -> motors off
            if flying:
                print("\n(released - stopped)")
                flying = False
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
