"""
Gamepad mapping test for the LiteWing flight script (Switch Pro / any pad).

SAFE: this does NOT connect to the drone and does NOT fly anything. It just
prints the controller's axes and pressed buttons so we can figure out which
stick is which and pick a "hold-to-fly" safety button before writing the
actual flight script.

Run it, then:
  - Push the LEFT stick   -> note which axis numbers change (that'll be vx/vy)
  - Push the RIGHT stick  -> note which axis (that'll be yaw)
  - Press the button you want as the "hold-to-fly" dead-man switch -> note its number
Ctrl+C to quit. Report the numbers back.
"""
import time
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No gamepad found. Plug it in and try again.")
    raise SystemExit(1)

js = pygame.joystick.Joystick(0)
print("Controller:", js.get_name())
print("axes:", js.get_numaxes(), " buttons:", js.get_numbuttons(), " hats:", js.get_numhats())
print("Move the sticks / press buttons. Ctrl+C to quit.\n")

try:
    while True:
        pygame.event.pump()
        axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
        pressed = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
        hats = [js.get_hat(i) for i in range(js.get_numhats())]
        print(f"axes={axes}  buttons_pressed={pressed}  hats={hats}        ", end="\r")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nbye")
finally:
    pygame.quit()
