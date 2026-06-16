"""
Set altitude-hold tuning parameters over WiFi (cflib), without cfclient's GUI.

Why: cfclient's Parameters tab won't send param writes over this UDP link, but
cflib's param.set_value() works fine (same protocol the firmware implements).
Parameters set this way STAY in the drone until it reboots, regardless of which
client is connected afterwards.

Workflow (reflash-free altitude tuning):
  1. Drone powered, on its WiFi, NOTHING else connected (close the app/cfclient).
  2. Edit the PARAMS values below, run this script  ->  it sets them and exits.
  3. Connect the phone app and fly with altitude hold to test.
  4. Not good? Tweak PARAMS, rerun, fly again.  (no firmware reflash needed)

Tuning hints:
  posCtlPid.thrustBase   hover throttle. Sinks slowly -> raise; climbs -> lower.
  velCtlPid.vzKi         vertical-speed integral. Altitude oscillates -> lower.
  posCtlPid.zKp          altitude stiffness. Oscillates/overshoots -> lower.
  posEstAlt.estAlphaAsl  baro filter. Jittery from prop wash -> raise (->0.98);
                         laggy -> lower.
"""
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie

# Set this to the SAME address cfclient/the app use to reach the drone.
DRONE_URI = "udp://192.168.43.1"

# Edit these, run, then fly via the app to test.
PARAMS = {
    "posCtlPid.thrustBase":  "24000",
    "velCtlPid.vzKi":        "6",
    "posCtlPid.zKp":         "1.0",
    "posEstAlt.estAlphaAsl": "0.95",
}

cflib.crtp.init_drivers()
cf = Crazyflie(rw_cache="./cache")

_connected = False
def _on_connected(uri):
    global _connected
    _connected = True

cf.connected.add_callback(_on_connected)
print("Connecting to", DRONE_URI, "...")
cf.open_link(DRONE_URI)

# Wait until connected (param TOC downloaded)
for _ in range(100):
    if _connected:
        break
    time.sleep(0.1)
if not _connected:
    print("ERROR: could not connect. Check DRONE_URI / WiFi / nothing else connected.")
    cf.close_link()
    raise SystemExit(1)
time.sleep(1.0)  # let the param TOC finish

for name, val in PARAMS.items():
    try:
        cf.param.set_value(name, val)
        print("set", name, "=", val)
    except Exception as e:
        print("FAILED", name, ":", e)
    time.sleep(0.15)

time.sleep(0.5)
cf.close_link()
print("done - params stay set until the drone reboots. Now fly via the app to test.")
