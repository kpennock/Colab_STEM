# UTILITY: Automated node tuning + execution (kRPC 0.6.0 autopilot API)
#
# 1. Tunes one handle of the existing maneuver node until the Duna periapsis is on target
# 2. Warps to just before the burn
# 3. Points at the node, burns, cuts off, and removes the node
#
# Before running: create the node with a Duna encounter showing, make sure an engine
# is active (staged), and turn OFF MechJeb's node executor / autopilots.
import krpc, time, math
 
TARGET_PE_M = 30_000    # wanted periapsis altitude at Duna, in meters
TOL_M = 1_000           # stop tuning when within 1 km
AXIS = "radial"         # handle to adjust: "prograde", "normal", or "radial"
DO_BURN = True          # False = tune and warp only, like the old version
LEAD_S = 30             # seconds of pointing time before the burn starts
POINT_TOL_DEG = 2.0     # how well aligned before we are willing to burn
CUTOFF_DV = 0.05        # m/s remaining at which the burn ends
 
conn = krpc.connect(name="duna-correction")
sc = conn.space_center
vessel = sc.active_vessel
ctl = vessel.control
ap = vessel.auto_pilot
 
nodes = ctl.nodes
if not nodes:
    raise SystemExit("Create a maneuver node first, with a Duna encounter showing.")
node = nodes[-1]
 
 
def clamp(x, lo, hi):
    return max(lo, min(hi, x))
 
 
def pe_error():
    nxt = node.orbit.next_orbit            # the patch inside Duna's SOI
    if nxt is None:
        return None
    return abs(nxt.periapsis_altitude - TARGET_PE_M)
 
 
# ---------------------------------------------------------------- 1. tune the node
value = getattr(node, AXIS)
best = pe_error()
if best is None:
    raise SystemExit("No Duna encounter on this node yet; nudge it by hand first.")
 
step = 1.0                                  # m/s
while best > TOL_M and step > 1e-4:
    improved = False
    for trial in (value + step, value - step):
        setattr(node, AXIS, trial)
        time.sleep(0.05)
        err = pe_error()
        if err is not None and err < best:
            value, best, improved = trial, err, True
            break
    if not improved:
        setattr(node, AXIS, value)          # put it back
        step /= 2
 
setattr(node, AXIS, value)                  # apply the final adjusted value to the node
nxt = node.orbit.next_orbit
print(f"{AXIS} = {value:.4f} m/s, total dv = {node.delta_v:.2f} m/s, "
      f"Duna periapsis = {nxt.periapsis_altitude/1000:.1f} km")
 
# ---------------------------------------------------------------- 2. burn timing
if vessel.available_thrust < 1.0:
    raise SystemExit("No active engine (available thrust is 0). Stage/activate the "
                     "transfer engine and run again; the node has been tuned already.")
isp = vessel.specific_impulse
if isp <= 0:
    raise SystemExit("Vessel reports Isp = 0 -- no active engine.")
 
ve = isp * 9.80665
m0 = vessel.mass
dv = node.delta_v
burn_time = (m0 - m0 / math.exp(dv / ve)) / (vessel.available_thrust / ve)
burn_start = node.ut - burn_time / 2.0
print(f"Burn time ~{burn_time:.1f} s; burn starts at T-{node.ut - burn_start:.0f} s "
      f"relative to the node")
 
if not DO_BURN:
    sc.warp_to(burn_start - LEAD_S)
    print("DO_BURN is False -- warped only.")
    raise SystemExit
 
# ---------------------------------------------------------------- 3. warp, point, burn
if burn_start - sc.ut > LEAD_S + 10:
    sc.warp_to(burn_start - LEAD_S)
 
ctl.sas = False
ap.reference_frame = node.reference_frame
ap.target_direction = (0, 1, 0)             # +y in the node frame = burn direction
ap.engaged = True
 
# wait until we are actually pointing at the node (max 60 s)
deadline = time.time() + 60
while ap.error > POINT_TOL_DEG and time.time() < deadline:
    time.sleep(0.2)
print(f"Pointing error {ap.error:.1f} deg")
if ap.error > POINT_TOL_DEG * 3:
    ctl.throttle = 0.0
    ap.engaged = False
    raise SystemExit("Could not align with the node (reaction wheels weak? RCS off?). "
                     "Aborting before the burn.")
 
while sc.ut < burn_start:
    time.sleep(0.1)
 
print("Burn START")
try:
    while True:
        if vessel.available_thrust < 1.0:
            print("ABORT: engine out of thrust mid-burn")
            break
        rem = node.remaining_delta_v
        along = node.remaining_burn_vector(node.reference_frame)[1]
        if rem < CUTOFF_DV or along < 0:
            break
        accel = max(vessel.available_thrust / vessel.mass, 0.1)
        ctl.throttle = clamp(rem / accel, 0.02, 1.0)   # eases off over the last second
        time.sleep(0.02)
finally:
    ctl.throttle = 0.0
 
print(f"Burn END, {node.remaining_delta_v:.2f} m/s remaining")
node.remove()
ap.engaged = False
ctl.sas = True
print("Done. Check the map: your Duna periapsis should match the target.")
 