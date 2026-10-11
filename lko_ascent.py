#!/usr/bin/env python3
"""
lko_ascent.py (v4) -- fly the whole ascent to a 78 km equatorial LKO with kRPC (set TARGET_ALT below).

Stage list this script is written for (stage numbers exactly as the VAB shows them,
0-indexed, first to fire at the top of this list):
    5  Twin Boar + SRB boosters      (core limited to 20% while the SRBs burn)
    4  separatrons + radial decouplers (booster separation)
    3  TD-25 decoupler + Vector
    2  TD-25 decoupler + Poodle
    1  TD-25 decoupler               <- NEVER fired by this script
    0  parachutes                    <- NEVER fired by this script

IMPORTANT -- stage numbers vs. kRPC's stage counter:
    ctl.current_stage is a COUNTER, not "the stage that fires next". It is always
    one higher than the stage that activate_next_stage() will fire:

        counter 6 -> fires stage 5 (launch)
        counter 5 -> fires stage 4 (booster separation)
        counter 4 -> fires stage 3 (Vector)
        counter 3 -> fires stage 2 (Poodle)
        counter 2 -> would fire stage 1 (refused by this script)
        counter 1 -> would fire stage 0 (refused by this script)

    Parts report their VAB stage number via part.stage, so engines in stage 5 are
    matched with  part.stage == counter - 1.

Before running:
  * Turn OFF MechJeb's Ascent Guidance, Autostage, and any other MechJeb
    autopilot. Two controllers fighting over throttle/staging is a mess.
  * Rocket on the pad, SAS off or on (script turns it off), stage 5 not yet fired
    (the stage counter should read 6).
  * Run krpc_logger.py in another terminal as usual if you want telemetry, e.g.
        python krpc_logger.py --mission ascent-test --design A --duration 600
    This script writes its own small event log (ascent_events.csv) as well.

Run:   python lko_ascent.py
"""
import csv
import math
import sys
import time

import krpc

# ----------------------------------------------------------------------------
# Tunable constants -- change these, not the logic
# ----------------------------------------------------------------------------
TARGET_ALT = 80_000          # m, target circular orbit altitude (atmosphere ends at 70 km)
HEADING = 90                 # deg, 90 = due east = equatorial

EXPECTED_COUNTER = 6         # stage counter on the pad (VAB stages 0-5 -> counter 6)

CORE_LIMIT_WITH_BOOSTERS = 0.20   # Twin Boar thrust limiter while SRBs burn
CORE_LIMIT_AFTER = 1.00           # restored the moment the SRBs are done
SRB_CUTOFF_FRAC = 0.03            # SRBs "done" when thrust < 3% of their peak
MIN_LIFTOFF_TWR = 1.15            # refuse to launch below this (with limiter set)

TURN_START_ALT = 1_000       # m, start pitching over
TURN_END_ALT = 45_000        # m, pitch reaches 0 deg here
TURN_EXPONENT = 0.70         # <1 turns earlier/faster, 1 = linear in altitude

APO_TAPER_FRAC = 0.08        # throttle back when within 8% of target apoapsis
ATMO_TOP = 70_500            # m, Kerbin's atmosphere ends at 70 km

# Staging floor, in COUNTER terms. At counter 3 the next activation fires VAB stage 2
# (TD-25 + Poodle), which is the last stage this script may fire. At counter 2 or
# below it would fire stage 1 (decoupler that drops the Poodle) or stage 0 (chutes).
MIN_COUNTER = 3

CIRC_LEAD_S = 30             # s of pointing time before the circularization burn
BOOSTER_WAIT_TIMEOUT_S = 120 # s to wait for SRBs to burn out if apoapsis target is hit first
TRIM_TOL_M = 1_500           # m, trim burns run until both apsides are within this of TARGET_ALT
TRIM_PASSES = 0              # trim burns DISABLED: in flight they wrecked a good orbit (see notes)
TOPUP_MAX_ERR_DEG = 5.0      # coast top-up burns only fire when pointing is this close to prograde
TRIM_DONE_M = 150            # m, a trim burn stops when its apsis is this close to the target
IGNITION_MAX_ERR_DEG = 10.0  # at burn start, wait (briefly) for pointing to be this close to the commanded pitch
IGNITION_WAIT_S = 8          # ...but never more than this: a late burn is far worse than a slightly off-axis one
BURN_END_TOL_M = 1_500       # burn ends when periapsis is within this of TARGET_ALT (simulated: 76.5 x 77.9 km)
BURN_MIN_THROTTLE = 0.10     # floor on the throttle ramp-down at the end of the burn
BURN_MAX_APO_OVER_M = 20_000 # ...or ends early if the apoapsis runs this far above TARGET_ALT

# Steering from the end of the ascent to the end of the circularization burn (v4).
# The ship is steered with pitch/heading in the surface frame (the same thing that tracked within
# ~1 deg all the way up): ORBITAL PROGRADE during the coast, then an ALTITUDE-HOLDING pitch during
# the burn (it starts a few tens of degrees above the horizon, because at ~1000 m/s the ship is far
# below orbital speed and would otherwise sink into the atmosphere, and flattens to 0 at orbital speed).
STEER_RATE_DPS = 1.0         # deg/s, max change of the commanded pitch (the reaction wheels are weak)
HOLD_KV = 0.10               # 1/s,  altitude-hold feedback on vertical speed
HOLD_KP = 0.004              # 1/s,  altitude-hold: vertical speed requested per metre of altitude error
HOLD_VCLAMP = 15.0           # m/s,  limit on that requested vertical speed
HOLD_PITCH_MIN = -5.0        # deg,  limits on the burn pitch
HOLD_PITCH_MAX = 30.0

EVENT_LOG = time.strftime("ascent_events_%Y%m%d_%H%M%S.csv")   # one file per run, never overwritten

# ----------------------------------------------------------------------------
# Setup
# ----------------------------------------------------------------------------
conn = krpc.connect(name="LKO ascent")
sc = conn.space_center
v = sc.active_vessel
ctl = v.control
ap = v.auto_pilot
body = v.orbit.body
mu = body.gravitational_parameter

ut = conn.add_stream(getattr, sc, "ut")
alt = conn.add_stream(getattr, v.flight(), "mean_altitude")
apo = conn.add_stream(getattr, v.orbit, "apoapsis_altitude")
peri = conn.add_stream(getattr, v.orbit, "periapsis_altitude")
flight = v.flight()
vspeed = conn.add_stream(getattr, flight, "vertical_speed")
ospeed = conn.add_stream(getattr, v.orbit, "speed")        # orbital speed (non-rotating frame)

t0 = ut()
_log_file = open(EVENT_LOG, "w", newline="")
_log = csv.writer(_log_file)
_log.writerow(["met_s", "event", "alt_m", "apo_m", "peri_m", "stage_counter"])


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ap_engage():
    """kRPC 0.5.x has ap.engage(); this 0.6.0 build uses the ap.engaged property."""
    if hasattr(ap, "engage"):
        ap.engage()
    else:
        ap.engaged = True


def ap_disengage():
    if hasattr(ap, "disengage"):
        ap.disengage()
    else:
        ap.engaged = False


def event(msg):
    met = ut() - t0
    print(f"[T{met:+8.1f}s] {msg}   (alt {alt()/1000:.1f} km, apo {apo()/1000:.1f} km, "
          f"counter {ctl.current_stage} -> next fires stage {ctl.current_stage - 1})")
    _log.writerow([f"{met:.1f}", msg, f"{alt():.0f}", f"{apo():.0f}",
                   f"{peri():.0f}", ctl.current_stage])
    _log_file.flush()


# ----------------------------------------------------------------------------
# Staging helpers (guarded -- this is where MechJeb's autostage was going wrong)
# ----------------------------------------------------------------------------
def stage_next(reason, expect=None):
    """Fire the next stage. `expect` is the stage COUNTER value we think we're at."""
    cur = ctl.current_stage
    if expect is not None and cur != expect:
        event(f"NOT staging ({reason}): expected counter {expect}, vessel says {cur}")
        return False
    if cur < MIN_COUNTER:
        event(f"REFUSED to fire stage {cur - 1} ({reason}) -- counter {cur} is below "
              f"MIN_COUNTER={MIN_COUNTER}")
        return False
    event(f"FIRING STAGE {cur - 1}: {reason}")
    ctl.activate_next_stage()
    time.sleep(0.6)
    return True


def ensure_engine(reason):
    """If nothing can produce thrust, fire the next stage (guarded). True if thrust exists."""
    for _ in range(3):
        if v.available_thrust > 1.0:
            return True
        if not stage_next(reason):
            return False
    return v.available_thrust > 1.0


# ----------------------------------------------------------------------------
# Identify engines in the first stage
# ----------------------------------------------------------------------------
stage_counter = ctl.current_stage
if stage_counter != EXPECTED_COUNTER:
    sys.exit(f"Stage counter is {stage_counter}, but this script expects {EXPECTED_COUNTER} "
             f"(VAB stages 0-5, nothing fired yet). Is the right rocket on the pad and has "
             f"anything already been staged? Edit EXPECTED_COUNTER / MIN_COUNTER if the "
             f"design changed.")

first_stage = stage_counter                 # counter value used for the launch guard
engine_stage = stage_counter - 1            # VAB stage number that holds the launch engines

stage_engines = [e for e in v.parts.engines if e.part.stage == engine_stage]
srbs = [e for e in stage_engines if "SolidFuel" in e.propellant_names]
core = [e for e in stage_engines if "SolidFuel" not in e.propellant_names]

print(f"Stage counter = {stage_counter}; launch stage (VAB) = {engine_stage}: "
      f"{len(core)} liquid engine(s), {len(srbs)} SRB(s)")
for e in stage_engines:
    print(f"   {e.part.title:28s} propellants={e.propellant_names}")
if not core:
    print("\nEngines found on this vessel (title, VAB stage):")
    for e in v.parts.engines:
        print(f"   {e.part.title:28s} stage={e.part.stage}")
    sys.exit(f"No liquid core engine found in VAB stage {engine_stage} -- check the stage list.")
if not srbs:
    print("WARNING: no SRBs found in the launch stage; booster logic will be skipped.")

for e in core:
    e.thrust_limit = CORE_LIMIT_WITH_BOOSTERS

liftoff_thrust = sum(e.max_thrust for e in stage_engines)   # includes the limiter
liftoff_twr = liftoff_thrust / (v.mass * body.surface_gravity)
event(f"Pre-launch: core limiter {CORE_LIMIT_WITH_BOOSTERS:.0%}, liftoff TWR {liftoff_twr:.2f}")
if liftoff_twr < MIN_LIFTOFF_TWR:
    sys.exit(f"Liftoff TWR {liftoff_twr:.2f} < {MIN_LIFTOFF_TWR}. Fix the design or raise the "
             f"limiter (CORE_LIMIT_WITH_BOOSTERS) and try again.")

# ----------------------------------------------------------------------------
# Booster handling (callable from anywhere, so separation can't be skipped when
# the apoapsis target is reached while the SRBs are still burning)
# ----------------------------------------------------------------------------
boost = {"gone": not srbs, "peak": 1.0, "last_dbg": 0.0}
_last_status = [0.0]
_cmd_throttle = [None]       # last throttle the ascent loop asked for (to spot overrides)


def dump_engines(tag):
    """Print every engine's state -- shows at a glance what stopped producing thrust."""
    print(f"   [{tag}] counter {ctl.current_stage}, throttle cmd {ctl.throttle:.2f}, "
          f"vessel thrust {v.thrust/1000:.0f} kN (available {v.available_thrust/1000:.0f} kN)")
    for e in v.parts.engines:
        print(f"      {e.part.title:28s} active={e.active} thrust={e.thrust/1000:.0f} kN "
              f"has_fuel={e.has_fuel} limit={e.thrust_limit:.2f} stage={e.part.stage}")


def status(tag, pitch_cmd=None):
    """Every 2 s: state line, printed AND saved to the event CSV (Jupyter truncates long output)."""
    if time.time() - _last_status[0] < 2.0:
        return
    _last_status[0] = time.time()
    t1 = time.time()
    thrust = v.thrust
    rtt = time.time() - t1
    pitch_now = v.flight().pitch
    err = ap.error
    cmd = f"{pitch_cmd:.0f}" if pitch_cmd is not None else "-"
    thr_now = ctl.throttle
    thr_txt = f"{thr_now:.2f}"
    if _cmd_throttle[0] is not None and abs(thr_now - _cmd_throttle[0]) > 0.1:
        thr_txt = f"{thr_now:.2f} (OVERRIDDEN, script asked {_cmd_throttle[0]:.2f})"
    line = (f"alt {alt()/1000:.1f} km  apo {apo()/1000:.1f} km  throttle {thr_txt}  "
            f"thrust {thrust/1000:.0f} kN  pitch cmd {cmd} actual {pitch_now:.0f} "
            f"err {err:.1f} deg  rtt {rtt*1000:.0f} ms")
    av = sc.active_vessel.name
    if av != v.name:
        line += f"  ACTIVE VESSEL CHANGED to {av}"
    print(f"   [{tag}] {line}")
    _log.writerow([f"{ut() - t0:.1f}", f"status[{tag}]: {line}", f"{alt():.0f}",
                   f"{apo():.0f}", f"{peri():.0f}", ctl.current_stage])
    _log_file.flush()


def boosters_spent():
    thr = sum(e.thrust for e in srbs)
    boost["peak"] = max(boost["peak"], thr)
    out_of_fuel = all(not e.has_fuel for e in srbs)
    tailed_off = boost["peak"] > 1e4 and thr < SRB_CUTOFF_FRAC * boost["peak"]
    return out_of_fuel or tailed_off


def handle_boosters():
    """Separate the SRBs once, as soon as they are spent. Safe to call every loop pass."""
    if boost["gone"]:
        return
    spent = boosters_spent()
    now = ut()
    if now - boost["last_dbg"] >= 10:          # periodic status so the console shows what we see
        boost["last_dbg"] = now
        print(f"   [SRB check] thrust {sum(e.thrust for e in srbs)/1000:.0f} kN, "
              f"peak {boost['peak']/1000:.0f} kN, has_fuel {[e.has_fuel for e in srbs]}")
    if not spent:
        return
    for e in core:
        e.thrust_limit = CORE_LIMIT_AFTER
    event("SRBs spent -> core limiter restored to 100%")
    # counter is first_stage - 1 (= 5) now; this fires VAB stage 4
    if not stage_next("booster separation", expect=first_stage - 1):
        event("WARNING: booster separation did not fire -- continuing without it")
    dump_engines("right after booster separation")
    boost["gone"] = True


def circularization_dv():
    """(dv, time of apoapsis, speed at apoapsis, circular speed there) from vis-viva."""
    o = v.orbit
    r_apo = o.apoapsis                         # from planet centre, m
    v_at_apo = math.sqrt(mu * (2.0 / r_apo - 1.0 / o.semi_major_axis))
    v_circ = math.sqrt(mu / r_apo)
    return v_circ - v_at_apo, ut() + o.time_to_apoapsis, v_at_apo, v_circ


# ----------------------------------------------------------------------------
# Steering helpers for the coast and the circularization burn
# ----------------------------------------------------------------------------
_steer = {"cmd": None, "t": 0.0}


def steer_begin():
    """Switch the autopilot to surface-frame pitch/heading, starting from the ship's current pitch."""
    ap.reference_frame = v.surface_reference_frame
    _steer["cmd"] = flight.pitch
    _steer["t"] = time.time()
    ap.target_pitch_and_heading(_steer["cmd"], HEADING)
    ap_engage()


def steer_resync():
    """Call after a time warp (the ship's attitude is frozen on rails): restart from where it points."""
    _steer["cmd"] = flight.pitch
    _steer["t"] = time.time()


def steer(target_pitch):
    """Command `target_pitch`, but never change the command faster than STEER_RATE_DPS."""
    now = time.time()
    dt = clamp(now - _steer["t"], 0.0, 0.5)
    _steer["t"] = now
    step = STEER_RATE_DPS * dt
    _steer["cmd"] += clamp(target_pitch - _steer["cmd"], -step, step)
    ap.target_pitch_and_heading(_steer["cmd"], HEADING)
    return _steer["cmd"]


def prograde_pitch():
    """Pitch above the local horizon of the ORBITAL velocity vector (flight path angle)."""
    vs = vspeed()
    vh = math.sqrt(max(ospeed() ** 2 - vs ** 2, 1.0))
    return math.degrees(math.atan2(vs, vh))


def altitude_hold_pitch():
    """Pitch that keeps the ship at TARGET_ALT while the speed builds up (gravity - centripetal)."""
    r = body.equatorial_radius + alt()
    vs = vspeed()
    vh = math.sqrt(max(ospeed() ** 2 - vs ** 2, 1.0))
    g = mu / (r * r)
    accel = max(v.available_thrust / v.mass, 1.0)
    vs_want = clamp((TARGET_ALT - alt()) * HOLD_KP, -HOLD_VCLAMP, HOLD_VCLAMP)
    up_needed = (g - vh * vh / r) + HOLD_KV * (vs_want - vs)        # m/s^2 the engine must supply upward
    return clamp(math.degrees(math.asin(clamp(up_needed / accel, -0.5, 0.9))),
                 HOLD_PITCH_MIN, HOLD_PITCH_MAX)


def burn_steering_pitch(t_rel):
    """t_rel = seconds relative to burn start. Prograde -> altitude-hold, blended over CIRC_LEAD_S."""
    if t_rel >= 0:
        return altitude_hold_pitch()
    gam = prograde_pitch()
    if t_rel < -CIRC_LEAD_S:
        return gam
    f = (t_rel + CIRC_LEAD_S) / CIRC_LEAD_S
    return gam + (altitude_hold_pitch() - gam) * f


def throttle_selftest():
    """Before igniting anything: prove nothing else is fighting kRPC for the throttle."""
    for target in (1.0, 0.0):
        ctl.throttle = target
        time.sleep(0.5)
        got = ctl.throttle
        if abs(got - target) > 0.05:
            ctl.throttle = 0.0
            sys.exit(f"THROTTLE SELF-TEST FAILED: wrote {target:.2f}, the game reports {got:.2f}. "
                     f"Something else is controlling the throttle (a MechJeb autopilot or throttle "
                     f"limiter, a game controller/joystick, or a stuck key). Fix that, then relaunch.")
    print("Throttle self-test passed: kRPC controls the throttle.")


def cut_throttle(tag=""):
    """Set throttle to 0 and confirm by reading it back; warn loudly if it does not stick."""
    for _ in range(5):
        try:
            ctl.throttle = 0.0
            time.sleep(0.1)
            if ctl.throttle < 0.01:
                return True
        except Exception as exc:
            print(f"(cut_throttle {tag}: {exc})")
    print(f"WARNING: throttle did not go to zero ({tag}) -- cut it in the game with X!")
    return False


# ----------------------------------------------------------------------------
# Trim burn: a short prograde/retrograde burn at one apsis that moves the OTHER apsis
# to target_alt_m. Fixes the small eccentricity a long finite circularization burn leaves.
# ----------------------------------------------------------------------------
def trim_burn(at_periapsis, target_alt_m, label):
    o = v.orbit
    r_other_new = body.equatorial_radius + target_alt_m
    if at_periapsis:
        r_burn, t_burn = o.periapsis, ut() + o.time_to_periapsis
    else:
        r_burn, t_burn = o.apoapsis, ut() + o.time_to_apoapsis
    v_now = math.sqrt(mu * (2.0 / r_burn - 1.0 / o.semi_major_axis))
    a_new = (r_burn + r_other_new) / 2.0
    v_new = math.sqrt(mu * (2.0 / r_burn - 1.0 / a_new))
    dv = v_new - v_now
    if abs(dv) < 0.3:
        event(f"{label}: dv {dv:+.2f} m/s is negligible, skipping")
        return False

    event(f"{label}: estimated dv {dv:+.2f} m/s at {'periapsis' if at_periapsis else 'apoapsis'}")
    if not ensure_engine(f"{label} needs an engine"):
        return False
    isp = v.specific_impulse
    if isp <= 0:
        return False
    ve = isp * 9.80665
    m0 = v.mass
    burn_time = (m0 - m0 / math.exp(abs(dv) / ve)) / (v.available_thrust / ve)
    burn_start = t_burn - burn_time / 2.0
    m_per_ms = 4.0 * o.semi_major_axis / v_now      # metres of apsis shift per m/s of dv

    if burn_start - ut() > CIRC_LEAD_S + 10:
        sc.warp_to(burn_start - CIRC_LEAD_S)
    ap.reference_frame = v.orbital_reference_frame   # +y = orbital prograde
    ap.target_direction = (0, 1 if dv > 0 else -1, 0)   # prograde to raise, retrograde to lower
    ap_engage()
    t_point = time.time()
    while ap.error > 3.0 and time.time() - t_point < 25 and ut() < burn_start:
        time.sleep(0.1)
    while ut() < burn_start:
        time.sleep(0.05)

    t_burn0 = time.time()
    try:
        while True:
            if v.available_thrust < 1.0:
                break
            # distance of the apsis we are moving from its target, straight from the orbit
            err_m = (apo() - target_alt_m) if at_periapsis else (target_alt_m - peri())
            if err_m < TRIM_DONE_M or time.time() - t_burn0 > 30:
                break
            rem = err_m / m_per_ms                          # remaining dv, m/s (estimate)
            accel = max(v.available_thrust / v.mass, 0.1)
            ctl.throttle = clamp(rem / accel, 0.02, 1.0)    # gentle: these burns are tiny
            time.sleep(0.02)
    finally:
        cut_throttle("trim burn")
    event(f"{label} done -> periapsis {peri()/1000:.1f} km, apoapsis {apo()/1000:.1f} km")
    return True


# ----------------------------------------------------------------------------
# Main sequence
# ----------------------------------------------------------------------------
try:
    # ---- Launch ----
    throttle_selftest()
    ctl.sas = False
    ctl.rcs = False
    ap.reference_frame = v.surface_reference_frame
    ap.target_pitch_and_heading(90, HEADING)
    ap_engage()
    ctl.throttle = 1.0
    for n in (3, 2, 1):
        print(f"{n}...")
        time.sleep(1)
    stage_next("launch (Twin Boar + boosters)", expect=first_stage)
    time.sleep(0.5)
    event("Liftoff")

    # ---- Atmospheric ascent: gravity turn, booster handling, throttle taper ----
    while True:
        a = alt()
        frac = clamp((a - TURN_START_ALT) / (TURN_END_ALT - TURN_START_ALT), 0.0, 1.0)
        pitch = 90.0 - 90.0 * frac ** TURN_EXPONENT
        ap.target_pitch_and_heading(pitch, HEADING)

        if not boost["gone"]:
            handle_boosters()
        else:
            ensure_engine("active engine ran dry during ascent")
        status("ascent", pitch)

        remaining = TARGET_ALT - apo()
        band = TARGET_ALT * APO_TAPER_FRAC
        if remaining <= 0:
            break
        thr_cmd = 1.0 if remaining > band else clamp(remaining / band, 0.10, 1.0)
        ctl.throttle = thr_cmd
        _cmd_throttle[0] = thr_cmd
        time.sleep(0.1)

    ctl.throttle = 0.0
    event("Target apoapsis reached, engines off")

    # If the apoapsis target was hit while the SRBs were still burning (they can't be
    # throttled), hold surface prograde, wait for them to burn out, then separate.
    if not boost["gone"]:
        event("Apoapsis target reached with SRBs still attached -- waiting to separate them")
        ap.reference_frame = v.surface_velocity_reference_frame
        ap.target_direction = (0, 1, 0)
        deadline = ut() + BOOSTER_WAIT_TIMEOUT_S
        while not boost["gone"] and ut() < deadline:
            handle_boosters()
            time.sleep(0.1)
        if not boost["gone"]:
            event("WARNING: boosters still attached after waiting -- continuing anyway")

    # ---- Early circularization node (shown on the map; also gives the burn timing) ----
    dv, t_apo, v_at_apo, v_circ = circularization_dv()
    node = ctl.add_node(t_apo, prograde=dv)
    event(f"Early node: dv ~{dv:.0f} m/s -- coasting on orbital prograde")

    # ---- Coast out of the atmosphere on orbital prograde (needs almost no turning), topping up apoapsis ----
    steer_begin()
    while alt() < ATMO_TOP:
        pc = steer(prograde_pitch())
        status("coast", pc)
        if apo() < TARGET_ALT - 200 and ap.error < TOPUP_MAX_ERR_DEG:
            if ensure_engine("top-up burn needs an engine"):
                ctl.throttle = clamp((TARGET_ALT - apo()) / 2000.0, 0.05, 0.30)
        else:
            ctl.throttle = 0.0
        time.sleep(0.1)
    ctl.throttle = 0.0
    event(f"Out of atmosphere, apoapsis {apo()/1000:.1f} km")

    # ---- Circularization node: refresh the early node with the final numbers ----
    dv, t_apo, v_at_apo, v_circ = circularization_dv()
    node.prograde = dv
    node.ut = t_apo
    event(f"Circularization node: dv = {dv:.1f} m/s (v_apo {v_at_apo:.0f} -> {v_circ:.0f} m/s)")

    # Make sure something can burn *before* we need it (staging at the last second is risky)
    if not ensure_engine("circularization needs an engine"):
        raise RuntimeError("No usable engine for circularization and staging was refused.")

    isp = v.specific_impulse
    if isp <= 0:
        raise RuntimeError("Vessel reports Isp = 0 -- no active engine.")
    ve = isp * 9.80665
    m0 = v.mass
    m1 = m0 / math.exp(dv / ve)
    burn_time = (m0 - m1) / (v.available_thrust / ve)
    burn_start = node.ut - burn_time / 2.0
    event(f"Burn time ~{burn_time:.1f} s, starting {burn_start - ut():.0f} s from now")

    # Warp if there is plenty of time (attitude is frozen on rails, so resync afterwards)
    if burn_start - ut() > CIRC_LEAD_S + 10:
        sc.warp_to(burn_start - CIRC_LEAD_S)
        steer_resync()

    # Steer (prograde -> altitude-hold, blended over the last CIRC_LEAD_S) until burn start
    while ut() < burn_start:
        pc = steer(burn_steering_pitch(ut() - burn_start))
        status("pre-burn", pc)
        time.sleep(0.1)

    # Brief alignment check only -- a late burn is far worse than a slightly off-axis one
    t_align = time.time()
    while ap.error > IGNITION_MAX_ERR_DEG and time.time() - t_align < IGNITION_WAIT_S:
        pc = steer(burn_steering_pitch(ut() - burn_start))
        time.sleep(0.1)
    event(f"Pointing error at ignition: {ap.error:.1f} deg (waited {time.time() - t_align:.0f} s)")

    # ---- Burn: altitude-holding steering, closed loop on the real periapsis ----
    event("Circularization burn START")
    m_per_ms = 4.0 * (body.equatorial_radius + TARGET_ALT) / v_circ   # m of periapsis per m/s
    t_burn0 = time.time()
    burn_cap_s = burn_time * 1.6 + 20
    while True:
        if v.available_thrust < 1.0:
            ctl.throttle = 0.0
            if not ensure_engine("engine ran dry mid-burn"):
                event("ABORT: no engine available and staging refused")
                break
        pc = steer(burn_steering_pitch(ut() - burn_start))
        status("burn", pc)
        err_m = TARGET_ALT - peri()                    # metres of periapsis still to gain
        if err_m <= BURN_END_TOL_M:
            break
        if apo() > TARGET_ALT + BURN_MAX_APO_OVER_M:
            event("WARNING: apoapsis running away -- ending the burn early")
            break
        if time.time() - t_burn0 > burn_cap_s:
            event("WARNING: burn exceeded its time cap -- ending it")
            break
        accel = max(v.available_thrust / v.mass, 0.1)
        ctl.throttle = clamp((err_m / m_per_ms) / accel, BURN_MIN_THROTTLE, 1.0)   # eases off near the end
        time.sleep(0.05)
    ctl.throttle = 0.0
    node.remove()
    event(f"Burn END -> periapsis {peri()/1000:.1f} km, apoapsis {apo()/1000:.1f} km")

    # ---- Trim: the long finite burn leaves a slightly eccentric orbit; fix with short burns ----
    for i in range(TRIM_PASSES):
        low, high = v.orbit.periapsis_altitude, v.orbit.apoapsis_altitude
        if low >= TARGET_ALT - TRIM_TOL_M and high <= TARGET_ALT + TRIM_TOL_M:
            break
        if low < TARGET_ALT - TRIM_TOL_M:
            ok = trim_burn(False, TARGET_ALT, f"Trim {i + 1} (raise periapsis at apoapsis)")
        else:
            ok = trim_burn(True, TARGET_ALT, f"Trim {i + 1} (lower apoapsis at periapsis)")
        if not ok:
            break
    event(f"Final orbit: periapsis {peri()/1000:.1f} km, apoapsis {apo()/1000:.1f} km")

    if peri() < TARGET_ALT - 3_000:
        print("WARNING: periapsis is more than 3 km under target. Check the event log; "
              "a trim burn at apoapsis may be needed.")
    else:
        print("In orbit.")

finally:
    cut_throttle("script exiting")          # first thing, before anything that could fail
    try:
        dump_engines("script exiting")
    except Exception as exc:
        print(f"(exit dump failed: {exc})")
    try:
        ap_disengage()
    except Exception as exc:
        print(f"(autopilot disengage failed: {exc})")
    try:
        ctl.sas = True
    except Exception as exc:
        print(f"(could not re-enable SAS: {exc})")
    _log_file.close()
    try:
        conn.close()                    # don't leave a stale kRPC connection behind
    except Exception:
        pass
