#!/usr/bin/env python3
"""
duna_encounter_check.py -- READ-ONLY. Tells you what your trajectory (including your last
maneuver node) will do at Duna, even when the game shows no Duna periapsis marker.

It does NOT change the node, the ship, or the game. Run it with the ship in flight:

    python duna_encounter_check.py            # uses your LAST maneuver node (or the current orbit if none)
    python duna_encounter_check.py --node 0   # use a specific node (0 = first)

What it prints:
  * every orbit "patch" the game has predicted after the node (Kerbin / Sun / Duna ...)
  * if one of them is a Duna patch: the predicted Duna PERIAPSIS altitude
  * if not: the CLOSEST APPROACH to Duna (distance and when), compared to Duna's sphere of influence
"""
import argparse
import math
import sys

import krpc

TARGET_BODY = "Duna"
SCAN_STEP_S = 6 * 3600          # coarse scan step: 6 hours
MAX_PATCHES = 8


def refine_min(f, a, b, iters=60):
    """Golden-section search for the minimum of f on [a, b]."""
    g = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    t = (a + b) / 2.0
    return t, f(t)


def closest_approach(dist_at, t0, t1, step):
    """Scan [t0, t1] for the smallest dist_at(t); refine around the best sample."""
    best_t, best_d = t0, dist_at(t0)
    t = t0
    while t <= t1:
        d = dist_at(t)
        if d < best_d:
            best_t, best_d = t, d
        t += step
    lo, hi = max(t0, best_t - step), min(t1, best_t + step)
    return refine_min(dist_at, lo, hi)


def fmt_km(m):
    return f"{m / 1000:,.0f} km"


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--node", type=int, default=None, help="node index (default: the last node)")
    args = ap_.parse_args()

    conn = krpc.connect(name="Duna encounter check")
    try:
        sc = conn.space_center
        v = sc.active_vessel
        target = sc.bodies[TARGET_BODY]
        sun = sc.bodies["Sun"]
        frame = sun.non_rotating_reference_frame
        now = sc.ut

        nodes = list(v.control.nodes)
        if nodes:
            idx = len(nodes) - 1 if args.node is None else args.node
            node = nodes[idx]
            start = node.orbit
            print(f"Using maneuver node {idx} of {len(nodes)}: dv {node.delta_v:.1f} m/s, "
                  f"in {(node.ut - now) / 86400:.1f} days")
        else:
            start = v.orbit
            print("No maneuver node found -- using the ship's current predicted trajectory.")

        # ---- the patches the game itself has predicted ----
        print("\nPredicted orbit patches (what the game's map is drawing):")
        patches, o = [], start
        while o is not None and len(patches) < MAX_PATCHES:
            patches.append(o)
            o = o.next_orbit
        for i, p in enumerate(patches):
            line = f"  {i}: around {p.body.name:6s}  ecc {p.eccentricity:.3f}"
            if p.body.name != "Sun":
                line += f"   periapsis {p.periapsis_altitude / 1000:,.1f} km"
            print(line)

        duna_patch = next((p for p in patches if p.body.name == TARGET_BODY), None)
        if duna_patch is not None:
            pe = duna_patch.periapsis_altitude
            print(f"\n>>> DUNA ENCOUNTER: predicted periapsis {pe / 1000:,.1f} km above Duna's surface.")
            if pe < 0:
                print("    That is BELOW the surface -- the ship would hit Duna.")
            elif pe < 50_000:
                print("    That is inside Duna's atmosphere (about 50 km): you would aerobrake / burn up.")
            else:
                print("    That is above the atmosphere (about 50 km).")
            if duna_patch.eccentricity >= 1:
                print("    The Duna path is open (hyperbolic): you need a capture burn at periapsis.")
            return

        # ---- no Duna patch: measure the closest approach ourselves ----
        helio = next((p for p in patches if p.body.name == "Sun"), None)
        if helio is None:
            print("\nNo orbit around the Sun in the prediction -- the ship is still in Kerbin's "
                  "sphere of influence after this node, so there is no transfer to measure yet.")
            return

        duna_orbit = target.orbit
        t0 = max(now, node.ut if nodes else now)
        t1 = t0 + min(helio.period, 2.0 * duna_orbit.period)

        def dist_at(t):
            a = helio.position_at(t, frame)
            b = duna_orbit.position_at(t, frame)
            return math.dist(a, b)

        print("\nNo Duna encounter in the prediction. Measuring the closest approach ...")
        t_ca, d_ca = closest_approach(dist_at, t0, t1, SCAN_STEP_S)
        soi = target.sphere_of_influence
        print(f"\n>>> CLOSEST APPROACH to {TARGET_BODY}: {fmt_km(d_ca)} "
              f"in {(t_ca - now) / 86400:.1f} days")
        print(f"    {TARGET_BODY}'s sphere of influence radius: {fmt_km(soi)}")
        if d_ca <= soi:
            print("    The ship passes INSIDE Duna's sphere of influence, so an encounter should exist.")
            print("    If the map shows none, the game's conic-patch display limit is probably too low;")
            print("    check the Settings screen for the patched-conics / conic-patch limit.")
        else:
            print(f"    The ship misses Duna's sphere of influence by about {fmt_km(d_ca - soi)}.")
            print("    Adjust the node (or the departure burn) until this closest approach gets "
                  "below that radius; then the map will show a Duna periapsis.")
    finally:
        conn.close()


if __name__ == "__main__":
    sys.exit(main())
