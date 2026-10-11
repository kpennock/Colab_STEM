#!/usr/bin/env python3
"""
duna_lever_advisor.py -- tells you WHICH lever of your maneuver node to pull (prograde / normal / radial),
which direction, and by how much, to bring your trajectory to Duna.

How it works (no guessing, no rules of thumb):
  1. It finds your closest approach to Duna on the trajectory that includes your LAST maneuver node.
  2. It nudges each node lever by 1 m/s (one at a time), measures how the miss changes, and puts the
     node back exactly as it was.
  3. From those measurements it works out a step, TRIES it on the game's real predicted orbit, and keeps it
     only if the miss really got smaller (otherwise it shortens the step). It repeats a few times.

It aims at Duna's CENTER (zero miss across the flight path). That would be a crash. It gets you an encounter; then tune the
node (or use duna_correction.py) to move the Duna periapsis to the altitude you want.

    python duna_lever_advisor.py                 # search; the node is restored afterwards
    python duna_lever_advisor.py --apply         # ...and KEEP the improved node (only if the real orbit got better)
    python duna_lever_advisor.py --reset --apply # start again from a zero node (a good idea if the node has grown huge)
    python duna_lever_advisor.py --set 5 49.5 229.9   # put a node back to exact values
    python duna_lever_advisor.py --pe-km 30 --apply   # with an encounter: tune the Duna periapsis to 30 km
    python duna_lever_advisor.py --node 0        # use a specific node (default: the last one)

SAVE THE GAME FIRST (Esc > Save Game). The script changes the node temporarily.
The game must be running and NOT paused for the node's predicted orbit to update.
"""
import argparse
import math
import sys
import time

TARGET_BODY = "Duna"
SCAN_STEP_S = 6 * 3600
PROBE_DV = 1.0               # m/s nudge for each lever
SETTLE_S = 0.6               # seconds to let the game recompute the node's orbit


# ----------------------------------------------------------------------------
# Small vector / matrix helpers (pure Python -- nothing to install)
# ----------------------------------------------------------------------------
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def norm(a): return math.sqrt(dot(a, a))
def scale(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def unit(a):
    n = norm(a)
    return scale(a, 1.0 / n) if n > 0 else a


def solve3(A, b):
    """Solve the 3x3 system A x = b by Gaussian elimination with partial pivoting."""
    M = [list(A[i]) + [b[i]] for i in range(3)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-30:
            raise ZeroDivisionError("singular")
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, 3):
            f = M[r][c] / M[c][c]
            for k in range(c, 4):
                M[r][k] -= f * M[c][k]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (M[r][3] - sum(M[r][k] * x[k] for k in range(r + 1, 3))) / M[r][r]
    return tuple(x)


def refine_min(f, a, b, iters=60):
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
    best_t, best_d = t0, dist_at(t0)
    t = t0
    while t <= t1:
        d = dist_at(t)
        if d < best_d:
            best_t, best_d = t, d
        t += step
    return refine_min(dist_at, max(t0, best_t - step), min(t1, best_t + step))


# ----------------------------------------------------------------------------
# The advisor itself. `probe` hides the game: see KrpcProbe below (a simulator is used for testing).
#   probe.get_axes() -> (prograde, normal, radial) m/s      probe.set_axes(p, n, r)
#   probe.ship_pos(t), probe.duna_pos(t) -> (x, y, z) m      probe.window() -> (t0, t1)
# ----------------------------------------------------------------------------
def duna_frame(probe, t):
    r_hat = unit(probe.duna_pos(t))
    v = sub(probe.duna_pos(t + 5.0), probe.duna_pos(t - 5.0))
    n_hat = unit(cross(r_hat, v))
    t_hat = unit(cross(n_hat, r_hat))
    return r_hat, t_hat, n_hat          # radial(out), along-track(ahead), normal


def miss_vec(probe, t):
    return sub(probe.ship_pos(t), probe.duna_pos(t))


def km(m): return f"{m / 1000:,.0f} km"


def closest(probe):
    t0, t1 = probe.window()
    return closest_approach(lambda t: norm(miss_vec(probe, t)), t0, t1, SCAN_STEP_S)


def jacobian(probe, t_ca, m0):
    """Miss change ACROSS the flight path per 1 m/s of each lever (3 vectors). Leaves the node untouched."""
    rel_v = unit(sub(miss_vec(probe, t_ca + 5.0), miss_vec(probe, t_ca - 5.0)))

    def across(vec):
        return sub(vec, scale(rel_v, dot(vec, rel_v)))

    base = probe.get_axes()
    cols = []
    try:
        for axis in range(3):
            trial = list(base)
            trial[axis] += PROBE_DV
            probe.set_axes(*trial)
            cols.append(across(scale(sub(miss_vec(probe, t_ca), m0), 1.0 / PROBE_DV)))
    finally:
        probe.set_axes(*base)
    return across(m0), cols


def optimize(probe, target_m, max_passes=8, out=print):
    """
    Gauss-Newton with a safety net: every step is judged by the REAL predicted orbit, never by the
    linear estimate. A step that does not reduce the miss by at least 15% is shortened (x0.5, x0.25,
    x0.1) and, if even that does not help, the search stops. The node is never left worse than it started.
    Returns (best_axes, start_distance, best_distance).
    """
    names = ("prograde", "normal", "radial")
    start_axes = probe.get_axes()
    t_ca, d_ca = closest(probe)
    start_d = d_ca
    out(f"Start: node = {start_axes[0]:+.1f} / {start_axes[1]:+.1f} / {start_axes[2]:+.1f} m/s "
        f"(prograde / normal / radial)  ->  closest approach {km(d_ca)}")

    for k in range(max_passes):
        if d_ca <= target_m:
            out(f"Reached the goal (closest approach {km(d_ca)} <= {km(target_m)}).")
            break
        base = probe.get_axes()
        m0 = miss_vec(probe, t_ca)
        if k == 0:
            r_hat, t_hat, n_hat = duna_frame(probe, t_ca)
            mr, mt, mn = dot(m0, r_hat), dot(m0, t_hat), dot(m0, n_hat)
            out("\nAt closest approach your ship is:")
            out(f"   {km(abs(mr)):>12} {'OUTSIDE' if mr > 0 else 'INSIDE'} Duna's orbit (distance from the Sun)")
            out(f"   {km(abs(mt)):>12} {'AHEAD of' if mt > 0 else 'BEHIND'} Duna along its orbit (mostly timing)")
            out(f"   {km(abs(mn)):>12} {'NORTH' if mn > 0 else 'SOUTH'} of Duna's orbital plane")
        m0p, cols = jacobian(probe, t_ca, m0)
        if max(norm(c) for c in cols) < 1e3:
            out("\nThe node's predicted orbit did not change when I nudged the levers. Is the game paused? "
                "Nothing was changed.")
            probe.set_axes(*start_axes)
            return start_axes, start_d, start_d
        if k == 0:
            out("\nWhat 1 m/s on each lever does to your miss across the flight path (km):")
            out(f"   {'lever':9s} {'outside':>10s} {'ahead':>10s} {'north':>10s}")
            for i, c in enumerate(cols):
                out(f"   {names[i]:9s} {dot(c, r_hat) / 1000:10.1f} {dot(c, t_hat) / 1000:10.1f} "
                    f"{dot(c, n_hat) / 1000:10.1f}")
            out("   Best single lever (linear estimate):")
            for i, c in enumerate(cols):
                cc = dot(c, c)
                if cc > 0:
                    dv = -dot(c, m0p) / cc
                    left = norm(tuple(m0p[q] + c[q] * dv for q in range(3)))
                    out(f"      {names[i]:9s} {dv:+8.2f} m/s -> leftover {km(left)}")
            out("")

        # smallest CHANGE of the three levers that cancels the cross-track miss (linear estimate)
        JtJ = [[dot(cols[i], cols[j]) for j in range(3)] for i in range(3)]
        lam = 1e-6 * (JtJ[0][0] + JtJ[1][1] + JtJ[2][2])
        for i in range(3):
            JtJ[i][i] += lam
        delta = solve3(JtJ, tuple(-dot(cols[i], m0p) for i in range(3)))

        accepted = False
        for alpha in (1.0, 0.5, 0.25, 0.1):
            trial = tuple(base[i] + alpha * delta[i] for i in range(3))
            probe.set_axes(*trial)
            t2, d2 = closest(probe)
            out(f"Pass {k + 1}: step x{alpha:<4} change {alpha * delta[0]:+7.1f} / {alpha * delta[1]:+6.1f} / "
                f"{alpha * delta[2]:+7.1f} m/s -> closest approach {km(d2)}")
            if d2 < 0.85 * d_ca:
                accepted, t_ca, d_ca = True, t2, d2
                break
        if not accepted:
            probe.set_axes(*base)
            out("No step improves the real orbit any more -- stopping here.")
            break

    final = probe.get_axes()
    out(f"\nResult: node = {final[0]:+.1f} / {final[1]:+.1f} / {final[2]:+.1f} m/s, "
        f"total {norm(final):.1f} m/s  ->  closest approach {km(d_ca)}  (was {km(start_d)})")
    return final, start_d, d_ca


def hyperbola_b(a, e):
    """Aim distance ('impact parameter') of a hyperbolic flyby: the miss distance if gravity did not bend the path."""
    return abs(a) * math.sqrt(max(e * e - 1.0, 0.0))


def b_for_periapsis(a, r_p):
    """The aim distance that gives periapsis radius r_p (from the planet's centre) for this approach speed."""
    return math.sqrt(r_p * r_p + 2.0 * r_p * abs(a))


def tune_periapsis(probe, target_alt_m, tol_m=2_000.0, max_passes=8, out=print):
    """
    Move the Duna periapsis to target_alt_m using the three node levers.
    The miss across the flight path is a 2-D vector. Duna's gravity bends the path, but the 'aim distance'
    b (the miss with no gravity) still equals that miss, and b maps one-to-one onto the periapsis. So: keep the
    miss on the same side of Duna, scale it to the b that gives the wanted periapsis, and move the node by the
    smallest amount that does it. Every step is judged by the game's own predicted periapsis.
    Returns (best_axes, start_pe, end_pe), or None if there is no Duna encounter.
    """
    names = ("prograde", "normal", "radial")
    hyp = probe.duna_hyperbola()
    if hyp is None:
        return None
    radius = probe.duna_radius

    def pe_of(h):
        return abs(h[0]) * (h[1] - 1.0) - radius

    start_pe = pe_of(hyp)
    out(f"Duna periapsis now {km(start_pe)}; target {km(target_alt_m)}.")
    for k in range(max_passes):
        pe = pe_of(hyp)
        if abs(pe - target_alt_m) <= tol_m:
            out(f"Reached the target (periapsis {km(pe)}).")
            break
        base = probe.get_axes()
        t_ca, d_ca = closest(probe)
        m0 = miss_vec(probe, t_ca)
        m0p, cols = jacobian(probe, t_ca, m0)
        if max(norm(c) for c in cols) < 1e3 or norm(m0p) < 1.0:
            out("The levers do not change the miss (is the game paused?); stopping.")
            break
        b_goal = b_for_periapsis(hyp[0], radius + target_alt_m)
        if abs(pe - target_alt_m) < 500_000.0:      # close: correct by the game's own aim distance
            want = scale(unit(m0p), norm(m0p) + (b_goal - hyperbola_b(*hyp)))
        else:                                       # far: scale the miss to the aim distance we need
            want = scale(unit(m0p), b_goal)
        rhs_v = sub(want, m0p)
        if k == 0:
            out(f"Aim distance now {km(norm(m0p))}, needed {km(b_goal)}.")
            out("What 1 m/s on each lever does to the aim point (km):  " +
                "   ".join(f"{names[i]} {norm(cols[i]) / 1000:.1f}" for i in range(3)))
        JtJ = [[dot(cols[i], cols[j]) for j in range(3)] for i in range(3)]
        lam = 1e-6 * (JtJ[0][0] + JtJ[1][1] + JtJ[2][2])
        for i in range(3):
            JtJ[i][i] += lam
        delta = solve3(JtJ, tuple(dot(cols[i], rhs_v) for i in range(3)))
        before = abs(pe - target_alt_m)
        if max(abs(d) for d in delta) < 0.001:
            out(f"Remaining correction is under 0.001 m/s -- close enough (periapsis {km(pe)}).")
            break
        accepted = False
        for alpha in (1.0, 0.5, 0.25, 0.1):
            trial = tuple(base[i] + alpha * delta[i] for i in range(3))
            probe.set_axes(*trial)
            h2 = probe.duna_hyperbola()
            if h2 is None:
                out(f"Pass {k + 1}: step x{alpha:<4} loses the encounter")
                continue
            out(f"Pass {k + 1}: step x{alpha:<4} change {alpha * delta[0]:+6.2f} / {alpha * delta[1]:+6.2f} / "
                f"{alpha * delta[2]:+6.2f} m/s -> periapsis {km(pe_of(h2))}")
            if abs(pe_of(h2) - target_alt_m) < before - 200.0:
                accepted, hyp = True, h2
                break
        if not accepted:
            probe.set_axes(*base)
            hyp = probe.duna_hyperbola() or hyp
            out("No further step gets meaningfully closer -- this is as close as the levers allow.")
            break
    final = probe.get_axes()
    out(f"\nResult: node = {final[0]:+.2f} / {final[1]:+.2f} / {final[2]:+.2f} m/s, total {norm(final):.2f} m/s"
        f"  ->  Duna periapsis {km(pe_of(hyp))}  (was {km(start_pe)})")
    return final, start_pe, pe_of(hyp)


# ----------------------------------------------------------------------------
# Real game
# ----------------------------------------------------------------------------
class KrpcProbe:
    def __init__(self, conn, node_index):
        self.sc = conn.space_center
        v = self.sc.active_vessel
        nodes = list(v.control.nodes)
        if not nodes:
            sys.exit("No maneuver node on the ship. Create one first (the one you want to tune).")
        self.idx = len(nodes) - 1 if node_index is None else node_index
        self.node = nodes[self.idx]
        sun = self.sc.bodies["Sun"]
        self.frame = sun.non_rotating_reference_frame
        self.duna = self.sc.bodies[TARGET_BODY]
        self.duna_orbit = self.duna.orbit
        self.soi = self.duna.sphere_of_influence
        self.duna_radius = self.duna.equatorial_radius
        self._patch = None

    def _helio(self):
        if self._patch is None:
            o = self.node.orbit
            while o is not None and o.body.name != "Sun":
                o = o.next_orbit
            if o is None:
                sys.exit("After this node the ship is still in a planet's sphere of influence; "
                         "there is no solar-orbit trajectory to analyse yet.")
            self._patch = o
        return self._patch

    def get_axes(self):
        return (self.node.prograde, self.node.normal, self.node.radial)

    def set_axes(self, p, n, r):
        self.node.prograde, self.node.normal, self.node.radial = p, n, r
        time.sleep(SETTLE_S)
        self._patch = None

    def duna_hyperbola(self):
        """(semi-major axis, eccentricity) of the game's predicted orbit around Duna, or None if there is none."""
        o = self.node.orbit
        while o is not None:
            if o.body.name == TARGET_BODY:
                return (o.semi_major_axis, o.eccentricity)
            o = o.next_orbit
        return None

    def ship_pos(self, t): return self._helio().position_at(t, self.frame)
    def duna_pos(self, t): return self.duna_orbit.position_at(t, self.frame)

    def window(self):
        t0 = max(self.sc.ut, self.node.ut)
        return t0, t0 + min(self._helio().period, 2.0 * self.duna_orbit.period)


def main():
    import krpc
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", type=int, default=None, help="node index (default: the last one)")
    ap.add_argument("--apply", action="store_true",
                    help="leave the node at the improved values (only if the real orbit got better)")
    ap.add_argument("--reset", action="store_true",
                    help="start from a zero node (prograde = normal = radial = 0) instead of its current values")
    ap.add_argument("--set", nargs=3, type=float, metavar=("PROGRADE", "NORMAL", "RADIAL"),
                    help="just set the node to these values and exit (use this to put a node back)")
    ap.add_argument("--pe-km", type=float, default=None,
                    help="after getting an encounter, tune the Duna periapsis to this altitude (km above the surface)")
    ap.add_argument("--goal-km", type=float, default=15_000.0,
                    help="stop when the closest approach is below this (default 15,000 km; Duna's sphere is 47,922 km)")
    args = ap.parse_args()

    conn = krpc.connect(name="Duna lever advisor")
    probe, original, keep = None, None, False
    try:
        probe = KrpcProbe(conn, args.node)
        original = probe.get_axes()
        print(f"Node {probe.idx} currently: prograde {original[0]:+.1f}, normal {original[1]:+.1f}, "
              f"radial {original[2]:+.1f} m/s\n")
        if args.set:
            probe.set_axes(*args.set)
            print(f"Node set to {args.set[0]:+.1f} / {args.set[1]:+.1f} / {args.set[2]:+.1f} m/s.")
            keep = True
            return
        if args.reset:
            probe.set_axes(0.0, 0.0, 0.0)
            print("Starting from a zero node (--reset).")
        if args.pe_km is not None and probe.duna_hyperbola() is None:
            print("This node's predicted path has NO Duna encounter, so there is no periapsis to tune.")
            print("Most likely the node sits in the wrong place. Delete it and make a NEW node on the orbit line")
            print("that is still around the Sun, BEFORE the point where the path enters Duna's sphere.")
            print("(A node placed after the encounter, or on the path after leaving Duna, will never show Duna.)")
            print("Nothing was changed. Run  python duna_encounter_check.py  to see which patches exist.")
            return
        if args.pe_km is not None:
            tuned = tune_periapsis(probe, args.pe_km * 1000.0)
            final, d_orig_pe, d_end_pe = tuned
            if args.apply and abs(d_end_pe - args.pe_km * 1000.0) < abs(d_orig_pe - args.pe_km * 1000.0):
                keep = True
                print("\nApplied: the node now has the tuned values.")
            elif args.apply:
                print("\nNo improvement found, so your original node was restored.")
            else:
                print("\nNode left exactly as it was. Add --apply to keep the tuned values.")
            return
        final, d_start, d_end = optimize(probe, args.goal_km * 1000.0)
        # the 'start' of the optimiser is the reset node if --reset was used; compare with the real original
        if args.reset:
            probe.set_axes(*original)
            _, d_orig = closest(probe)
            probe.set_axes(*final)
        else:
            d_orig = d_start
        if args.apply and d_end < d_orig:
            keep = True
            print(f"\nApplied: the node now has the improved values (closest approach {km(d_end)}).")
            print("Run duna_encounter_check.py to see the result. If the closest approach is still above "
                  "Duna's sphere of influence (47,922 km), run this again. Once there is an encounter, "
                  "use --pe-km to tune the periapsis, e.g.  python duna_lever_advisor.py --pe-km 30 --apply")
        elif args.apply:
            print("\nThe result is NOT better than your original node, so I did not apply it.")
        else:
            print("\nNode left exactly as it was. Add --apply to keep the improved values.")
    finally:
        if probe is not None and original is not None and not keep:
            try:
                probe.set_axes(*original)
            except Exception as exc:
                print(f"!! could not restore the node ({exc}); put it back by hand: {original}")
        conn.close()


if __name__ == "__main__":
    main()
