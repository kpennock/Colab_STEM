#!/usr/bin/env python3
"""restore_node.py -- put the ship's LAST maneuver node back to exact values.

    python restore_node.py                 # sets prograde 5.0, normal 49.5, radial 229.9
    python restore_node.py 0 0 0           # or any values: prograde normal radial
"""
import sys

import krpc

values = [float(x) for x in sys.argv[1:4]] if len(sys.argv) >= 4 else [5.0, 49.5, 229.9]

conn = krpc.connect(name="restore node")
try:
    nodes = conn.space_center.active_vessel.control.nodes
    if not nodes:
        sys.exit("No maneuver node on the ship.")
    node = nodes[-1]
    node.prograde, node.normal, node.radial = values
    print(f"Node set to prograde {node.prograde:+.1f}, normal {node.normal:+.1f}, radial {node.radial:+.1f} m/s")
finally:
    conn.close()
