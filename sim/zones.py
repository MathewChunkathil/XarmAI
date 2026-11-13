"""Place zones + slots (pure Python, no PyBullet).  SIMULATION ONLY.

Zones come from workspace.yaml `place_zones:`. Zone 1 is the legacy place_zone / place_target (old keys keep working:
if `place_zones` is missing the list is built from them). All geometry is PLACEHOLDER (unmeasured).
"""
import math

DEFAULT_COLORS = {1: (250, 140, 40), 2: (90, 200, 240), 3: (190, 120, 250)}


def place_zones(ws):
    """-> list of dict(number, name, x_m, y_m, target=dict(x_m,y_m,yaw_rad), color_rgb), sorted by number."""
    zs = []
    for z in ws.get("place_zones") or []:
        n = int(z["number"])
        zs.append({"number": n, "name": z.get("name", "zone %d" % n), "x_m": list(z["x_m"]), "y_m": list(z["y_m"]),
                   "target": dict(z["target"]), "color_rgb": tuple(z.get("color_rgb", DEFAULT_COLORS.get(n, (200, 200, 200))))})
    if not zs:       # legacy keys
        z = ws["place_zone"]
        zs.append({"number": 1, "name": "zone 1", "x_m": list(z["x_m"]), "y_m": list(z["y_m"]),
                   "target": dict(ws["place_target"]), "color_rgb": DEFAULT_COLORS[1]})
    return sorted(zs, key=lambda z: z["number"])


def get_zone(ws, number):
    for z in place_zones(ws):
        if z["number"] == int(number):
            return z
    raise KeyError("zone %s" % number)


def zone_slots(zone, spacing, edge, margin=0.002):
    """Candidate cube-centre slots inside the zone box, target first, then grid points (spacing apart) by
    distance from the target. A slot keeps the whole cube (half edge + margin) inside the zone box."""
    h = edge / 2 + margin
    x0, x1 = zone["x_m"][0] + h, zone["x_m"][1] - h
    y0, y1 = zone["y_m"][0] + h, zone["y_m"][1] - h
    tx, ty = zone["target"]["x_m"], zone["target"]["y_m"]
    slots = [(tx, ty)]
    pts = []
    nx, ny = int(math.floor((x1 - x0) / spacing)) + 1, int(math.floor((y1 - y0) / spacing)) + 1
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            if i == 0 and j == 0:
                continue
            x, y = tx + i * spacing, ty + j * spacing
            if x0 - 1e-9 <= x <= x1 + 1e-9 and y0 - 1e-9 <= y <= y1 + 1e-9:
                pts.append((math.hypot(x - tx, y - ty), abs(j), x, y))
    slots += [(x, y) for _, _, x, y in sorted(pts)]
    return slots


def zone_of_point(ws, x, y, tol=0.0):
    """Zone number containing (x, y), or None."""
    for z in place_zones(ws):
        if z["x_m"][0] - tol <= x <= z["x_m"][1] + tol and z["y_m"][0] - tol <= y <= z["y_m"][1] + tol:
            return z["number"]
    return None


def boxes_overlap(a, b):
    return not (a["x_m"][1] <= b["x_m"][0] or b["x_m"][1] <= a["x_m"][0] or
                a["y_m"][1] <= b["y_m"][0] or b["y_m"][1] <= a["y_m"][0])


def min_cube_spacing(pick_box, k, rng, spacing, margin, max_tries=2000):
    """Random non-overlapping centres inside pick_box (shrunk by margin). -> list of k (x, y) or None."""
    pts = []
    tries = 0
    while len(pts) < k and tries < max_tries:
        tries += 1
        x = rng.uniform(pick_box["x_m"][0] + margin, pick_box["x_m"][1] - margin)
        y = rng.uniform(pick_box["y_m"][0] + margin, pick_box["y_m"][1] - margin)
        if all(math.hypot(x - px, y - py) >= spacing for px, py in pts):
            pts.append((x, y))
    return pts if len(pts) == k else None
