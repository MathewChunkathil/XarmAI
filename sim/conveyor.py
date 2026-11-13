"""Conveyor belt for the simulation (lane geometry + kinematic belt + visual bodies).  SIMULATION ONLY.

The belt is KINEMATIC: every sim step each unattached cube resting on the belt is translated by v*dt along the flow
(yaw kept, linear velocity set to v for physics consistency). A cube that reaches the end stop stops against it.
Visual-only cleats (cross stripes) are stepped with the belt so the motion is visible. All numbers come from
workspace.yaml `conveyor:` and are PLACEHOLDER (no real belt exists).

Lane coordinates: s = distance along the flow from the feed end (0) to the end stop (L); c = lateral offset from
the lane centre line (left of the flow positive). Only axis-aligned lanes (direction = +-x or +-y) are supported.
"""
import math

import numpy as np
import pybullet as pb


class ConveyorCfg:
    def __init__(self, d, table_z, edge):
        self.raw = d
        (x0, x1), (y0, y1) = d["lane_box"]["x_m"], d["lane_box"]["y_m"]
        self.box = {"x_m": [float(x0), float(x1)], "y_m": [float(y0), float(y1)]}
        dx, dy = (float(v) for v in d["direction"])
        n = math.hypot(dx, dy)
        dx, dy = dx / n, dy / n
        if min(abs(dx), abs(dy)) > 1e-9:
            raise ValueError("conveyor direction must be axis aligned (+-x or +-y)")
        self.dir = np.array([dx, dy])
        self.nrm = np.array([-dy, dx])               # left of the flow
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        self.centre = np.array([cx, cy])
        self.length = (x1 - x0) if abs(dx) > 0.5 else (y1 - y0)
        self.width = (y1 - y0) if abs(dx) > 0.5 else (x1 - x0)
        self.start = self.centre - self.dir * self.length / 2     # centre line point at s = 0
        self.table_z = float(table_z)
        self.edge = float(edge)
        self.height = float(d["belt_height_m"])
        self.top_z = self.table_z + self.height                   # belt top surface
        self.rest_z = self.top_z + self.edge / 2                  # centre height of a cube resting on the belt
        self.speed_default = float(d["speed_m_s"])
        self.speed_max = float(d["max_speed_m_s"])
        self.stop_t = float(d.get("end_stop_thickness_m", 0.012))
        self.pitch = float(d.get("cleat_pitch_m", 0.045))
        self.feed_s = float(d["feed_s_m"])
        self.feed_cross = float(d.get("feed_cross_m", 0.0))
        self.window = tuple(float(v) for v in d["pick_window_s_m"])
        self.park_q = [float(v) for v in d["park_joint_state_rad"]]
        self.track_frames = int(d.get("track_frames", 6))
        self.track_dt = float(d.get("track_dt_s", 0.10))
        self.stationary_v = float(d.get("stationary_speed_m_s", 0.003))
        self.hover_s = float(d.get("hover_s", 0.30))
        self.pregrasp_above = float(d.get("pregrasp_above_m", 0.07))
        self.descend_speed = float(d.get("track_descend_speed_m_s", 0.20))
        self.stop_margin = float(d.get("stop_margin_m", 0.004))

    # ---- lane coordinates
    def to_world(self, s, c=0.0):
        p = self.start + self.dir * s + self.nrm * c
        return float(p[0]), float(p[1])

    def from_world(self, x, y):
        d = np.array([x, y], float) - self.start
        return float(d @ self.dir), float(d @ self.nrm)

    def half_extent(self, yaw):
        """Half extent of the (square) cube footprint along the flow for a cube yaw (rad)."""
        a = yaw - math.atan2(self.dir[1], self.dir[0])
        return self.edge / 2 * (abs(math.cos(a)) + abs(math.sin(a)))

    def window_box(self):
        """World x/y box of the pick window (cube centres), full lane width."""
        s0, s1 = self.window
        pts = [self.to_world(s, c) for s in (s0, s1) for c in (-self.width / 2, self.width / 2)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return {"x_m": [min(xs), max(xs)], "y_m": [min(ys), max(ys)]}

    def corners3d(self, margin=0.0, z_top=None):
        """Lane corners (belt top) and the same corners raised to a cube top, for camera-visibility checks."""
        z0 = self.top_z
        z1 = self.top_z + self.edge if z_top is None else z_top
        out = []
        for s in (-margin, self.length + margin):
            for c in (-self.width / 2 - margin, self.width / 2 + margin):
                x, y = self.to_world(s, c)
                out += [(x, y, z0), (x, y, z1)]
        return out


class Conveyor:
    """Belt state + bodies in ONE pybullet client (the visible world and the hidden aux world each own one)."""

    def __init__(self, cfg: ConveyorCfg):
        self.cfg = cfg
        self.running = False
        self.speed = cfg.speed_default          # commanded belt speed [m/s] (applies while running)
        self.phase = 0.0                         # belt travel [m] (cleat offset)
        self.bodies = {}
        self.cleats = []
        self.p = None
        self._exp = {}                           # body -> xy this belt put it at last step (removes the physics step's own drift)

    @property
    def v(self):
        """Actual belt speed now [m/s] (0 when stopped). GROUND TRUTH: scoring / UI only."""
        return self.speed if self.running else 0.0

    def set_speed(self, v):
        self.speed = float(np.clip(v, 0.0, self.cfg.speed_max))

    # ---- bodies (all visual-only except the belt top and the end stop)
    def build(self, p):
        self.p = p
        c = self.cfg
        alu = [0.72, 0.74, 0.78, 1]
        belt_rgba = [0.10, 0.10, 0.11, 1]
        ang = math.atan2(c.dir[1], c.dir[0])
        q = p.getQuaternionFromEuler([0, 0, ang])          # lane frame: x along the flow, y left

        def box(s, cc, half, z, rgba, collide, name):
            """box centred at lane coords (s, cc), half extents (along, across, height), centre height z."""
            x, y = c.to_world(s, cc)
            vs = p.createVisualShape(pb.GEOM_BOX, halfExtents=list(half), rgbaColor=rgba)
            cs = p.createCollisionShape(pb.GEOM_BOX, halfExtents=list(half)) if collide else -1
            b = p.createMultiBody(0, cs, vs, [x, y, z], q)
            self.bodies[name] = b
            return b
        h = c.height
        b = box(c.length / 2, 0.0, (c.length / 2, c.width / 2, h / 2), c.table_z + h / 2, belt_rgba, True, "belt")
        p.changeDynamics(b, -1, lateralFriction=0.8)
        for sgn, nm in ((1, "rail_l"), (-1, "rail_r")):
            box(c.length / 2, sgn * (c.width / 2 + 0.005), (c.length / 2 + 0.01, 0.005, h / 2 + 0.006),
                c.table_z + h / 2 + 0.006, alu, False, nm)
        # rollers at both ends (visual): cylinder with its axis across the belt
        rq = p.getQuaternionFromEuler([math.pi / 2, 0, ang])
        for s, nm in ((0.0, "roller_a"), (c.length, "roller_b")):
            x, y = c.to_world(s, 0.0)
            vs = p.createVisualShape(pb.GEOM_CYLINDER, radius=h / 2 + 0.001, length=c.width + 0.004, rgbaColor=[0.55, 0.56, 0.6, 1])
            self.bodies[nm] = p.createMultiBody(0, -1, vs, [x, y, c.table_z + h / 2], rq)
        # end stop (collision): block behind s = L
        box(c.length + c.stop_t / 2, 0.0, (c.stop_t / 2, c.width / 2 + 0.012, 0.022), c.table_z + 0.022,
            [0.86, 0.36, 0.12, 1], True, "end_stop")
        # cleats: thin lighter stripes across the belt, repositioned every step (visual only)
        n = max(1, int(c.length / c.pitch))
        self.cleats = []
        for k in range(n):
            vs = p.createVisualShape(pb.GEOM_BOX, halfExtents=[0.004, c.width / 2 - 0.004, 0.0006], rgbaColor=[0.30, 0.31, 0.34, 1])
            self.cleats.append(p.createMultiBody(0, -1, vs, [0, 0, -1]))
        self._q = q
        self.set_phase(self.phase)

    def add_window_overlay(self):
        """Visible-world only (app): translucent yellow strip + end lines marking the pick window on the belt. No collision.
        -> list of bodies (the caller keeps them; the hidden camera world never gets this overlay)."""
        p, c = self.p, self.cfg
        s0, s1 = c.window
        ang = math.atan2(c.dir[1], c.dir[0])
        q = p.getQuaternionFromEuler([0, 0, ang])
        out = []
        for s, half_s, a, zc in (((s0 + s1) / 2, (s1 - s0) / 2, 0.22, c.top_z + 0.0012), (s0, 0.0015, 0.95, c.top_z + 0.0016),
                                 (s1, 0.0015, 0.95, c.top_z + 0.0016)):
            x, y = c.to_world(s, 0.0)
            vs = p.createVisualShape(pb.GEOM_BOX, halfExtents=[half_s, c.width / 2 - 0.002, 0.0004], rgbaColor=[1.0, 0.85, 0.15, a])
            out.append(p.createMultiBody(0, -1, vs, [x, y, zc], q))
        return out

    def set_phase(self, phase):
        """Reposition the cleats for belt travel `phase` [m] (wraps over the lane length)."""
        c = self.cfg
        self.phase = float(phase)
        L = c.length - 0.02
        for k, b in enumerate(self.cleats):
            s = 0.01 + (k * c.pitch + self.phase) % L
            x, y = c.to_world(s, 0.0)
            self.p.resetBasePositionAndOrientation(b, [x, y, c.top_z + 0.0006], self._q)

    # ---- per sim step
    def step(self, world, dt):
        """Called from SimWorld._tick before stepSimulation: advance the belt one step and carry the cubes on it."""
        if not self.running or self.speed <= 0.0:
            return
        c = self.cfg
        ds = self.speed * dt
        self.set_phase(self.phase + ds)
        p = world.p
        bodies = set(world.cubes.values())
        if world.cube is not None:
            bodies.add(world.cube)
        for body in bodies:
            if world.cube_attached is not None and body == world.cube:
                continue
            pos, orn = p.getBasePositionAndOrientation(body)
            exp = self._exp.get(body)
            if exp is not None and math.hypot(pos[0] - exp[0], pos[1] - exp[1]) < 0.004:
                pos = (exp[0], exp[1], pos[2])      # the physics step moved it a little (velocity integration): ignore that
            s, cc = c.from_world(pos[0], pos[1])
            if abs(cc) > c.width / 2 or s < -0.02 or s > c.length + 0.02:
                self._exp.pop(body, None)
                continue
            if not (c.rest_z - 0.004 <= pos[2] <= c.rest_z + 0.006):        # resting on the belt (not lifted / not on the table)
                continue
            yaw = p.getEulerFromQuaternion(orn)[2]
            s_max = c.length - c.half_extent(yaw) - 0.0005                   # end stop
            s_new = min(s + ds, max(s, s_max))
            moved = s_new - s
            x, y = c.to_world(s_new, cc)
            if moved > 0.0:
                p.resetBasePositionAndOrientation(body, [x, y, pos[2]], orn)
                p.resetBaseVelocity(body, [float(c.dir[0] * moved / dt), float(c.dir[1] * moved / dt), 0.0], [0, 0, 0])
                self._exp[body] = (x, y)
            else:
                p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
                self._exp.pop(body, None)

    def remove(self):
        if self.p is None:
            return
        for b in list(self.bodies.values()) + self.cleats:
            try:
                self.p.removeBody(b)
            except pb.error:
                pass
        self.bodies, self.cleats = {}, []
