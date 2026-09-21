from raicom_core.types import ServoCommand


class ServoConfig(object):
    def __init__(
        self,
        aim_nx,
        aim_ny,
        x_sign=1.0,
        y_sign=1.0,
        kp_x=0.35,
        kp_y=0.40,
        coarse_max_v=0.07,
        fine_max_v=0.025,
        coarse_tol=0.08,
        fine_tol_n=0.045,
        fine_tol_f=0.055,
        need_stable=6,
        timeout_s=14.0,
        max_travel_m=0.40,
        stale_s=0.6,
    ):
        self.aim_nx = float(aim_nx)
        self.aim_ny = float(aim_ny)
        self.x_sign = float(x_sign)
        self.y_sign = float(y_sign)
        self.kp_x = float(kp_x)
        self.kp_y = float(kp_y)
        self.coarse_max_v = float(coarse_max_v)
        self.fine_max_v = float(fine_max_v)
        self.coarse_tol = float(coarse_tol)
        self.fine_tol_n = float(fine_tol_n)
        self.fine_tol_f = float(fine_tol_f)
        self.need_stable = int(need_stable)
        self.timeout_s = float(timeout_s)
        self.max_travel_m = float(max_travel_m)
        self.stale_s = float(stale_s)


class VisualServo(object):
    def __init__(self, config):
        self.cfg = config
        self.aligned = False
        self.stable = 0
        self.t0 = None
        self.travel = 0.0
        self.coarse = True

    def command(self, nx, ny, stamp, now, dt):
        if self.t0 is None:
            self.t0 = now
        if nx is None or ny is None:
            self.stable = 0
            self.aligned = False
            return ServoCommand(0.0, 0.0, "lost")
        if now - float(stamp) > self.cfg.stale_s:
            self.stable = 0
            self.aligned = False
            return ServoCommand(0.0, 0.0, "stale")
        if now - self.t0 > self.cfg.timeout_s:
            return ServoCommand(0.0, 0.0, "timeout")
        if self.travel > self.cfg.max_travel_m:
            return ServoCommand(0.0, 0.0, "travel_limit")

        ex = float(nx) - self.cfg.aim_nx
        ey = float(ny) - self.cfg.aim_ny
        err = max(abs(ex), abs(ey))
        if err < self.cfg.coarse_tol:
            self.coarse = False
        max_v = self.cfg.coarse_max_v if self.coarse else self.cfg.fine_max_v
        vx = self.cfg.x_sign * (-self.cfg.kp_x) * ey
        vy = self.cfg.y_sign * (-self.cfg.kp_y) * ex
        vx = max(-max_v, min(max_v, vx))
        vy = max(-max_v, min(max_v, vy))
        if abs(ex) < self.cfg.fine_tol_n:
            vy = 0.0
        if abs(ey) < self.cfg.fine_tol_f:
            vx = 0.0
        if vx == 0.0 and vy == 0.0:
            self.stable += 1
        else:
            self.stable = 0
            self.aligned = False
        self.travel += (abs(vx) + abs(vy)) * max(0.0, float(dt))
        if self.stable >= self.cfg.need_stable:
            self.aligned = True
            return ServoCommand(0.0, 0.0, "aligned")
        return ServoCommand(vx, vy, "tracking")
