"""Quadratic-programming MPC with online identification and discrete action allocation.

Extracted for standalone lunar-qp on 2026-09-08; evaluator imports changed.

The convex relaxation is solved by OSQP. Applied actions remain discrete; its
relaxed plans are not integer-optimal or certified safe.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import osqp
from scipy import sparse

DT = 1 / 50
OBS_SCALE = np.array([10, 20 / 3, 5, 7.5, 1, 2.5], dtype=float)

def state_from_obs(obs):
    obs = np.asarray(obs, dtype=float)
    if obs.shape != (8,) or not np.isfinite(obs).all():
        raise ValueError("Expected eight finite LunarLander observation values")
    return obs[:6] * OBS_SCALE

class Dynamics:
    """Planar acceleration model, updated by bounded recursive least squares."""
    def __init__(self, forgetting=0.99):
        # [main acceleration, side acceleration, horizontal bias, vertical bias]
        self.linear = np.array([18., 2.5, 0., 0.])
        # [side angular acceleration, angular bias]
        self.angular = np.array([5., 0.])
        self.p = np.diag([4., 1., 1., 1.])
        self.pa = np.diag([1., .5])
        self.forgetting = forgetting
        self.updates = 0
        self.last_error = 0.

    def predict(self, x, action, dt=DT):
        x = np.asarray(x)
        action = np.asarray(action)
        main = action == 2
        side = np.where(action == 1, -1., np.where(action == 3, 1., 0.))
        s, c = np.sin(x[..., 4]), np.cos(x[..., 4])
        mg, sg, bx, by = self.linear
        ax = -mg * main * s + sg * side * c + bx
        ay = mg * main * c + sg * side * s - 10 + by
        aw = -self.angular[0] * side + self.angular[1]
        out = x.copy()
        out[..., 2] += dt * ax
        out[..., 3] += dt * ay
        out[..., 5] += dt * aw
        out[..., 0] += dt * out[..., 2]
        out[..., 1] += dt * out[..., 3]
        out[..., 4] += dt * out[..., 5]
        return out

    def _fit(self, theta, covariance, phi, target):
        projected = covariance @ phi
        gain = projected / (self.forgetting + phi @ projected)
        theta += gain * (target - phi @ theta)
        covariance[:] = (covariance - np.outer(gain, projected)) / self.forgetting
        covariance[:] = (covariance + covariance.T) / 2

    def observe(self, before, action, after, learn=True):
        x, nxt = state_from_obs(before), state_from_obs(after)
        prediction = self.predict(x, action)
        self.last_error = float(np.linalg.norm(nxt[[2, 3, 5]] - prediction[[2, 3, 5]]))
        if not learn:
            return
        # Contact impulses are not evidence of changed airborne engine strength.
        if np.any(np.asarray(before)[6:]) or np.any(np.asarray(after)[6:]):
            return
        acceleration = (nxt[[2, 3, 5]] - x[[2, 3, 5]]) / DT
        if np.max(np.abs(acceleration)) > 60:
            return
        main, side = float(action == 2), float(action == 3) - float(action == 1)
        s, c = np.sin(x[4]), np.cos(x[4])
        self._fit(self.linear, self.p, np.array([-main*s, side*c, 1., 0.]), acceleration[0])
        self._fit(self.linear, self.p, np.array([main*c, side*s, 0., 1.]), acceleration[1]+10)
        self._fit(self.angular, self.pa, np.array([-side, 1.]), acceleration[2])
        self.linear[:] = np.clip(self.linear, [5., .3, -4., -4.], [35., 6., 4., 4.])
        self.angular[:] = np.clip(self.angular, [1., -2.], [10., 2.])
        self.updates += 1

@dataclass
class QPConfig:
    horizon: int = 16
    hold: int = 4
    adaptive: bool = True
    forgetting: float = .99
    max_iter: int = 4000

    def __post_init__(self):
        if any(type(v) is not int or v < 1 for v in (self.horizon, self.hold, self.max_iter)):
            raise ValueError('horizon, hold and max_iter must be positive integers')
        if not 0 < self.forgetting <= 1:
            raise ValueError('forgetting must be in (0, 1]')


class QuadraticMPC:
    """Convex QP over relaxed engine duties, followed by discrete pulse allocation."""
    def __init__(self, config=None):
        self.cfg = config or QPConfig()
        self.model = Dynamics(self.cfg.forgetting)
        self.credit = np.zeros(4)
        self.warm = None
        self.last_plan, self.last_prediction, self.last_cost = [], [], 0.
        self.last_qp = {}

    def observe(self, before, action, after):
        self.model.observe(before, action, after, learn=self.cfg.adaptive)

    def linearize(self, state):
        """Affine semi-implicit model about current tilt and nominal hover duty."""
        dt = DT*self.cfg.hold
        mg, sg, bx, by = self.model.linear
        tg, bw = self.model.angular
        angle = state[4]
        sine, cosine = np.sin(angle), np.cos(angle)
        hover = np.clip((10-by)/max(mg*cosine, .1), 0, 1)
        # Acceleration = J*x + G*u + bias; u order is main, left, right.
        jac = np.zeros((3, 6))
        jac[:, 4] = [-mg*hover*cosine, -mg*hover*sine, 0]
        engine = np.array([[-mg*sine, -sg*cosine, sg*cosine],
                           [mg*cosine, -sg*sine, sg*sine], [0, tg, -tg]])
        bias = np.array([bx, by-10, bw]) - jac @ state
        a, b, c = np.eye(6), np.zeros((6, 3)), np.zeros(6)
        for position, velocity, axis in ((0, 2, 0), (1, 3, 1), (4, 5, 2)):
            a[velocity] += dt*jac[axis]
            b[velocity], c[velocity] = dt*engine[axis], dt*bias[axis]
            a[position] += dt*a[velocity]
            b[position], c[position] = dt*b[velocity], dt*c[velocity]
        return a, b, c

    def problem(self, state):
        """Return P,q,A,l,u for z=[engine duties, approach slacks]."""
        n = self.cfg.horizon
        nx, nu, ns = 6*(n+1), 3*n, 3*n
        a, b, c = self.linearize(state)
        # Eliminate equality-constrained states: X = base + response @ U.
        base, response = np.zeros((n+1, 6)), np.zeros((n+1, 6, nu))
        base[0] = state
        for k in range(n):
            base[k+1] = a @ base[k] + c
            response[k+1] = a @ response[k]
            response[k+1, :, 3*k:3*k+3] += b
        self.base, self.response = base.ravel(), response.reshape(nx, nu)
        # A fixed reference within this solve keeps the objective quadratic.
        target = np.zeros((n+1, 6))
        clearance = state[1] - .5*max(0., abs(state[0])-1.)
        descent = np.clip(-.8*clearance, -2.5, .5)
        if abs(state[0]) < 1 and state[1] < 1:
            descent = -.3
        target[:, 1] = np.maximum(-.08, state[1] + np.arange(n+1)*DT*self.cfg.hold*descent)
        target[:, 3] = np.maximum(descent, -.8*np.maximum(.05, target[:, 1]))
        target[:, 3] = np.minimum(target[:, 3], -.2)
        qstate = np.tile([.3, .2, .5, 5., 25., 2.], n+1)
        qstate[-6:] *= 5
        hessian = self.response.T @ (qstate[:, None]*self.response) + .03*np.eye(nu)
        p = sparse.block_diag([2*hessian, sparse.diags(2*np.tile([1000., 200., 1000.], n))], format='csc')
        q = np.r_[2*self.response.T @ (qstate*(self.base-target.ravel())), np.zeros(ns)]
        # Nonnegative duties, at most one full engine per prediction block.
        bounds = sparse.eye(nu+ns)
        simplex = sparse.hstack([sparse.kron(sparse.eye(n), np.ones((1, 3))), sparse.csc_matrix((n, ns))])
        # Soft clearance corridor, tilt and viewport margins; no safety certificate.
        envelope = np.array([[.5, -1, 0, 0, 0, 0], [-.5, -1, 0, 0, 0, 0],
                             [0, -1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, -1, 0], [1, 0, 0, 0, 0, 0], [-1, 0, 0, 0, 0, 0]])
        slack = np.zeros((7, 3))
        slack[np.arange(7), [0, 0, 0, 1, 1, 2, 2]] = -1
        envelope = sparse.kron(sparse.eye(n), envelope)
        limits = sparse.hstack([envelope @ self.response[6:], sparse.kron(sparse.eye(n), slack)])
        matrix = sparse.vstack([bounds, simplex, limits], format='csc')
        lower = np.r_[np.zeros(nu+ns), np.zeros(n), np.full(7*n, -np.inf)]
        upper = np.r_[np.ones(nu), np.full(ns, np.inf), np.ones(n),
                       np.tile([.55, .55, .05, .65, .65, 8.5, 8.5], n)-envelope @ self.base[6:]]
        return p, q, matrix, lower, upper

    def allocate(self, duties):
        """Error diffusion converts relaxed fractions to exactly one legal action."""
        main, left, right = np.clip(duties, 0, 1)
        fractions = np.array([max(0., 1-main-left-right), left, main, right])
        fractions /= fractions.sum()
        self.last_qp['probabilities'] = fractions.tolist()
        self.credit += fractions
        action = int(np.argmax(self.credit))
        self.credit[action] -= 1
        return action

    def act(self, obs):
        state = state_from_obs(obs)
        self.last_plan, self.last_prediction, self.last_cost = [], [state.tolist()], 0.
        self.last_qp = {'status': 'contact', 'fallback': False, 'iterations': 0,
                        'constraint_violation': 0., 'slack_max': 0., 'duties': [0., 0., 0.]}
        if np.any(np.asarray(obs)[6:]):
            self.credit[:] = 0
            self.warm = None
            return 0
        p, q, matrix, lower, upper = self.problem(state)
        solver = osqp.OSQP()
        solver.setup(P=p, q=q, A=matrix, l=lower, u=upper, verbose=False,
                     eps_abs=1e-4, eps_rel=1e-4, max_iter=self.cfg.max_iter,
                     polishing=False, warm_starting=True, adaptive_rho=False, rho=.1)
        if self.warm is not None:
            solver.warm_start(x=self.warm)
        result = solver.solve(raise_error=False)
        self.last_qp.update(status=result.info.status, iterations=int(result.info.iter))
        violation = None
        if result.info.status_val in (1, 2) and result.x is not None and np.isfinite(result.x).all():
            residual = matrix @ result.x
            violation = float(max(0., np.max(lower-residual), np.max(residual-upper)))
            if not np.isfinite(violation):
                violation = None
        self.last_qp['constraint_violation'] = violation
        if violation is None or violation > .005:
            # Explicit coast-on-failure policy, not a validated recovery controller.
            self.last_qp['fallback'] = True
            self.warm = None
            self.credit[:] = 0
            return 0
        self.warm = result.x.copy()
        end = 3*self.cfg.horizon
        duties = result.x[:end].reshape(-1, 3)
        self.last_prediction = (self.base+self.response @ result.x[:end]).reshape(-1, 6).tolist()
        self.last_plan = duties.tolist()
        self.last_cost = float(result.info.obj_val)
        self.last_qp.update(duties=duties[0].tolist(), slack_max=float(np.max(result.x[end:])))
        return self.allocate(duties[0])


def landing_result(env, terminated, truncated, final_obs):
    """Evaluator-only simulator truth; accumulated reward is not a landing test."""
    u = env.unwrapped
    feet = [u.lander.position.x + sign*2/3*np.cos(u.lander.angle) + .6*np.sin(u.lander.angle)
            for sign in (-1, 1)]
    margin = min(min(feet)-u.helipad_x1, u.helipad_x2-max(feet))
    landed = bool(terminated and not truncated and not u.game_over and not u.lander.awake
                  and all(leg.ground_contact for leg in u.legs) and abs(final_obs[0]) < 1)
    return {"landed": landed, "feet_inside": bool(margin >= 0),
            "pad_margin": float(margin), "passed": bool(landed and margin >= 0),
            "terminated": bool(terminated), "truncated": bool(truncated),
            "rendered_foot_x": [float(x) for x in feet]}

def run_episode(seed, cfg, thrust_scale=1., change_step=100):
    import gymnasium as gym
    from gymnasium.envs.box2d import lunar_lander as LL
    from lunar_qp.environment import StartPose
    if not 0 < thrust_scale <= 1 or change_step < 0:
        raise ValueError("thrust_scale must be in (0, 1] and change_step nonnegative")
    env = StartPose(gym.make("LunarLander-v3"), 6, .5)
    controller = QuadraticMPC(cfg)
    obs, info = env.reset(seed=seed)
    total, records, first_contact = 0., [], None
    # This evaluator runs one environment at a time. The temporary, process-local
    # engine constant changes simulator physics, never the controller's model.
    try:
        term = trunc = False
        while not (term or trunc):
            tick = len(records)
            begin = time.perf_counter()
            action = controller.act(obs)
            elapsed = time.perf_counter() - begin
            model_before = controller.model.linear.tolist()
            pre_velocity = list(env.unwrapped.lander.linearVelocity)
            pre_tilt = float(env.unwrapped.lander.angle)
            original_power = LL.MAIN_ENGINE_POWER
            try:
                if tick >= change_step:
                    LL.MAIN_ENGINE_POWER = original_power * thrust_scale
                nxt, reward, term, trunc, _ = env.step(action)
            finally:
                LL.MAIN_ENGINE_POWER = original_power
            controller.observe(obs, action, nxt)
            if first_contact is None and np.any(nxt[6:]):
                first_contact = {"step": tick, "pre_contact_velocity": pre_velocity,
                                 "pre_contact_tilt": pre_tilt}
            records.append({"state": obs.tolist(), "action": action, "reward": float(reward),
                            "next_state": nxt.tolist(), "solve_ms": elapsed*1000,
                            "plan": controller.last_plan, "prediction": controller.last_prediction,
                            "plan_cost": controller.last_cost, "model_before": model_before,
                            "model": controller.model.linear.tolist(),
                            "angular_model": controller.model.angular.tolist(),
                            "prediction_error": controller.model.last_error, "qp": controller.last_qp.copy()})
            total += reward
            obs = nxt
        from lunar_qp.environment import ground_line
        u = env.unwrapped
        return {"seed": seed, "return": total, "steps": len(records),
                **landing_result(env, term, trunc, obs), "first_contact": first_contact,
                "final_state": obs.tolist(), "records": records, "start": info["start"],
                "terrain": ground_line(env),
                "helipad": [u.helipad_x1, u.helipad_x2, u.helipad_y]}
    finally:
        env.close()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--fixed-model", action="store_true")
    p.add_argument("--horizon", type=int, default=16)
    p.add_argument("--hold", type=int, default=4)
    p.add_argument("--max-iter", type=int, default=4000)
    p.add_argument("--thrust-scale", type=float, default=1.)
    p.add_argument("--change-step", type=int, default=100)
    p.add_argument("--out", type=Path, default=Path("dist/qp/evaluation.json"))
    p.add_argument("--replay", type=Path, help="write a self-contained interactive HTML replay")
    args = p.parse_args()
    if args.episodes < 1:
        p.error("episodes must be positive")
    if not 0 < args.thrust_scale <= 1 or args.change_step < 0:
        p.error("thrust-scale must be in (0, 1] and change-step nonnegative")
    cfg = QPConfig(args.horizon, args.hold, not args.fixed_model, max_iter=args.max_iter)
    episodes = []
    for seed in range(args.seed, args.seed+args.episodes):
        ep = run_episode(seed, cfg, args.thrust_scale, args.change_step)
        episodes.append(ep)
        print(f"seed={seed} return={ep['return']:.1f} steps={ep['steps']} "
              f"landed={ep['landed']} pad_margin={ep['pad_margin']:.3f}", flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    latencies = [r["solve_ms"] for ep in episodes for r in ep["records"]]
    qp_records = [r['qp'] for ep in episodes for r in ep['records']]
    data = {"method": "convex QP with discrete pulse allocation",
            "config": vars(cfg), "thrust_scale": args.thrust_scale,
            "change_step": args.change_step, "episodes": episodes,
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "summary": {"episodes": len(episodes), "passed": sum(e["passed"] for e in episodes),
                        "landings": sum(e["landed"] for e in episodes),
                        "mean_return": float(np.mean([e["return"] for e in episodes])),
                        "worst_return": min(e["return"] for e in episodes),
                        "minimum_pad_margin": min(e["pad_margin"] for e in episodes),
                        "solve_ms_p50": float(np.percentile(latencies, 50)),
                        "solve_ms_p95": float(np.percentile(latencies, 95)),
                        "solve_ms_p99": float(np.percentile(latencies, 99)),
                        "solve_ms_max": max(latencies),
                        "planning_over_20ms": sum(ms > 20 for ms in latencies),
                        "fallback_steps": sum(r['fallback'] for r in qp_records),
                        "solved_inaccurate_steps": sum(r['status'] == 'solved inaccurate' for r in qp_records),
                        "maximum_slack": max(r['slack_max'] for r in qp_records)}}
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False)
    args.out.write_text(payload)
    if args.replay:
        template = Path(__file__).with_name("qp_replay.html").read_text()
        args.replay.parent.mkdir(parents=True, exist_ok=True)
        args.replay.write_text(template.replace("__MPC_DATA__", payload.replace("<", "\\u003c")))
        print(f"Replay: {args.replay} (open the file directly)", flush=True)
    print(f"Saved {args.out}; passed {sum(e['passed'] for e in episodes)}/{len(episodes)}", flush=True)

if __name__ == "__main__":
    main()
