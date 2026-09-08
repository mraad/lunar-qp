"""QP algebra, actuator relaxation, estimator and solver failure regressions."""
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import patch

import numpy as np

from lunar_qp.qp import QuadraticMPC, QPConfig, OBS_SCALE, DT


def observation(state):
    return np.r_[np.asarray(state)/OBS_SCALE, 0., 0.]


class QPTest(TestCase):
    def test_linearization_matches_model_at_hover_and_its_derivative(self):
        controller = QuadraticMPC()
        state = np.array([2., 6., .4, -.8, .2, -.1])
        a, b, c = controller.linearize(state)
        hover = 10/(18*np.cos(state[4]))
        duties = np.array([hover, 0., 0.])
        def exact(x):
            dt = DT*controller.cfg.hold
            return (1-hover)*controller.model.predict(x, 0, dt) + hover*controller.model.predict(x, 2, dt)
        np.testing.assert_allclose(a @ state + b @ duties + c, exact(state), atol=1e-12)
        for i in range(6):
            delta = np.eye(6)[i]*1e-6
            derivative = (exact(state+delta)-exact(state-delta))/2e-6
            np.testing.assert_allclose(a[:, i], derivative, atol=1e-8)

    def test_qp_is_convex_and_solution_respects_linear_constraints(self):
        controller = QuadraticMPC(QPConfig(horizon=8))
        obs = observation([1., 5., .2, -.5, .1, 0])
        p, q, a, lower, upper = controller.problem(obs[:6]*OBS_SCALE)
        self.assertGreater(np.linalg.eigvalsh(p.toarray()).min(), 0)
        self.assertTrue(np.all(lower <= upper))
        self.assertIn(controller.act(obs), range(4))
        self.assertFalse(controller.last_qp['fallback'])
        z = controller.warm
        residual = a @ z
        self.assertLessEqual(max(np.max(lower-residual), np.max(residual-upper)), .005)
        self.assertEqual(z.size, len(q))
        self.assertEqual(len(controller.last_prediction), 9)
        self.assertEqual(len(controller.last_plan), 8)
        transition, engine, bias = controller.linearize(obs[:6]*OBS_SCALE)
        predicted = obs[:6]*OBS_SCALE
        for duty, recorded in zip(controller.last_plan, controller.last_prediction[1:]):
            predicted = transition @ predicted + engine @ duty + bias
            np.testing.assert_allclose(predicted, recorded, atol=1e-10)

    def test_online_model_tracks_unannounced_main_engine_loss(self):
        controller = QuadraticMPC()
        truth = QuadraticMPC().model
        for tick in range(300):
            if tick == 150:
                truth.linear[0] = 12.6
            state = np.array([0., 5., .1, -.2, .3*np.sin(tick), .05])
            action = tick % 4
            after = truth.predict(state, action)
            controller.observe(observation(state), action, observation(after))
        self.assertLess(abs(controller.model.linear[0]-12.6), .5)

    def test_pulse_allocation_preserves_fractional_requests(self):
        controller = QuadraticMPC()
        actions = [controller.allocate([.4, .1, .2]) for _ in range(100)]
        np.testing.assert_array_equal(np.bincount(actions, minlength=4), [30, 10, 40, 20])
        self.assertLess(np.max(np.abs(controller.credit)), 1e-12)
        self.assertIn(controller.allocate([-.0001, 1.0001, 0]), range(4))

    def test_bad_solver_results_use_explicit_coast_fallback(self):
        obs = observation([2., 4., 0, -1, .2, 0])
        for status, vector in ((7, None), (1, np.full(6*8, np.nan)), (1, np.full(6*8, 1e6))):
            controller = QuadraticMPC(QPConfig(horizon=8))
            result = SimpleNamespace(x=vector, info=SimpleNamespace(status='mock failure', status_val=status, iter=1))
            with patch('lunar_qp.qp.osqp.OSQP') as solver:
                solver.return_value.solve.return_value = result
                self.assertEqual(controller.act(obs), 0)
            self.assertTrue(controller.last_qp['fallback'])
            self.assertIsNone(controller.warm)

    def test_contact_and_fixed_model_do_not_learn(self):
        controller = QuadraticMPC(QPConfig(adaptive=False))
        before = observation([0., 4., 0, -1, 0, 0])
        after = before.copy(); after[3] += .1
        controller.observe(before, 2, after)
        np.testing.assert_array_equal(controller.model.linear, [18., 2.5, 0, 0])
        after[6] = 1
        self.assertEqual(controller.act(after), 0)
        self.assertEqual(controller.last_qp['status'], 'contact')

    def test_configuration_and_observation_validation(self):
        for args in ({'horizon': 0}, {'hold': 1.5}, {'max_iter': -1}, {'forgetting': 0}):
            with self.assertRaises(ValueError):
                QPConfig(**args)
        with self.assertRaises(ValueError):
            QuadraticMPC().act(np.full(8, np.nan))


if __name__ == '__main__':
    main()
