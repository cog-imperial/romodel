import GPy
import rogp
import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples
import romodel as ro
import numpy as np
from pyomo.repn import generate_standard_repn


def generate_data(N, noise):
    np.random.seed(5)
    x = np.random.uniform(size=(N, 1))*6
    y = np.exp(-x/2) + np.random.normal(scale=noise*4*np.exp(-x/2),
                                        size=(N, 1))
    return x, y


def train_warped_gp(N, noise, warping_terms=3):
    x, y = generate_data(N, noise)
    norm = rogp.util.Normalizer()
    norm.scale_by(x, y)
    X, Y = norm.normalize(x, y)
    kernel = GPy.kern.RBF(input_dim=1, variance=1., lengthscale=1.)
    gp = GPy.models.WarpedGP(X, Y, kernel=kernel,
                             warping_terms=warping_terms)
    gp.optimize(messages=True)
    gp.noise = noise
    gp.dist = 'nonuniform'
    return gp, norm


class TestWarpedGP(unittest.TestCase):
    def test_simple_gp(self):
        m = pe.ConcreteModel()
        gp, norm = train_warped_gp(20, 0.05)
        gp = rogp.from_gpy(gp)

        m.x = pe.Var(range(2))
        m.z = pe.Var(range(2))

        m.uncset = ro.uncset.WarpedGPSet(gp, m.z, 0.95)
        m.w = ro.UncParam(range(2), uncset=m.uncset)

        m.c = pe.Constraint(expr=m.x[0]*m.w[0] + m.x[1]*m.w[1] <= 1)

        t = pe.TransformationFactory('romodel.warpedgp')
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'c_counterpart'))
        self.assertTrue(hasattr(m.c_counterpart, 'y'))
        self.assertTrue(hasattr(m.c_counterpart, 'primal'))
        self.assertTrue(hasattr(m.c_counterpart, 'stationarity'))
        self.assertTrue(hasattr(m.c_counterpart, 'dual'))
        self.assertEqual(len(m.c_counterpart.y), 2)

        repn = generate_standard_repn(m.c_counterpart.primal.body)
        self.assertEqual(repn.constant, 0)
        self.assertEqual(len(repn.linear_vars), 0)
        baseline = []
        for i in range(2):
            idx = id(m.x[i])
            idy = id(m.c_counterpart.y[i])
            if idx <= idy:
                baseline.append((idx, idy))
            else:
                baseline.append((idy, idx))
        self.assertEqual(len(repn.quadratic_vars), 2)
        for x, y in repn.quadratic_vars:
            idx = id(x)
            idy = id(y)
            if idx <= idy:
                self.assertIn((idx, idy), baseline)
            else:
                self.assertIn((idy, idx), baseline)

        self.assertEqual(len(repn.nonlinear_vars), 0)
        self.assertEqual(len(m.c_counterpart.stationarity), 2)

    def test_simple_gp_constant(self):
        m = pe.ConcreteModel()
        gp, norm = train_warped_gp(20, 0.05)
        gp = rogp.from_gpy(gp)

        m.x = pe.Var(range(2))
        m.z = pe.Var(range(2))

        m.uncset = ro.uncset.WarpedGPSet(gp, {i: [m.z[i]] for i in m.z}, 0.95)
        m.w = ro.UncParam(range(2), uncset=m.uncset)

        m.c = pe.Constraint(expr=m.x[0] + m.x[0]*m.w[0] + m.x[1]*m.w[1] <= 1)

        t = pe.TransformationFactory('romodel.warpedgp')
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'c_counterpart'))
        self.assertTrue(hasattr(m.c_counterpart, 'y'))
        self.assertTrue(hasattr(m.c_counterpart, 'primal'))
        self.assertTrue(hasattr(m.c_counterpart, 'stationarity'))
        self.assertTrue(hasattr(m.c_counterpart, 'dual'))
        self.assertEqual(len(m.c_counterpart.y), 2)

        repn = generate_standard_repn(m.c_counterpart.primal.body)
        self.assertEqual(repn.constant, 0)
        self.assertEqual(len(repn.linear_vars), 1)
        self.assertEqual(id(repn.linear_vars[0]), id(m.x[0]))
        self.assertEqual(len(repn.quadratic_vars), 2)
        baseline = []
        for i in range(2):
            idx = id(m.x[i])
            idy = id(m.c_counterpart.y[i])
            if idx <= idy:
                baseline.append((idx, idy))
            else:
                baseline.append((idy, idx))
        self.assertEqual(len(repn.quadratic_vars), 2)
        for x, y in repn.quadratic_vars:
            idx = id(x)
            idy = id(y)
            if idx <= idy:
                self.assertIn((idx, idy), baseline)
            else:
                self.assertIn((idy, idx), baseline)

        self.assertEqual(len(repn.nonlinear_vars), 0)
        self.assertEqual(len(m.c_counterpart.stationarity), 2)

    def test_objective(self):
        m = pe.ConcreteModel()
        gp, norm = train_warped_gp(20, 0.05)
        gp = rogp.from_gpy(gp)

        m.x = pe.Var(range(2))
        m.z = pe.Var(range(2))

        m.uncset = ro.uncset.WarpedGPSet(gp, m.z, 0.95)
        m.w = ro.UncParam(range(2), uncset=m.uncset)

        m.c = pe.Objective(expr=m.x[0] + m.x[0]*m.w[0] + m.x[1]*m.w[1])

        t = pe.TransformationFactory('romodel.warpedgp')
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'c_counterpart'))
        self.assertTrue(hasattr(m.c_counterpart, 'y'))
        self.assertTrue(hasattr(m.c_counterpart, 'primal'))
        self.assertTrue(hasattr(m.c_counterpart, 'stationarity'))
        self.assertTrue(hasattr(m.c_counterpart, 'dual'))
        self.assertEqual(len(m.c_counterpart.y), 2)

        repn = generate_standard_repn(m.c_counterpart.primal.expr)
        self.assertEqual(repn.constant, 0)
        self.assertEqual(len(repn.linear_vars), 1)
        self.assertEqual(id(repn.linear_vars[0]), id(m.x[0]))
        self.assertEqual(len(repn.quadratic_vars), 2)
        baseline = []
        for i in range(2):
            idx = id(m.x[i])
            idy = id(m.c_counterpart.y[i])
            if idx <= idy:
                baseline.append((idx, idy))
            else:
                baseline.append((idy, idx))
        self.assertEqual(len(repn.quadratic_vars), 2)
        for x, y in repn.quadratic_vars:
            idx = id(x)
            idy = id(y)
            if idx <= idy:
                self.assertIn((idx, idy), baseline)
            else:
                self.assertIn((idy, idx), baseline)

        self.assertEqual(len(repn.nonlinear_vars), 0)
        self.assertEqual(len(m.c_counterpart.stationarity), 2)
