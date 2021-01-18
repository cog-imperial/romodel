import pyomo.environ as pe
import pyutilib.th as unittest
import romodel as ro
from romodel.generator import generate_linear_repn
from pyomo.opt import check_available_solvers
from pyomo.repn import generate_standard_repn

solvers = check_available_solvers('gurobi')


class TestGenerator(unittest.TestCase):
    def test_repn(self):
        m = pe.ConcreteModel()
        m.x = pe.Var([0, 1])
        m.z = pe.Var()
        m.w = ro.UncParam([0, 1], nominal=(0.5, 0.5))
        expr = m.z + m.x[0]*m.w[0] + m.x[1]*m.w[1]

        baseline_param = set([id(m.w[i]) for i in m.w])
        baseline_vars = set([id(m.x[i]) for i in m.w])

        repn = generate_linear_repn(expr)
        self.assertEqual(set(id(i) for i in repn.linear_coefs), baseline_vars)
        self.assertEqual(set(id(i) for i in repn.linear_vars), baseline_param)
        self.assertEqual(id(repn.constant), id(m.z))

    def test_nominal(self):
        m = pe.ConcreteModel()
        m.x = pe.Var([0, 1])
        m.w = ro.UncParam([0, 1], nominal=(0.5, 0.5))
        # for i in m.w:
        #     m.w[i].value = 0.5
        m.c = pe.Constraint(expr=m.x[0]*m.w[0] + m.x[1]*m.w[1] <= 1)

        m.rc = ro.RobustConstraint()
        m.rc.build(m.c.lower, m.c.body, m.c.upper)

        lb, nominal, ub = m.rc.nominal_constraint_expr()
        repn = generate_standard_repn(nominal)

        self.assertIsNone(lb)
        self.assertEqual(pe.value(ub), 1.0)
        self.assertEqual(repn.linear_coefs, (0.5, 0.5))
        self.assertEqual(repn.linear_coefs, (0.5, 0.5))
        self.assertEqual(set(id(i) for i in repn.linear_vars),
                         set(id(m.x[j]) for j in m.x))

    def test_construct_separation_problem(self):
        m = pe.ConcreteModel()
        m.x = pe.Var([0, 1])
        m.U = ro.UncSet()
        m.w = ro.UncParam([0, 1], nominal=(0.5, 0.5), uncset=m.U)
        m.U.c0 = pe.Constraint(expr=m.w[0] <= 1)
        m.U.c1 = pe.Constraint(expr=m.w[1] <= 1)
        # for i in m.w:
        #     m.w[i].value = 0.5.lower, m.c.expr, m.c.upper
        for i in m.x:
            m.x[i].value = 0.8
        m.c = pe.Constraint(expr=m.x[0]*m.w[0] + m.x[1]*m.w[1] <= 1)

        m.rc = ro.RobustConstraint()
        m.rc.build(m.c.lower, m.c.body, m.c.upper)

        sep = m.rc.construct_separation_problem()
        repn = generate_standard_repn(sep.obj)
        self.assertEqual(repn.linear_coefs, (0.8, 0.8))
