import pyomo.environ as pe
import pyutilib.th as unittest
import pro
from pyomo.repn import generate_standard_repn


class TestAdjustable(unittest.TestCase):
    def test_second_stage_var(self):
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.x = pe.Var()
        m.y = pro.SecondStageVar()
        m.c = pe.Constraint(expr=m.p + m.x + m.y <= 0)
        t = pe.TransformationFactory('pro.ldr')
        t.apply_to(m)
        repn = generate_standard_repn(m.c.body)
        self.assertEqual(len(repn.linear_vars), 2)
        self.assertIn(m.p, repn.linear_vars)
        self.assertIn(m.x, repn.linear_vars)
        self.assertTupleEqual((1, 1), repn.linear_coefs)
        self.assertEqual(len(repn.quadratic_vars), 1)
        self.assertEqual(repn.quadratic_coefs, [1])
        self.assertIn(m.y_p_coef, repn.quadratic_vars[0])
        self.assertIn(m.p, repn.quadratic_vars[0])

    def test_different_ways_of_modeling_second_stage(self):
        # New second stage variable component
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.x = pe.Var()
        m.y = pro.SecondStageVar()
        m.c = pe.Constraint(expr=m.p + m.x + m.y <= 0)
        # Pass second stage vars as list to solve
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.x = pe.Var()
        m.y = pe.Var()
        m.c = pe.Constraint(expr=m.p + m.x + m.y <= 0)
        solver = pe.SolverFactory('gurobi')
        solver.solve(m, second_stage_vars=[m.y])
        # Use attribute on variable
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.x = pe.Var()
        m.y = pe.Var()
        m.y.stage = 2
        m.c = pe.Constraint(expr=m.p + m.x + m.y <= 0)


