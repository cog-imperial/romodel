import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples
import romodel as ro
from romodel.adjustable import (LDRAdjustableTransformation)
from pyomo.opt import check_available_solvers
from pyomo.repn import generate_standard_repn

solvers = check_available_solvers('gurobi')


class TestLDR(unittest.TestCase):
    def test_simple_one_uncparam_ldr(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.y = ro.AdjustableVar(uncparams=[m.w])
        m.cons = pe.Constraint(expr=sum(m.y[i] for i in m.y) <= 1)

        t = LDRAdjustableTransformation()
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'cons_ldr'))
        repn = generate_standard_repn(m.cons_ldr.body)
        self.assertEqual(len(repn.linear_vars), 0)
        self.assertEqual(len(repn.quadratic_vars), 3)
        baseline = set((id(m.w[i]), id(m.y_w_coef[j, i]))
                       for i in m.w for j in m.y)
        for x in repn.quadratic_vars:
            self.assertIn((id(x[0]), id(x[1])), baseline)

    def test_indexed_one_uncparam_ldr(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.u = ro.UncParam()
        m.y = ro.AdjustableVar([0, 1, 2], uncparams=[m.w])
        m.z = ro.AdjustableVar([0, 1], uncparams=[m.w, m.u])
        m.cons = pe.Constraint(expr=sum(m.y[i] for i in m.y) <= 1)

        t = LDRAdjustableTransformation()
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'cons_ldr'))
        repn = generate_standard_repn(m.cons_ldr.body)
        self.assertEqual(len(repn.linear_vars), 0)
        self.assertEqual(len(repn.quadratic_vars), 9)
        baseline = set((id(m.w[i]), id(m.y_w_coef[j, i]))
                       for i in m.w for j in m.y)
        for x in repn.quadratic_vars:
            self.assertIn((id(x[0]), id(x[1])), baseline)

    def test_indexed_two_uncparams(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.u = ro.UncParam()
        m.y = ro.AdjustableVar([0, 1, 2], uncparams=[m.w])
        m.z = ro.AdjustableVar([0, 1], uncparams=[m.w, m.u])
        m.cons = pe.Constraint(expr=m.u + sum(m.z[i] for i in m.z) <= 1)

        t = LDRAdjustableTransformation()
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'cons_ldr'))
        repn = generate_standard_repn(m.cons_ldr.body)
        self.assertEqual(len(repn.linear_vars), 1)
        self.assertEqual(len(repn.quadratic_vars), 8)
        baseline = set((id(m.w[i]), id(m.z_w_coef[j, i]))
                       for i in m.w for j in m.z)
        baseline = baseline.union((id(m.u[None]), id(m.z_u_coef[j, None]))
                                  for j in m.z)
        for x in repn.quadratic_vars:
            self.assertIn((id(x[0]), id(x[1])), baseline)

    def test_equality_one_uncparam(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.y = ro.AdjustableVar([0, 1], uncparams=[m.w])
        m.cons = pe.Constraint(expr=(sum(m.w[i] for i in m.w)
                                     == sum(2*m.y[i] for i in m.y)))

        t = LDRAdjustableTransformation()
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'cons_ldr'))
        baseline = set()
        for i in m.w:
            id1 = id(m.y_w_coef[0, i])
            id2 = id(m.y_w_coef[1, i])
            if id1 < id2:
                baseline.add((id1, id2))
            else:
                baseline.add((id2, id1))
        for c in m.cons_ldr.values():
            repn = generate_standard_repn(c.body)
            self.assertEqual(repn.constant, 1)
            self.assertEqual(len(repn.linear_vars), 2)
            self.assertEqual(repn.linear_coefs, (-2, -2))
            self.assertEqual(len(repn.quadratic_vars), 0)

            id1 = id(repn.linear_vars[0])
            id2 = id(repn.linear_vars[1])
            if id1 < id2:
                self.assertIn((id1, id2), baseline)
            else:
                self.assertIn((id2, id1), baseline)

    def test_equality_one_uncparam_cons_expr(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.x = pe.Var([0, 1])
        m.y = ro.AdjustableVar([0, 1], uncparams=[m.w])
        m.cons = pe.Constraint(expr=(sum(m.w[i] for i in m.w)
                                     == sum(2*m.y[i] for i in m.y)
                                     + 2*m.x[0] + 2*m.x[1] - 1))

        t = LDRAdjustableTransformation()
        t.apply_to(m)

        self.assertTrue(hasattr(m, 'cons_ldr'))
        baseline = set()
        for i in m.w:
            id1 = id(m.y_w_coef[0, i])
            id2 = id(m.y_w_coef[1, i])
            if id1 < id2:
                baseline.add((id1, id2))
            else:
                baseline.add((id2, id1))
        id1 = id(m.x[0])
        id2 = id(m.x[1])
        if id1 < id2:
            baseline.add((id1, id2))
        else:
            baseline.add((id2, id1))

        for c in m.cons_ldr.values():
            repn = generate_standard_repn(c.body)
            self.assertEqual(repn.constant, 1)
            self.assertEqual(len(repn.linear_vars), 2)
            self.assertEqual(repn.linear_coefs, (-2, -2))
            self.assertEqual(len(repn.quadratic_vars), 0)

            id1 = id(repn.linear_vars[0])
            id2 = id(repn.linear_vars[1])
            if id1 < id2:
                self.assertIn((id1, id2), baseline)
            else:
                self.assertIn((id2, id1), baseline)

    def test_equality_one_uncparam_cons(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.x = pe.Var([0, 1])
        m.y = ro.AdjustableVar([0, 1], uncparams=[m.w])
        m.cons = pe.Constraint(expr=(sum(m.w[i] for i in m.w)
                                     == sum(2*m.y[i] for i in m.y) + 1))

        t = LDRAdjustableTransformation()
        self.assertRaises(ValueError, lambda: t.apply_to(m))


class TestAdjustable(unittest.TestCase):
    def test_simple_adjustable(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.y = ro.AdjustableVar(uncparams=[m.w])
        self.assertFalse(m.y.is_constant())
        self.assertTrue(m.y.is_potentially_variable())
        self.assertTrue(m.y.is_variable_type())
        self.assertFalse(m.y.is_parameter_type())
        self.assertIs(m.y.ctype, ro.AdjustableVar)

    def test_indexed_adjustable(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.y = ro.AdjustableVar([0, 1], uncparams=[m.w])
        self.assertFalse(m.y[0].is_constant())
        self.assertTrue(m.y[1].is_potentially_variable())
        self.assertTrue(m.y[0].is_variable_type())
        self.assertFalse(m.y[1].is_parameter_type())
        self.assertIs(m.y[0].ctype, ro.AdjustableVar)
