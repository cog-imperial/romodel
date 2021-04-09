import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples
import romodel as ro
from romodel.reformulate import (EllipsoidalTransformation,
                                 PolyhedralTransformation)
from pyomo.opt import check_available_solvers
from pyomo.repn import generate_standard_repn

solvers = check_available_solvers('gurobi_direct')


class TestReformulation(unittest.TestCase):
    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_polyhedral(self):
        m = romodel.examples.Knapsack()
        m.w.uncset = m.P
        t = PolyhedralTransformation()
        self.assertTrue(t._check_applicability(m.P))
        self.assertTrue(t._check_applicability(m.Plib))
        self.assertFalse(t._check_applicability(m.E))
        self.assertFalse(t._check_applicability(m.Elib))
        t.apply_to(m)
        solver = pe.SolverFactory('gurobi_direct')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_polyhedral_lib(self):
        m = romodel.examples.Knapsack()
        m.w.uncset = m.Plib
        t = PolyhedralTransformation()
        t.apply_to(m)
        solver = pe.SolverFactory('gurobi_direct')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    def test_polyhedral_cons_lb(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.P = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.P)
        m.P.cons = pe.ConstraintList()
        m.P.cons.add(pe.inequality(0.5, m.w[0], 1.5))
        m.P.cons.add(pe.inequality(1.5, m.w[1], 2.5))

        expr = pe.sum_product(m.w, m.x)
        m.cons = pe.Constraint(expr=2 <= expr)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.minimize)
        t = ro.PolyhedralTransformation()
        t.apply_to(m)
        self.assertFalse(m.cons.active)
        self.assertTrue(hasattr(m, 'cons_counterpart'))
        self.assertTrue(hasattr(m.cons_counterpart, 'lower'))

    def test_polyhedral_cons_ub(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.P = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.P)
        m.P.cons = pe.ConstraintList()
        m.P.cons.add(pe.inequality(0.5, m.w[0], 1.5))
        m.P.cons.add(pe.inequality(1.5, m.w[1], 2.5))

        expr = pe.sum_product(m.w, m.x)
        m.cons = pe.Constraint(expr=expr <= 2)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.maximize)
        t = ro.PolyhedralTransformation()
        t.apply_to(m)
        self.assertFalse(m.cons.active)
        self.assertTrue(hasattr(m, 'cons_counterpart'))
        self.assertTrue(hasattr(m.cons_counterpart, 'upper'))

    def test_polyhedral_obj_min(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.P = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.P)
        m.P.cons = pe.ConstraintList()
        m.P.cons.add(pe.inequality(0.5, m.w[0], 1.5))
        m.P.cons.add(pe.inequality(1.5, m.w[1], 2.5))

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.minimize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) >= 4)
        t = ro.PolyhedralTransformation()
        t.apply_to(m)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertTrue(hasattr(m.obj_counterpart, 'obj'))
        self.assertTrue(hasattr(m.obj_counterpart, 'dual'))
        self.assertIs(m.obj_counterpart.obj.sense, pe.minimize)

    def test_polyhedral_obj_max(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.P = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.P)
        m.P.cons = pe.ConstraintList()
        m.P.cons.add(pe.inequality(0.5, m.w[0], 1.5))
        m.P.cons.add(pe.inequality(1.5, m.w[1], 2.5))

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.maximize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) <= 4)
        t = ro.PolyhedralTransformation()
        t.apply_to(m)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertTrue(hasattr(m.obj_counterpart, 'obj'))
        self.assertTrue(hasattr(m.obj_counterpart, 'dual'))
        self.assertIs(m.obj_counterpart.obj.sense, pe.maximize)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_ellipsoidal(self):
        m = romodel.examples.Knapsack()
        m.w.uncset = m.E
        t = EllipsoidalTransformation()
        self.assertFalse(t._check_applicability(m.P))
        self.assertFalse(t._check_applicability(m.Plib))
        self.assertTrue(t._check_applicability(m.E))
        self.assertTrue(t._check_applicability(m.Elib))
        t.apply_to(m)
        solver = pe.SolverFactory('gurobi_direct')
        solver.options['NonConvex'] = 2
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_ellipsoidal_lib(self):
        m = romodel.examples.Knapsack()
        m.w.uncset = m.Elib
        t = EllipsoidalTransformation()
        t.apply_to(m)
        solver = pe.SolverFactory('gurobi_direct')
        solver.options['NonConvex'] = 2
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    def test_ellipsoidal_cons_lb(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.cons = pe.Constraint(expr=2 <= expr)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.minimize)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=False)
        self.assertFalse(m.cons.active)
        self.assertTrue(hasattr(m, 'cons_counterpart'))
        self.assertTrue(hasattr(m.cons_counterpart, 'lower'))
        self.assertTrue(hasattr(m.cons_counterpart.lower, 'rob'))

    def test_ellipsoidal_objective_min(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.minimize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) >= 4)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=False)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertTrue(hasattr(m.obj_counterpart, 'padding'))
        self.assertTrue(hasattr(m.obj_counterpart, 'det'))
        self.assertTrue(hasattr(m.obj_counterpart, 'rob'))
        self.assertIs(m.obj_counterpart.rob.sense, pe.minimize)

    def test_ellipsoidal_objective_max(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.maximize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) <= 4)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=False)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertTrue(hasattr(m.obj_counterpart, 'padding'))
        self.assertTrue(hasattr(m.obj_counterpart, 'det'))
        self.assertTrue(hasattr(m.obj_counterpart, 'rob'))
        self.assertIs(m.obj_counterpart.rob.sense, pe.maximize)

    def test_ellipsoidal_cons_lb_root(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.cons = pe.Constraint(expr=2 <= expr)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.minimize)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=True)
        self.assertFalse(m.cons.active)
        self.assertTrue(hasattr(m, 'cons_counterpart'))
        self.assertTrue(hasattr(m.cons_counterpart, 'lower'))
        self.assertTrue(hasattr(m.cons_counterpart.lower, 'rob'))

    def test_ellipsoidal_obj_min_root(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.minimize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) >= 4)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=True)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertFalse(hasattr(m.obj_counterpart, 'det'))
        self.assertTrue(hasattr(m.obj_counterpart, 'rob'))
        self.assertIs(m.obj_counterpart.rob.sense, pe.minimize)

    def test_ellipsoidal_obj_max_root(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        expr = ((m.w[0] - 1)**2
                + 0.1*(m.w[0] - 1)*(m.w[1] - 2)
                + (m.w[1] - 2)**2
                <= 0.1)
        m.U.cons = pe.Constraint(expr=expr)

        expr = pe.sum_product(m.w, m.x)
        m.obj = pe.Objective(expr=expr, sense=pe.maximize)
        m.cons = pe.Constraint(expr=pe.quicksum(m.x[i] for i in m.x) <= 4)
        t = ro.EllipsoidalTransformation()
        t.apply_to(m, root=True)
        self.assertFalse(m.obj.active)
        self.assertTrue(hasattr(m, 'obj_counterpart'))
        self.assertFalse(hasattr(m.obj_counterpart, 'det'))
        self.assertTrue(hasattr(m.obj_counterpart, 'rob'))
        self.assertIs(m.obj_counterpart.rob.sense, pe.maximize)

    # def test_ellipsoidal_lib_root(self):
    #     m = romodel.examples.Knapsack()
    #     m.w.uncset = m.Elib
    #     t = EllipsoidalTransformation()
    #     t.apply_to(m, root=True)
    #     solver = pe.SolverFactory('gams')
    #     solver.options['solver'] = 'Baron'
    #     solver.solve(m)
    #     self.assertEqual(m.value(), 25.)

    def test_empty_uncset(self):
        m = romodel.examples.Knapsack()
        m.Uempty = ro.UncSet()
        m.w.uncset = m.Uempty
        solver = pe.SolverFactory('romodel.reformulation')
        self.assertRaises(AssertionError, lambda: solver.solve(m))

    def test_unknown_uncset(self):
        m = pe.ConcreteModel()
        m.x = pe.Var(range(2))
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(2), nominal=(1, 2), uncset=m.U)
        m.obj = pe.Objective(expr=pe.sum_product(m.w, m.x), sense=pe.maximize)
        m.cons = pe.Objective(expr=pe.quicksum(m.x[i] for i in m.x) <= 4)
        m.U.cons = pe.Constraint(expr=(m.w[0] - 1)**4 + pe.sin(m.w[1]) <= 1)
        solver = pe.SolverFactory('romodel.reformulation')

        msg = "Cannot reformulate UncSet with unknown geometry: U"
        try:
            solver.solve(m)
        except RuntimeError as e:
            self.assertEqual(str(e), msg)
        else:
            self.fail('"solver.solve was expected to throw RuntimeError')

    def test_uncparam_has_no_uncset(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam(range(3), nominal=(1, 2, 3))
        m.x = pe.Var(range(3))
        expr = pe.quicksum(m.w[i]*m.x[i] for i in range(3))
        m.cons = pe.Constraint(expr=expr <= 5)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.maximize)
        solver = pe.SolverFactory('romodel.reformulation')
        self.assertRaises(AssertionError, lambda: solver.solve(m))

    def test_cons_nonlinear_in_uncparam(self):
        m = pe.ConcreteModel()
        m.U = ro.UncSet()
        m.w = ro.UncParam(range(3), nominal=(1, 2, 3), uncset=m.U)
        m.x = pe.Var(range(3))
        expr = pe.quicksum(m.w[i]**2 * m.x[i] for i in range(3))
        m.cons = pe.Constraint(expr=expr <= 5)
        m.obj = pe.Objective(expr=m.x[0], sense=pe.maximize)
        solver = pe.SolverFactory('romodel.reformulation')
        self.assertRaises(AssertionError, lambda: solver.solve(m))

    def test_nominal_transform(self):
        m = pe.ConcreteModel()
        m.x = pe.Var()
        m.y = pe.Var(range(2))
        m.u = ro.UncParam(nominal=3.)
        m.w = ro.UncParam(range(2), nominal=[1, 2])
        m.c = pe.Constraint(expr=m.y[0]*m.w[0] + m.y[1]*m.w[1] <= 1)
        m.o = pe.Objective(expr=m.u**2*m.x**2)
        t = pe.TransformationFactory('romodel.nominal')
        t.apply_to(m)
        repn = generate_standard_repn(m.c.body)
        self.assertEqual(len(repn.linear_vars), 2)
        self.assertEqual(len(repn.quadratic_vars), 0)
        self.assertIsNone(repn.nonlinear_expr)
        baseline = {id(m.y[0]): 1,
                    id(m.y[1]): 2}
        for v, c in zip(repn.linear_vars, repn.linear_coefs):
            self.assertEqual(baseline[id(v)], c)

        repn = generate_standard_repn(m.o.expr, compute_values=False)
        self.assertEqual(len(repn.linear_vars), 0)
        self.assertEqual(len(repn.quadratic_vars), 1)
        self.assertIsNone(repn.nonlinear_expr)
        self.assertEqual(repn.quadratic_coefs[0], 3.**2)

    def test_create_linear_dual(self):
        c = [0.5, 0.7]
        b = 0.1
        P = [[1, 0], [0, 1], [-1, 0], [0, -1]]
        d = [1.2, 1.3, 0.9, 0.8]
        t = PolyhedralTransformation()
        blk = t.create_linear_dual(c, b, P, d)
        # Right number of dual vars
        self.assertEqual(len(blk.var), 4)
        # Dual objective
        repn = generate_standard_repn(blk.obj.body)
        self.assertEqual(repn.linear_coefs, (1.2, 1.3, 0.9, 0.8))
        # Right number of constraints
        self.assertEqual(len(blk.cons), 2)
        # Dual cons 1:
        repn = generate_standard_repn(blk.cons[1].body)
        self.assertEqual(repn.linear_coefs, (1, -1))
        self.assertEqual(repn.linear_vars, (blk.var[0], blk.var[2]))

        # Dual cons 2:
        repn = generate_standard_repn(blk.cons[2].body)
        self.assertEqual(repn.linear_coefs, (1, -1))
        self.assertEqual(repn.linear_vars, (blk.var[1], blk.var[3]))


if __name__ == "__main__":
    unittest.main()
