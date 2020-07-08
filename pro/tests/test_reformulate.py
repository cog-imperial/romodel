from pyomo.environ import SolverFactory
import pyutilib.th as unittest
from pro.examples import Knapsack
from pro.reformulate import (EllipsoidalTransformation,
                             PolyhedralTransformation)
from pyomo.opt import check_available_solvers

solvers = check_available_solvers('gurobi')


class TestReformulation(unittest.TestCase):
    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_polyhedral(self):
        m = Knapsack()
        m.w.uncset = m.P
        self.assertTrue(m.P.is_polyhedral())
        self.assertTrue(m.Plib.is_polyhedral())
        self.assertFalse(m.E.is_polyhedral())
        self.assertFalse(m.Elib.is_polyhedral())
        t = PolyhedralTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_polyhedral_lib(self):
        m = Knapsack()
        m.w.uncset = m.Plib
        t = PolyhedralTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_ellipsoidal(self):
        m = Knapsack()
        m.w.uncset = m.E
        self.assertFalse(m.P.is_ellipsoidal())
        self.assertFalse(m.Plib.is_ellipsoidal())
        self.assertTrue(m.E.is_ellipsoidal())
        self.assertTrue(m.Elib.is_ellipsoidal())
        t = EllipsoidalTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 25.)

    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_ellipsoidal_lib(self):
        m = Knapsack()
        m.w.uncset = m.Elib
        t = EllipsoidalTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 25.)


if __name__ == "__main__":
    unittest.main()
