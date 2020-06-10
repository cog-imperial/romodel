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
        t = EllipsoidalTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)

    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_ellipsoidal_lib(self):
        m = Knapsack()
        m.w.uncset = m.Elib
        t = PolyhedralTransformation()
        t.apply_to(m)
        solver = SolverFactory('gurobi')
        solver.solve(m)
        self.assertEqual(m.value(), 19.)


if __name__ == "__main__":
    unittest.main()
