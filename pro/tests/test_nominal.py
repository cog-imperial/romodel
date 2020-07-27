import pyutilib.th as unittest
import pyomo.environ as pe
import pro.examples
import pro


class TestNominalSolver(unittest.TestCase):
    def test_nominal_solver(self):
        m = pro.examples.Knapsack()
        opt = pe.SolverFactory('pro.nominal')
        opt.solve(m)

        self.assertEqual(m.value(), 25)
        self.assertEqual(m.x['hammer'](), 1)
        self.assertEqual(m.x['wrench'](), 0)
        self.assertEqual(m.x['screwdriver'](), 1)
        self.assertEqual(m.x['towel'](), 1)
