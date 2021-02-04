import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples
import romodel


class TestNominalSolver(unittest.TestCase):
    def test_nominal_solver(self):
        m = romodel.examples.Knapsack()
        opt = pe.SolverFactory('romodel.nominal')
        opt.options['solver'] = 'gurobi_direct'
        opt.solve(m)

        self.assertEqual(m.value(), 25)
        self.assertEqual(m.x['hammer'](), 1)
        self.assertEqual(m.x['wrench'](), 0)
        self.assertEqual(m.x['screwdriver'](), 1)
        self.assertEqual(m.x['towel'](), 1)
