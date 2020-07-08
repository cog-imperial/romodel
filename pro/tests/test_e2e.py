import pyutilib.th as unittest
import pyomo.environ as pe
import pro.examples


class TestE2E(unittest.TestCase):
    def test_knapsack_reformulation(self):
        m = pro.examples.Knapsack()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.solve(m, tee=False)

    def test_knapsack_cuts(self):
        m = pro.examples.Knapsack()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.solve(m, tee=False)

    def test_portfolio_reformulation(self):
        m = pro.examples.Portfolio()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.solve(m, tee=False)

    def test_portfolio_cuts(self):
        m = pro.examples.Portfolio()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.solve(m, tee=False)

    def test_pooling_reformulation(self):
        m = pro.examples.Pooling()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.solve(m, tee=False)

    def test_pooling_cuts(self):
        m = pro.examples.Pooling()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.solve(m, tee=False)
