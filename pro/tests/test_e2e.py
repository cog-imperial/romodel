import pyutilib.th as unittest
import pyomo.environ as pe
import pro.examples


class TestE2E(unittest.TestCase):
    def test_knapsack_reformulation(self):
        m = pro.examples.Knapsack()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m = pro.examples.Knapsack()
        m.w.uncset = m.P
        solver.solve(m, tee=False)

    def test_knapsack_cuts(self):
        m = pro.examples.Knapsack()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.solve(m, tee=False)

    def test_portfolio_reformulation(self):
        m = pro.examples.Portfolio()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_portfolio_cuts(self):
        m = pro.examples.Portfolio()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.solve(m, tee=False)

    def test_pooling_reformulation_ellipsoidal(self):
        m = pro.examples.Pooling()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_pooling_reformulation_polyhedral(self):
        m = pro.examples.Pooling()
        solver = pe.SolverFactory('pro.robust.reformulation')
        solver.options['NonConvex'] = 2
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    def test_pooling_cuts(self):
        m = pro.examples.Pooling()
        solver = pe.SolverFactory('pro.robust.cuts')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)
