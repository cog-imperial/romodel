import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples


class TestE2E(unittest.TestCase):
    def test_knapsack_reformulation(self):
        m = romodel.examples.Knapsack()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m = romodel.examples.Knapsack()
        m.w.uncset = m.P
        solver.solve(m, tee=False)

    def test_knapsack_cuts(self):
        m = romodel.examples.Knapsack()
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_portfolio_reformulation(self):
        m = romodel.examples.Portfolio()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_portfolio_cuts(self):
        m = romodel.examples.Portfolio()
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_pooling_reformulation_ellipsoidal(self):
        m = romodel.examples.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_pooling_reformulation_polyhedral(self):
        m = romodel.examples.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    def test_pooling_cuts(self):
        m = romodel.examples.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    def test_pooling_convex_cuts(self):
        m = romodel.examples.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['NonConvex'] = 2
        m.price_product.uncset = m.C
        solver.solve(m, tee=False)
