import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples as ex


class TestE2E(unittest.TestCase):
    def test_knapsack_reformulation(self):
        m = ex.Knapsack()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m = ex.Knapsack()
        m.w.uncset = m.P
        solver.solve(m, tee=False)

    def test_knapsack_cuts(self):
        m = ex.Knapsack()
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_knapsack_cuts_poly_lib(self):
        m = ex.Knapsack()
        m.w.uncset = m.Plib
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_knapsack_cuts_ellipsoidal_lib(self):
        m = ex.Knapsack()
        m.w.uncset = m.Elib
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_portfolio_reformulation(self):
        m = ex.Portfolio()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_portfolio_cuts(self):
        m = ex.Portfolio()
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_pooling_reformulation_ellipsoidal(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    def test_pooling_reformulation_polyhedral(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    def test_pooling_cuts(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    def test_pooling_convex_cuts(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['NonConvex'] = 2
        m.price_product.uncset = m.C
        solver.solve(m, tee=False)

    def test_facility_nominal(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.nominal')
        solver.solve(m, tee=False)

    def test_facility_ldr_reformulation(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['NonConvex'] = 2
        solver.solve(m, tee=False)

    def test_facility_ldr_cuts(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.cuts')
        solver.solve(m, tee=False)

    def test_planning_wgp_reform(self):
        m = ex.ProductionPlanning()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'ipopt'
        solver.solve(m, tee=False)

    def test_planning_gp_reform(self):
        m = ex.ProductionPlanning()
        m.demand.uncset = m.uncset_standard
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'ipopt'
        solver.solve(m, tee=False)
