import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples as ex
from pyomo.opt import check_available_solvers

solvers = check_available_solvers('gurobi_direct', 'ipopt')


class TestE2E(unittest.TestCase):
    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_knapsack_reformulation(self):
        m = ex.Knapsack()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)
        m = ex.Knapsack()
        m.w.uncset = m.P
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_knapsack_cuts(self):
        m = ex.Knapsack()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_knapsack_cuts_poly_lib(self):
        m = ex.Knapsack()
        m.w.uncset = m.Plib
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_knapsack_cuts_ellipsoidal_lib(self):
        m = ex.Knapsack()
        m.w.uncset = m.Elib
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_portfolio_reformulation(self):
        m = ex.Portfolio()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_portfolio_cuts(self):
        m = ex.Portfolio()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_pooling_reformulation_ellipsoidal(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_pooling_reformulation_polyhedral(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_pooling_cuts(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.options['MIPGap'] = 0.02
        solver.solve(m, tee=False)
        m = ex.Pooling()
        m.price_product.uncset = m.P
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_pooling_convex_cuts(self):
        m = ex.Pooling()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        m.price_product.uncset = m.C
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_facility_nominal(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.nominal')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_facility_ldr_reformulation(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['NonConvex'] = 2
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('gurobi_direct' not in solvers,
                     'gurobi_direct not available')
    def test_facility_ldr_cuts(self):
        m = ex.Facility()
        solver = pe.SolverFactory('romodel.cuts')
        solver.options['solver'] = 'gurobi_direct'
        solver.options['TimeLimit'] = 60
        solver.solve(m, tee=False)

    @unittest.skipIf('ipopt' not in solvers,
                     'ipopt not available')
    def test_planning_wgp_reform(self):
        m = ex.ProductionPlanning()
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'ipopt'
        solver.solve(m, tee=True)

    @unittest.skipIf('ipopt' not in solvers,
                     'ipopt not available')
    def test_planning_gp_reform(self):
        m = ex.ProductionPlanning(warped=False)
        solver = pe.SolverFactory('romodel.reformulation')
        solver.options['solver'] = 'ipopt'
        solver.solve(m, tee=False)


if __name__ == '__main__':
    unittest.main()
