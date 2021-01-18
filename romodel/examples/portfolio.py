import pyomo.environ as pe
import romodel as ro


def Portfolio():
    m = pe.ConcreteModel()
    m.cons = pe.ConstraintList()

    N = 5
    index = list(range(N))
    mean = [0.1, 0.3, 0.5, 0.7, 0.4]

    m.x = pe.Var(index, bounds=(0, 1))
    m.z = pe.Var(within=pe.PositiveReals)

    m.U = ro.UncSet()
    m.r = ro.UncParam(index, uncset=m.U, nominal=mean)
    r = m.r
    expr = 0
    for i in index:
        expr += (m.r[i] - mean[i])**2
    m.U.cons = pe.Constraint(expr=expr <= 0.0005)
    m.Elib = ro.uncset.EllipsoidalSet(mean,
                                      [[0.0005, 0, 0, 0, 0],
                                       [0, 0.0005, 0, 0, 0],
                                       [0, 0, 0.0005, 0, 0],
                                       [0, 0, 0, 0.0005, 0],
                                       [0, 0, 0, 0, 0.0005]])

    P = [[1, 1, 0, 0, 0],
         [-1, 1, 0, 0, 0],
         [1, -1, 0, 0, 0],
         [-1, -1, 0, 0, 0],
         [0, 0, 1, 0, 0],
         [0, 0, -1, 0, 0],
         [0, 0, 0, 1, 0],
         [0, 0, 0, -1, 0],
         [0, 0, 0, 0, 1],
         [0, 0, 0, 0, -1]]
    rhs = [0.001 + mean[0] + mean[1],
           0.001 - mean[0] + mean[1],
           0.001 + mean[0] - mean[1],
           0.001 - mean[0] - mean[1],
           0.001 + mean[2],
           0.001 - mean[2],
           0.001 + mean[3],
           0.001 - mean[3],
           0.001 + mean[4],
           0.001 - mean[4]]
    m.Plib = ro.uncset.PolyhedralSet(P, rhs)

    m.P = ro.UncSet()
    m.P.cons = pe.ConstraintList()
    for i, row in enumerate(P):
        m.P.cons.add(expr=pe.quicksum(row[j]*m.r[j] for j in m.r) <= rhs[i])

    m.Obj = pe.Objective(expr=m.z, sense=pe.maximize)
    # x0 = m.x[0]
    # expr = x0*3
    expr = sum([m.x[i] for i in index]) == 1
    m.cons.add(expr)
    m.cons.add(sum([r[i]*m.x[i] for i in index]) >= m.z)

    return m


if __name__ == '__main__':
    m = Portfolio()
    solver = pe.SolverFactory('gurobi')
    solver = pe.SolverFactory('romodel.cuts')
    solver.solve(m)
