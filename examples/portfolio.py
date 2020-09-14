import pyomo.environ as pe
import romodel as ro


m = pe.ConcreteModel()
m.cons = pe.ConstraintList()

N = 5
index = list(range(N))
mean = [0.1, 0.3, 0.5, 0.7, 0.4]
# mean = [0.1, 0.3, 0.5, 0.7, 0.4]
# P = np.matrix([])
# U = pro.uncset.Polyhedral(I, mean, P)
# m.r = pro.UncertainParam(I, uncset=U)

m.x = pe.Var(index, bounds=(0, 1))
m.z = pe.Var(within=pe.PositiveReals)

m.U = ro.UncSet()
m.r = ro.UncParam(index, uncset=m.U)
for i in index:
    m.r[i].value = mean[i]
r = m.r
expr = 0
for i in index:
    expr += (m.r[i] - mean[i])**2
m.U.cons = pe.Constraint(expr=expr <= 0.0005)

m.Obj = pe.Objective(expr=m.z, sense=pe.maximize)
x0 = m.x[0]
expr = x0*3
expr = sum([m.x[i] for i in index]) == 1
m.cons.add(expr)
m.cons.add(-sum([r[i]*m.x[i] for i in index]) <= -m.z)

solver = pe.SolverFactory('gurobi')
solver = pe.SolverFactory('romodel.robust.cuts')
solver.solve(m)
