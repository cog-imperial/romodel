#  ___________________________________________________________________________
#
#  Pyomo: Python Optimization Modeling Objects
#  Copyright 2017 National Technology and Engineering Solutions of Sandia, LLC
#  Under the terms of Contract DE-NA0003525 with National Technology and
#  Engineering Solutions of Sandia, LLC, the U.S. Government retains certain
#  rights in this software.
#  This software is distributed under the 3-clause BSD License.
#  ___________________________________________________________________________

#
# Robust Knapsack Problem
#

from pyomo.environ import ConcreteModel, Set, Binary, Var, Constraint
from pyomo.environ import Objective, maximize, SolverFactory, ConstraintList
from romodel import UncSet, UncParam
from romodel.uncset import EllipsoidalSet, PolyhedralSet
import itertools


v = {'hammer': 8, 'wrench': 3, 'screwdriver': 6, 'towel': 11}
w = {'hammer': 5, 'wrench': 7, 'screwdriver': 4, 'towel': 3}

limit = 14

M = ConcreteModel()

M.ITEMS = Set(initialize=v.keys())
M.x = Var(M.ITEMS, within=Binary)

# Define Uncertainty set & uncertain parameters
mu = w
A = [[0.1, 0.01, 0.0, 0.0],
     [0.01, 0.1, 0.0, 0.0],
     [0.0, 0.0, 0.1, 0.0],
     [0.0, 0.0, 0.0, 0.1]]
tools = ['hammer', 'wrench', 'screwdriver', 'towel']
Adict = {(ti, tj): A[i][j]
         for i, ti in enumerate(tools)
         for j, tj in enumerate(tools)}
perm = itertools.product([1, -1], repeat=len(tools))
P = [dict(zip(tools, i)) for i in perm]
rhs = [sum(p[t]*w[t] for t in tools) + 5.5 for p in P]


M.U = UncSet()
M.Ulib = EllipsoidalSet(mu, Adict)
M.w = UncParam(M.ITEMS, uncset=M.U)
w = M.w

# Ellipsoidal uncertainty set direct: (w - mu)^T * A * (w - mu) <= 1
lhs = 0
for i in M.ITEMS:
    for j in M.ITEMS:
        lhs += (w[i] - mu[i])*Adict[i, j]*(w[j] - mu[j])
M.U.cons = Constraint(expr=lhs <= 1)

# Polyhedral set:
# direct
M.P = UncSet()
M.P.cons = ConstraintList()
for i, p in enumerate(P):
    M.P.cons.add(sum(M.w[t]*p[t] for t in tools) <= rhs[i])
# library
M.Plib = PolyhedralSet(P, rhs)

M.value = Objective(expr=sum(v[i]*M.x[i] for i in M.ITEMS), sense=maximize)
M.weight = Constraint(expr=sum(w[i]*M.x[i] for i in M.ITEMS) <= limit)

# Set UncSet
M.w.uncset = M.P

solver = SolverFactory('romodel.reformulation')
solver.solve(M, options={'solver': 'gurobi'})
M.pprint()
