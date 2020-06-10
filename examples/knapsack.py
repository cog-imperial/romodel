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
from pro import UncSet, UncParam
from pro.uncset import EllipsoidalSet, PolyhedralSet
from pro.reformulate import PolyhedralTransformation
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
rhs = [sum(p[t]*w[t]  for t in tools) + 5.5 for p in P]


# # It should be easy to exchange one uncertainty set for another
# # OPTION 1: w is component of U
# M.U = UncSet()
# M.U.w = UncParam(M.ITEMS)
# w = M.U.w
# OPTION 2: w is component of M and gets U as argument
M.U = UncSet()
M.Ulib = EllipsoidalSet(mu, Adict)
M.w = UncParam(M.ITEMS, uncset=M.U)
# M.w.set_uncset(M.U2) # Update uncertainty set like this
w = M.w
# # OPTION 3: w is component of M, connection between w and U is implicit
# M.U = UncSet()
# M.w = UncParam(M.ITEMS)
# w = M.w
# 
# # OPTION 4: U gets w as argument
# # Let's got with this one for starters
# M.w = UncParam(M.ITEMS)
# M.U = UncSet(param=M.w)
# M.U2 = UncSet(param=M.w)
# w = M.w
# 
# # # OPTION 5: no UncParam
# M.U = UncSet(M.ITEMS)
# w = M.U
# # or something like
# w = M.U.get_params()

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
# M.w._uncset = M.P

M.value = Objective(expr=sum(v[i]*M.x[i] for i in M.ITEMS), sense=maximize)
M.weight = Constraint(expr=sum(w[i]*M.x[i] for i in M.ITEMS) <= limit)

t = PolyhedralTransformation()
t.apply_to(M)
solver = SolverFactory('gurobi')
solver.solve(M)
M.x.pprint()
