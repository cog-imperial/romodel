# Knapsack example

In a [Knapsack problem](https://en.wikipedia.org/wiki/Knapsack_problem), the
goal is to choose from a list of items, each of which has a given weight and
value, such that the total weight of the chosen items is below some limit and
the total value is as large as possible.

## The nominal problem
We use a simple Knapsack implementation from the Pyomo [example library](https://github.com/Pyomo/pyomo/blob/main/examples/pyomo/concrete/knapsack-concrete.py). Parameters of the problem are the value `v` and weight `w` of each item and the upper limit for the weight. The problem has a simple objective and a single constraint.

```python
tools = ['hammer', 'wrench', 'screwdriver', 'towel']
v = {'hammer': 8, 'wrench': 3, 'screwdriver': 6, 'towel': 11}
w = {'hammer': 5, 'wrench': 7, 'screwdriver': 4, 'towel': 3}
limit = 14

m = ConcreteModel()

m.x = Var(tools, within=Binary)

# Define objective and constraint
m.value = Objective(expr=sum(v[i]*m.x[i] for i in tools), sense=maximize)
m.weight = Constraint(expr=sum(m.w[i]*m.x[i] for i in tools) <= limit)
```

## Robust version
Most Knapsack implementations assume that the parameters are known precisely.
If this is not the case, the suggested solution may become infeasible. If the
weights are not known exactly, a solution which is just under the weight limit
in theory may end up above the limit in practice. We consider a robust version
of the Knapsack problem with uncertain weights:

```python
tools = ['hammer', 'wrench', 'screwdriver', 'towel']
v = {'hammer': 8, 'wrench': 3, 'screwdriver': 6, 'towel': 11}
w = {'hammer': 5, 'wrench': 7, 'screwdriver': 4, 'towel': 3}
limit = 14

m = ConcreteModel()

m.x = Var(tools, within=Binary)

# Define uncertainty set
m.P = UncSet()
# Define uncertain parameter
m.w = UncParam(tools, uncset=m.P, nominal=w)

# Define objective and constraint
m.value = Objective(expr=sum(v[i]*m.x[i] for i in tools), sense=maximize)
m.weight = Constraint(expr=sum(m.w[i]*m.x[i] for i in tools) <= limit)

# Define Polyhedral uncertainty set ( U = {w | Pw <= rhs} )
perm = itertools.product([1, -1], repeat=len(tools))
P = [i for i in perm]
rhs = [sum(p[i]*w[t] for i, t in enumerate(tools)) + 5.5 for p in P]
M.P.cons = ConstraintList()
for i, p in enumerate(P):
    M.P.cons.add(sum(M.w[t]*p[i] for i, t in enumerate(tools)) <= rhs[i])
```

A total of 8 lines of code allow us to transform the nominal problem into a
robust problem with a polyhedral uncertainty set.

## Running the example
The Knapsack example is available as part of ROmodel and can be used as
follows:

    import romodel.examples as ex
    import pyomo.environ as pe

    m = ex.Knapsack()

    solver = pe.SolverFactory('romodel.reformulation')
    solver.solve(m)

The default uncertainty set is an ellipsoidal set, but you can switch to a
polyhedral set using:

    m.w.uncset = m.P

Or define your own set:

    m.custom_uncset = ...
    m.w.uncset = m.custom_uncset

The full code is available [here](../romodel/examples/knapsack.py).
