# Portfolio example
The goal in portfolio optimization is to maximize profit by choosing a number
of assets in which to invest a given amount of money.

## Nominal version
Parameters of the problem are the returns `r` for each asset. Each
variable `x[i]` indicates the fraction of the available money which is invested
in asset `i`. The only constraint ensures that all of the money is invested.

```python
m = pe.ConcreteModel()
m.cons = pe.ConstraintList()

r = [0.1, 0.3, 0.5, 0.7, 0.4]

m.x = pe.Var(range(5), bounds=(0, 1))

m.cons = pe.Constraint(expr=sum([m.x[i] for i in index]) == 1)
m.obj = pe.Objective(expr=sum([r[i]*m.x[i] for i in index]),
                     sense=pe.maximize)
```


## Robust version
Future returns are usually not known explicitly. It therefore makes sense to
consider `r` to be uncertain and to optimize the worst case profit.

```python
m = pe.ConcreteModel()
m.cons = pe.ConstraintList()

r = [0.1, 0.3, 0.5, 0.7, 0.4]

m.x = pe.Var(range(5), bounds=(0, 1))

# Construct uncertainty set and uncertain parameter
m.U = ro.uncset.EllipsoidalSet(r,
                               [[0.0005, 0, 0, 0, 0],
                               [0, 0.0005, 0, 0, 0],
                               [0, 0, 0.0005, 0, 0],
                               [0, 0, 0, 0.0005, 0],
                               [0, 0, 0, 0, 0.0005]])
m.r = ro.UncParam(range(5), uncset=m.U, nominal=r)

# Add constraint and objective
m.cons = pe.Constraint(expr=sum([m.x[i] for i in index]) == 1)
m.obj = pe.Objective(expr=sum([m.r[i]*m.x[i] for i in index]),
                     sense=pe.maximize)
```

## Running the example
The portfolio optimization example is available as part of ROmodel and can be
used as follows:

```python
import romodel.examples as ex
import pyomo.environ as pe

m = ex.Portfolio()

solver = pe.SolverFactory('romodel.cuts')
solver.solve(m)
```

The full code is available [here](../romodel/examples/portfolio.py).
