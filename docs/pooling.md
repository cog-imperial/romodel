## Pooling problem

The pooling problem is a well known case study in global optimization. The goal
is to maximize profit for given a number of quality constraints for flow
through a network of sources, intermediary pools, and product terminals. It has
applications in the oil and gas industry, water systems, supply chains, and
other areas. The pooling problem is known to be NP-hard due to bilinearities in
its formulation.

## Nominal version
The pooling problem is usually considered in its deterministic form with no
parametric uncertainty. Decision variables are the flows from sources to pools,
direct flows from sources to terminals,
and the fractions of flow from the pools to each product terminal. The problem
is constrained by feed availability at the sources, pool capacity, product
demand, and product quality. The product quality is the result of mixing of
sources with different feed qualities. For the full mathematical formulation of
the pooling problem see [our paper](https://arxiv.org/abs/1906.07612)
on the pooling problem or the [full code](../romodel/examples/pooling.py).

## Robust version
For a detailed discussion of robust optimization and the pooling problem please
see [our paper](https://arxiv.org/abs/1906.07612).
The robust version of the pooling problem included in ROmodel assumes that the
price at which each product can be sold is uncertain. 
The example includes the following uncertainty sets:

```python
# Ellipsoidal set constructed using the general approach:
m.U
# Ellipsoidal set constructed using the library approach:
m.Elib
# Polyhedral set constructed using the general approach:
m.P
# Polyhedral set constructed using the library approach:
m.Plib
# Convex set (neither ellipsoidal nor polyhedral):
m.C
```


## Running the example
The pooling problem example is available as part of ROmodel and can be used as
follows:

```python
import romodel.examples as ex
import pyomo.environ as pe

m = ex.Pooling()

solver = pe.SolverFactory('romodel.cuts')
solver.solve(m)
```

In order to use one of the other uncertainty sets from above use:

```python
m = ex.Pooling
m.price_product.uncset = m.C

solver = pe.SolverFactory('romodel.cuts')
solver.solve(m)
```

Note that the solving the pooling problem requires a solver which can solve
non-convex quadratically constrained problems, e.g. gurobi:

```python
solver = pe.SolverFactory('romodel.cuts')
solver.options['solver'] = 'gurobi'
# Allow non-convex problems
solver.options['NonConvex'] = 2
```

The full code is available [here](../romodel/examples/pooling.py).
