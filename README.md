# ROmodel

ROmodel is a Python package which extends the modeling capabilities of
[Pyomo](https://github.com/Pyomo/pyomo) to robust optimization problems. It
also implements a number of algorithms for solving these problems.

## Install

To install ROmodel use:

```bash        
pip install git+https://github.com/johwiebe/romodel.git
```

## Getting started 

This introduction assumes basic familiarity with modeling optimization problems
in Pyomo. ROmodel introduces three main components for modeling robust optimization
problems:

1. `UncSet` for modeling uncertainty sets
2. `UncParam` for modeling uncertain parameters 
3. `AdjustableVar` for modeling adjustable variables in adjustable robust
   optimization problems

To start using ROmodel, import Pyomo and ROmodel:

```python
import pyomo.environ as pe
import romodel as ro
```

Create a Pyomo model (ROmodel is currently only tested with ConcreteModel) and
some variables:

```python
m = pe.ConcreteModel()
# Create an indexed variable
m.x = pe.Var([0, 1])
```

Next, create an uncertainty set. Uncertainty sets can be created in one of two
ways: 

1. Using generic Pyomo constraints.
2. Choosing from a library of commonly used geometries.

### Generic uncertainty sets

To create a generic uncertainty set use the `UncSet` class. This class inherits
from Pyomo's `Block` class and can be used similarly. To define the uncertainty
set simply add constraints to the `UncSet` object:

```python
# Create a generic uncertainty set
m.U = ro.UncSet()
# Create an indexed uncertain parameter
m.w = ro.UncParam([0, 1], uncset=m.U, nominal=[0.5, 0.8])
# Add constraints to the uncertainty set
m.U.cons1 = pe.Constraint(m.w[0] + m.w[1] <= 1.5)
m.U.cons2 = pe.Constraint(m.w[0] - m.w[1] <= 1.5)
```

### Library uncertainty sets

ROmodel currently implements library versions of polyhedral uncertainty sets
(`P*w <= d`), ellipsoidal sets (`(w - mu)^T Cov^-1 (w - mu) <= r^2`), and
Gaussian Process-based uncertainty sets (see our [paper]() for more details).

Polyhedral sets can be constructed using their matrix representation with the
`PolyhedralSet` class:

```python
from romodel.uncset import PolyhedralSet
# Define polyhedral set
m.U = PolyhedralSet(mat=[[ 1,  1],
                         [ 1, -1],
                         [-1,  1],
                         [-1, -1]],
                    rhs=[1, 1, 1, 1])
```

Similarly, ellipsoidal sets can be constructed from a covariance matrix and a
mean vector using the `EllipsoidalSet` class:

```python
from romodel.uncset import EllipsoidalSet
# Define ellipsoidal set
m.U = EllipsoidalSet(cov=[[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]],
                     mean=[0.5, 0.3, 0.1])
```


### Creating uncertain parameters and constraints

Uncertain parameters can be created using the `UncParam` class. They are
similar to Pyomo's `Param` modeling object and take a `nominal` argument, which
specifies the nominal values, and an 'uncset' object, which specifies which
uncertainty set to use.  Uncertain constraints (or objectives) are created
implicitly by using uncertain
parameters in constraint expressions. As an example, consider the following
deterministic constraint:

```python
# deterministic
m.x = pe.Var(range(3))
c = [0.1, 0.2, 0.3]
m.cons = pe.Constraint(expr=sum(c[i]*m.x[i] for i in m.x) <= 0)
```

If the coefficients `c` are uncertain, we can model the robust constraint as:

```python
# robust
m.x = pe.Var(range(3))
m.c = ro.UncParam(range(3), nominal=[0.1, 0.2, 0.3], uncset=m.U)
m.cons = pe.Constraint(expr=sum(m.c[i]*m.x[i] for i in m.x) <= 0)
```

### Adjustable robust optimization
ROmodel also has capabilities for modeling adjustable variables for adjustable
robust optimization. Defining an adjustable variable is analogous to defining a
regular variable in Pyomo, with an additional `uncparam` argument
specifying a list of uncertain parameters which the adjustable variable depends
on:

```python
# Define uncertain parameters
m.w = ro.UncParam(range(3), nominal=[1, 2, 3])
# Define adjustable variable which depends on uncertain parameter
m.y = ro.AdjustableVar(range(3), uncparams=[m.w], bounds=(0, 1))
```

The uncertain parameters can also be set individually for each element of the
adjustable variables index using the `set_uncparams` function:

```python
# Set uncertain parameters for individual indicies
m.y[0].set_uncparams([m.w[0]])
m.y[1].set_uncparams([m.w[0], m.w[1]])
```

The only method currently implemented for solving adjustable robust
optimization problems in ROmodel is linear decision rules. If a model contains
adjustable variables in a constraint or objective, ROmodel automatically
replaces it by a linear decision rule based on the specified uncertain
parameters.


### Solvers
Robust optimization problems modeled in ROmodel can be solved using one of
three solvers:
    
1. Reformulation solver: this solver applies duality based reformulations to
   the robust problem to generate it's deterministic counterpart. ROmodel
   currently includes reformulations for ellipsoidal, polyhedral, and Gaussian
   process-based uncertainty sets.
2. Cutting plane solver: this solver starts by solving the nominal problem and
   then iteratively adds uncertainty scenarios which violate the robust
   constraints until all constraints are robustly feasible. The cutting plane
   solver can be applied to any uncertainty set geometry which can be solved
   with one of the solvers available through Pyomo.
3. Nominal solver: this solver simply replaces each uncertain parameter by its
   nominal value and solves the nominal problem. This solver is included for
   convenience.

ROmodel solvers can be instantiated using Pyomo's `SolverFactory`:
```python
# Solve robust problem using reformulation
solver = pe.SolverFactory('romodel.reformulation')
solver.solve(m)
# Solve robust problem using cutting planes
solver = pe.SolverFactory('romodel.cuts')
solver.solve(m)
# Solve nominal problem
solver = pe.SolverFactory('romodel.nominal')
solver.solve(m)
```

## Example problems
ROmodel includes a number of example problems:

1. [Knapsack problem](docs/knapsack.md)
2. [Portfolio optimiation problem](docs/portfolio.md)
3. [Pooling problem](docs/pooling.md) (Nonlinear robust optimization)
4. [Facility location problem](docs/facility.md) (Adjustable robust optimization)
5. [Production planning problem](docs/planning.md) (Warped GP based uncertainty sets)

Further freely available examples of problems implemented in ROmodel include:

6. [State-Task-Network with degradation](https://github.com/johwiebe/stn)
7. [Drill scheduling problem](https://github.com/johwiebe/drilling)



## References & Funding
This work was funded by an EPSRC/Schlumberger CASE studentship to J.W.
(EP/R511961/1).
