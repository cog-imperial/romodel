# ROmodel

ROmodel is a Python package which extends the modeling capabilities of
[Pyomo](https://github.com/Pyomo/pyomo) to robust optimization problems. It
also implements a number of algorithms for solving these problems.

## Install

To install ROmodel use:
        
        pip install https://github.com/johwiebe/romodel

## Getting started 

This introduction assumes basic familiarity with modeling optimization problems
in Pyomo. ROmodel introduces two main components for modeling robust optimization
problems: `UncParam` for modeling uncertain parameters and `UncSet` for
modeling uncertainty sets.

To start using ROmodel, import Pyomo and ROmodel:

    import pyomo.environ as pe
    import romodel as ro

Create a Pyomo model (ROmodel is currently only tested with ConcreteModel) and
some variables:

    m = pe.ConcreteModel()
    # Create an indexed variable
    m.x = pe.Var([0, 1])

Next, create an uncertainty set and some uncertain parameters:

    # Create a generic uncertainty set
    m.U = ro.UncSet()
    # Create an indexed uncertain parameter
    m.w = ro.UncParam([0, 1], uncset=m.U, nominal=[0.5, 0.8])

Uncertain constraints (or objectives) are created implicitly by using uncertain
parameters in constraint expressions:

    # Create an uncertain constraint 
    m.c = pe.Constraint(expr = m.x[0]*m.w[0] + m.x[1]*m.w[1] <= 1)
    m.o = pe.Objective(expr=m.x[0], sense=pe.maximize)

The robust model can be solved using one of ROmodels solvers:

    # Solve nominal problem
    solver = pe.SolverFacotry('romodel.nominal')
    solver.solve(m)
    # Solve robust problem using reformulation
    solver = pe.SolverFactory('romodel.reformulation')
    solver.solve(m)
    # Solve robust problem using cutting planes
    solver = pe.SolverFactory('romodel.cuts')
    solver.solve(m)

## Example problems
ROmodel includes a number of example problems:

1. [Knapsack problem](docs/knapsack.md)
2. [Portfolio optimiation problem]()
3. [Pooling problem]() (Nonlinear robust optimization)
4. [Facility location problem]() (Adjustable robust optimization)
5. [Production planning problem]() (Warped GP based uncertainty sets)

Further freely available examples of problems implemented in ROmodel include:

6. [State-Task-Network with degradation](https://github.com/johwiebe/stn)
7. [Drill scheduling problem](https://github.com/johwiebe/drilling)



## References & Funding
This work was funded by an EPSRC/Schlumberger CASE studentship to J.W.
(EP/R511961/1).
