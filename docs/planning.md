## Production planning problem
This example is a simple production planning problem outlined in detail in [our
paper on (warped)-Gaussian process constrained optimization](https://arxiv.org/abs/2006.08222).
The main idea is to determine production targets as to maximize profit over a
number of time periods given a price at which the product can be sold and a
cost of production. We assume that he sales price depends on the amount
produced through an uncertain black-box function. Our paper outlines two
approaches for modeling this black-box function based on data and incorporating
it into the optimization problem:

1. Use a [standard Gaussian Process] as a stochastic model for the uncertain
   black-box function.

2. Use a [warped Gaussian Process] instead

We have implemented both approaches in ROmodel to make them more accessible.
The (warped) Gaussian Process can be trained using the [GPy
framework](https://sheffieldml.github.io/GPy/). ROmodel contains two dedicated
uncertainty sets, `GPSet` and `WarpedGPSet`, which take a GPY object as an
input and construct corresponding uncertainty sets using the approaches
outlined in [our paper](https://arxiv.org/abs/2006.08222).

## Formulation 
The only decision variables of the problem are the production amounts in each
time period. Parameters are the cost of production and the price at which the
product can be sold:

    # Number of planning periods
    T = 6
    # Construct model
    m = pe.ConcreteModel()
    # Decision variables
    m.x = pe.Var(range(T), within=pe.NonNegativeReals, bounds=(xmin, xmax))
    # Uncertainty set based on warped Gaussian process
    m.uncset = ro.uncset.WarpedGPSet(gp, m.x, alpha)
    # Uncertainty set based on standard Gaussian process
    m.uncset = ro.uncset.GPSet(gp, m.x, alpha)
    # Uncertain parameter
    m.demand = ro.UncParam(range(T), uncset=m.uncset, bounds=(0, 1))
    # Uncertain objective
    profit = sum(m.x[t]*m.demand[t] for t in range(T))
    profit -= sum(cost[t]*m.x[t] for t in range(T))
    m.u = pe.Var()
    m.profit = pe.Constraint(expr=profit >= m.u)
    m.Obj = pe.Objective(expr=m.u, sense=pe.maximize)

## Running the example
The production planning example is available as part of ROmodel and can be used
as follows:

    import romodel.examples as ex
    import pyomo.environ as pe
    
    m = ex.ProductionPlanning(alpha=0.9, warped=True)

    solver = pe.SolverFactory('romodel.reformulation')
    solver.options['solver'] = 'ipopt'
    solver.solve(m)

Parameter `alpha` determines the confidence with which the solution will be
feasible, while the `warped` parameter determines whether the standard or
warped Gaussian Process is used. Solving the problem requires a non-linear,
non-convex solver like [ipopt](https://github.com/coin-or/Ipopt).

