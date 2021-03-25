## Facility location problem
We consider a [capacitated facility location
problem](https://en.wikipedia.org/wiki/Facility_location_problem) as an example
for adjustable robust optimization. The goal is to decide on a number of
facilities to build first and then determine from which facilities customers'
demand should be met.

# Nominal problem

    ```python
    # Define parameters
    N = 4
    M = 5
    cost_facility = [100, 140, 80, 150]
    cost_transport = [[2.0, 3.0, 1.5, 4.0, 3.3],
                      [1.5, 0.4, 1.0, 2.2, 3.5],
                      [3.4, 1.4, 2.1, 3.3, 1.6],
                      [2.6, 2.1, 2.0, 1.4, 1.3]]
    demand = [45, 30, 60, 55, 25]
    max_dem = [100, 140, 80, 150]

    # Construct model
    m = pe.ConcreteModel()
    # Define variables
    m.x = pe.Var(range(N), within=pe.Binary)
    m.y = pe.Var(range(N), range(M), within=pe.NonNegativeReals)
    # Add objective
    expr = 0
    for i in range(N):
        for j in range(M):
            expr += cost_transport[i][j]*m.y[i, j]
        expr += cost_facility[i]*m.x[i]
    m.obj = pe.Objective(expr=expr, sense=pe.minimize)

    # Add constraints
    def sum_y_rule(m, j):
        return sum(m.y[i, j] for i in range(N)) == m.demand[j]
    m.sum_y = pe.Constraint(range(M), rule=sum_y_rule)

    def max_demand_rule(m, i):
        lhs = sum(m.y[i, j] for j in range(M))
        return lhs <= max_dem[i]*m.x[i]
    m.max_dem = pe.Constraint(range(N), rule=max_demand_rule)
    ```

# Robust problem

    # Construct model
    m = pe.ConcreteModel()
    # Define variables
    m.x = pe.Var(range(N), within=pe.Binary)
    # Construct uncertainty set
    m.uncset = ro.UncSet()
    m.uncset.cons = pe.ConstraintList()
    # Define uncertain parameter
    m.demand = ro.UncParam(range(M), nominal=demand, uncset=m.uncset)
    # Define adjustable variables
    m.y = ro.AdjustableVar(range(N), range(M), bounds=(0, None), uncparams=[m.demand])
    # Define box uncertainty set
    for i in range(M):
        m.uncset.cons.add(expr=pe.inequality(0.9*demand[i], m.demand[i], 1.1*demand[i]))
    # Add objective
    expr = 0
    for i in range(N):
        for j in range(M):
            expr += cost_transport[i][j]*m.y[i, j]
        expr += cost_facility[i]*m.x[i]
    m.obj = pe.Objective(expr=expr, sense=pe.minimize)

    # Add constraints
    def sum_y_rule(m, j):
        return sum(m.y[i, j] for i in range(N)) == m.demand[j]
    m.sum_y = pe.Constraint(range(M), rule=sum_y_rule)

    def max_demand_rule(m, i):
        lhs = sum(m.y[i, j] for j in range(M))
        return lhs <= max_dem[i]*m.x[i]
    m.max_dem = pe.Constraint(range(N), rule=max_demand_rule)

# Running the example
The facility location problem is available as part of ROmodel and can be used
as follows:

    import romodel.examples as ex
    import pyomo.environ as pe

    m = ex.Facility()

    solver = pe.SolverFactory('romodel.reformulation')
    solver.solve(m)
