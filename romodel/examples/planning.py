import pyomo.environ as pe
import romodel as ro
import numpy as np


T = 3
xmin = 0.1
xmax = 6
cost = [0.1, 0.05, 0.01]  # , 0.2, 0.1, 0.15, 0.04, 0.03, 0.1, 0.11, 0.25, 0.1]


def generate_data(N, noise):
    np.random.seed(5)
    x = np.random.uniform(size=(N, 1))*6
    y = np.exp(-x/2) + np.random.normal(scale=noise*4*np.exp(-x/2),
                                        size=(N, 1))
    return x, y


def ProductionPlanning(alpha=0.90):
    import GPy
    import rogp
    # Train warped GP
    x, y = generate_data(50, 0.05)
    kernel = GPy.kern.RBF(input_dim=1, variance=1., lengthscale=1.)
    gp = GPy.models.WarpedGP(x, y, kernel=kernel, warping_terms=3)
    gp.optimize(messages=True)
    # Make Pyomo
    gp = rogp.from_gpy(gp)
    # Pyomo model
    m = pe.ConcreteModel()
    m.x = pe.Var(range(T), within=pe.NonNegativeReals, bounds=(xmin, xmax))
    for i in m.x:
        m.x[i].value = (xmin + xmax)/2
    # Uncertainty set
    m.uncset = ro.uncset.WarpedGPSet(gp, m.x, alpha)
    # Uncertain parameter
    m.demand = ro.UncParam(range(T), uncset=m.uncset)
    # Uncertain objective
    profit = sum(m.x[t]*m.demand[t] for t in range(T))
    profit -= sum(cost[t]*m.x[t] for t in range(T))
    m.Obj = pe.Objective(expr=profit, sense=pe.maximize)

    return m
