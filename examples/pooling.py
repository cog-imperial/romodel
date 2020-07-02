import pyomo.environ as pe
import pro

feeds = range(5)
products = range(4)
pools = range(2)
qualities = range(4)

con_feed_pool = [(0, 0), (1, 0), (2, 0), (3, 1), (4, 1)]
con_pool_prod = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3)]
con_feed_prod = []

price_product = [16, 25, 15, 10]
price_feed = [7, 3, 2, 10, 5]

max_flow = [float('inf'), float('inf'), float('inf'), float('inf'), float('inf')]
min_flow = [0, 0, 0, 0, 0]
pool_size = [float('inf'), float('inf')]
max_demand = [10, 25, 30, 10]
min_demand = [0, 0, 0, 0]
feed_cons = [[1.0, 6.0, 4.0, 0.5],
             [4.0, 1.0, 3.0, 2.0],
             [4.0, 5.5, 3.0, 0.9],
             [3.0, 3.0, 3.0, 1.0],
             [1.0, 2.7, 4.0, 1.6]]

max_cons = [[3.00, 3.00, 3.25, 0.75],
            [4.00, 2.50, 3.50, 1.50],
            [1.50, 5.50, 3.90, 0.80],
            [3.00, 4.00, 4.00, 1.80]]
min_cons = [[0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]]

m = pe.ConcreteModel()
m.q = pe.Var(con_feed_pool, bounds=(0, 1))
m.y = pe.Var(con_pool_prod, within=pe.NonNegativeReals)
m.z = pe.Var(con_feed_prod, within=pe.NonNegativeReals)

m.U = pro.UncSet()
m.price_product = pro.UncParam(products, nominal=price_product, uncset=m.U)
expr = 0
for j in products:
    expr += (m.price_product[j] - price_product[j])**2
m.U.c = pe.Constraint(expr=expr <= 0.1)
price_product = m.price_product

obj = 0
for i, l in con_feed_pool:
    for j in [jj for ll, jj in con_pool_prod if ll == l]:
        obj += price_feed[j]*m.y[(l, j)]*m.q[i, l]

for l, j in con_pool_prod:
    obj -= price_product[j]*m.y[(l, j)]

for i, j in con_feed_prod:
    obj -= (price_product[j] - price_feed[i])*m.z[(i, j)]
m.obj = pe.Objective(expr=obj, sense=pe.minimize)


# Feed availability
def feed_availability_rule(m, i):
    expr = 0
    for l in [ll for ii, ll in con_feed_pool if ii == i]:
        for j in [jj for ll, jj in con_pool_prod if ll == l]:
            expr += m.q[(i, l)]*m.y[(l, j)]
    for j in [jj for ii, jj in con_feed_prod if ii == i]:
        expr += m.z[(i, l)]
    return min_flow[i], expr,  max_flow[i]


m.feed_availability = pe.Constraint(feeds, rule=feed_availability_rule)


# Pool capacity
def pool_capacity_rule(m, l):
    expr = 0
    for j in [jj for ll, jj in con_pool_prod if ll == l]:
        expr += m.y[(l, j)]
    return None, expr, pool_size[l]


m.pool_capacity = pe.Constraint(pools, rule=pool_capacity_rule)


# Product demand
def prod_demand_rule(m, j):
    expr = 0
    for l in [ll for ll, jj in con_pool_prod if jj == j]:
        expr += m.y[(l, j)]
    for i in [ii for ii, jj in con_feed_prod if jj == j]:
        expr += m.z[(i, j)]
    return min_demand[j], expr, max_demand[j]


m.product_demand = pe.Constraint(products, rule=prod_demand_rule)


# Simplex
def simplex_rule(m, l):
    return pe.quicksum(m.q[(i, l)] for i, ll in m.q if ll == l) == 1


m.simplex = pe.Constraint(pools, rule=simplex_rule)


# Product quality
def prod_quality_rule_upper(m, j, k):
    expr = 0
    flow = 0
    for l in [ll for ll, jj in con_pool_prod if jj == j]:
        flow += m.y[l, j]
        for i in [ii for ii, ll in con_feed_pool if ll == l]:
            expr += feed_cons[i][k]*m.q[(i, l)]*m.y[(l, j)]
    for i in [ii for ii, jj in con_feed_prod if jj == j]:
        flow += m.z[i, j]
        expr += feed_cons[i][k]*m.z[(i, j)]
    return expr <= max_cons[j][k]*flow


def prod_quality_rule_lower(m, j, k):
    expr = 0
    flow = 0
    for l in [ll for ll, jj in con_pool_prod if jj == j]:
        flow += m.y[l, j]
        for i in [ii for ii, ll in con_feed_pool if ll == l]:
            expr += feed_cons[i][k]*m.q[(i, l)]*m.y[(l, j)]
    for i in [ii for ii, jj in con_feed_prod if jj == j]:
        flow += m.z[i, j]
        expr += feed_cons[i][k]*m.z[(i, j)]
    return min_cons[j][k]*flow <= expr


m.prod_quality_upper = pe.Constraint(products, qualities,
                                     rule=prod_quality_rule_upper)
m.prod_quality_lower = pe.Constraint(products, qualities,
                                     rule=prod_quality_rule_lower)

solver = pe.SolverFactory('pro.robust.cuts')
# solver = pe.SolverFactory('gurobi')
solver.options['NonConvex'] = 2
solver.solve(m, tee=True)
