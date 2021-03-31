from pyomo.environ import (Constraint,
                           Var,
                           quicksum,
                           Objective,
                           maximize,
                           minimize,
                           ConstraintList,
                           NonNegativeReals,
                           Block,
                           native_numeric_types)
from pyomo.core import TransformationFactory
from romodel.duality import create_linear_dual_from
from itertools import chain
from romodel.util import collect_uncparam
from romodel.reformulate import BaseRobustTransformation


@TransformationFactory.register('romodel.polyhedral',
                                doc="Polyhedral Counterpart")
class PolyhedralTransformation(BaseRobustTransformation):
    def _apply_to(self, instance, pao=False):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            # Collect UncParam and UncSet
            param = collect_uncparam(c)
            uncset = param.uncset
            # Check if uncertainty set is empty
            assert not uncset.is_empty(), ("{} does not have any "
                                           "constraints.".format(uncset.name))
            # Check if uncertainty set is polyhedral
            if not uncset.is_polyhedral():
                continue

            if c.ctype is Constraint:
                assert not c.equality, (
                        "Currently can't handle equality constraints yet.")
            repn = self.generate_repn_param(instance, c)
            assert repn.is_linear(), (
                    "Constraint {} should be linear in "
                    "unc. parameters".format(c.name))

            # Get coefficients
            id_coef_dict = {id(repn.linear_vars[i]):
                            repn.linear_coefs[i]
                            for i in range(len(repn.linear_vars))}
            c_coefs = [id_coef_dict.get(id(i), 0) for i in param.values()]
            cons = repn.constant

            # Add dual constraints d^T * v <= b, P^T * v = x
            # Constraint
            if c.ctype is Constraint:
                # LEQ
                if c.has_ub():
                    # Create linear dual
                    if pao:
                        uncset.obj = Objective(expr=c.body, sense=maximize)
                        dual = create_linear_dual_from(uncset,
                                                       unfixed=param.values())
                        o_expr = dual.o.expr
                        del dual.o
                        dual.o = Constraint(expr=o_expr <= c.upper)
                        del uncset.obj
                    else:
                        dual = self.create_linear_dual(c_coefs,
                                                       c.upper - cons,
                                                       uncset.mat,
                                                       uncset.rhs)
                    setattr(instance, c.name + '_counterpart_upper', dual)
                # GEQ
                if c.has_lb():
                    # Create linear dual
                    if pao:
                        uncset.obj = Objective(expr=c.body, sense=minimize)
                        dual = create_linear_dual_from(uncset,
                                                       unfixed=param.values())
                        o_expr = dual.o.expr
                        del dual.o
                        dual.o = Constraint(expr=c.lower <= o_expr)
                        del uncset.obj
                    else:
                        dual = self.create_linear_dual([-1*c for c in c_coefs],
                                                       -1*(c.lower - cons),
                                                       uncset.mat,
                                                       uncset.rhs)
                    setattr(instance, c.name + '_counterpart_lower', dual)
            # Objective
            else:
                setattr(instance, c.name + '_epigraph', Var())
                epigraph = getattr(instance, c.name + '_epigraph')
                sense = c.sense
                # Create linear dual
                if pao:
                    uncset.obj = Objective(expr=c.expr, sense=-sense)
                    dual = create_linear_dual_from(uncset,
                                                   unfixed=param.values())
                    o_expr = dual.o.expr
                    del dual.o
                    dual.o = Constraint(expr=sense*o_expr <= sense*epigraph)
                    del uncset.obj
                else:
                    dual = self.create_linear_dual([sense*c for c in c_coefs],
                                                   sense*(epigraph - cons),
                                                   uncset.mat,
                                                   uncset.rhs)

                setattr(instance, c.name + '_counterpart', dual)
                setattr(instance,
                        c.name + '_new',
                        Objective(expr=epigraph, sense=sense))
            # Deactivate original constraint
            c.deactivate()

    def create_linear_dual(self, c, b, P, d):
        '''
        Robust constraint:
            c^T*w <= b for all P*w <= d
        '''
        blk = Block()
        blk.construct()
        n, m = len(P), len(P[0])
        # Add dual variables
        blk.var = Var(range(n), within=NonNegativeReals)
        # Dual objective
        blk.obj = Constraint(expr=quicksum(d[j]*blk.var[j]
                                           for j in range(n)) <= b)
        # Dual constraints
        blk.cons = ConstraintList()
        for i in range(m):
            lhs = quicksum(P[j][i]*blk.var[j] for j in range(n))
            if lhs.__class__ not in native_numeric_types or lhs != 0:
                blk.cons.add(lhs == c[i])

        return blk
