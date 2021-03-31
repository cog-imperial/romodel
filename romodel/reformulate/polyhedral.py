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
from romodel.reformulate import BaseRobustTransformation
from pyomo.repn import generate_standard_repn
from romodel.uncset import UncSet, PolyhedralSet
from romodel.util import collect_uncparam


@TransformationFactory.register('romodel.polyhedral',
                                doc="Polyhedral Counterpart")
class PolyhedralTransformation(BaseRobustTransformation):
    def _check_applicability(self, uncset):
        """
        Returns `True` if the reformulation is applicable to `uncset`

            uncset: UncSet

        """
        # Check for library set
        if uncset.__class__ == PolyhedralSet:
            return True
        # Check generic set:
        elif uncset.__class__ == UncSet:
            mat = []
            rhs = []
            for c in uncset.component_data_objects(Constraint, active=True):
                # Generate standard repn
                repn = generate_standard_repn(c.body)
                param = collect_uncparam(c)
                # If uncertainty set contains a non-linear constraint it's not
                # polyhedral.
                if not repn.is_linear():
                    return False
                coef_dict = {id(x): y for x, y in zip(repn.linear_vars,
                                                      repn.linear_coefs)}
                if c.has_ub():
                    mat.append([coef_dict.get(id(param[i]), 0) for i in param])
                    rhs.append(c.upper - repn.constant)
                if c.has_lb():
                    mat.append([-coef_dict.get(id(param[i]), 0) for i in param])
                    rhs.append(repn.constant - c.lower)

            uncset.mat = mat
            uncset.rhs = rhs

            return True

        return False

    def _check_constraint(self, c):
        """
        Raise an error if the constraint is inappropriate for this
        reformulation

            c: Constraint

        """
        assert not c.equality, (
                "Currently can't handle equality constraints yet.")

    def _reformulate(self, c, param, uncset, counterpart, pao=False):
        """
        Reformulate an uncertain constraint or objective

            c: Constraint or Objective
            param: UncParam
            uncset: UncSet
            counterpart: Block

        """

        repn = self.generate_repn_param(c)
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
                counterpart.upper = dual
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
                counterpart.lower = dual
        # Objective
        else:
            counterpart.epigraph = Var()
            epigraph = counterpart.epigraph
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
            counterpart.dual = dual
            counterpart.obj = Objective(expr=epigraph, sense=sense)

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
