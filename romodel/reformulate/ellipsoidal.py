from pyomo.environ import Constraint, Var, quicksum, sqrt, Objective, Block
from romodel.reformulate import BaseRobustTransformation
from pyomo.core import TransformationFactory


@TransformationFactory.register('romodel.ellipsoidal',
                                doc="Ellipsoidal Counterpart")
class EllipsoidalTransformation(BaseRobustTransformation):
    def _check_applicability(self, c, param, uncset):
        return uncset.is_ellipsoidal()

    def _check_constraint(self, c):
        assert not c.equality, (
                "Currently can't handle equality constraints yet.")

    def _check_objective(self, o):
        pass

    def _reformulate(self, c, param, uncset, counterpart, root=False):
        # Check constraint/objective
        repn = self.generate_repn_param(c)
        assert repn.is_linear(), (
                "Constraint {} should be linear in "
                "unc. parameters".format(c.name))

        # Generate robust counterpart
        det = quicksum(x[0]*x[1].nominal for x in zip(repn.linear_coefs,
                                                      repn.linear_vars))
        det += repn.constant
        param_var_dict = {id(param): var
                          for param, var
                          in zip(repn.linear_vars, repn.linear_coefs)}
        # padding = sqrt( var^T * cov^-1 * var )
        padding = quicksum(param_var_dict[id(param[ind_i])]
                           * uncset.cov[i][j]
                           * param_var_dict[id(param[ind_j])]
                           for i, ind_i in enumerate(param)
                           for j, ind_j in enumerate(param))
        if c.ctype is Constraint:
            # For upper bound: det + padding <= b
            if c.has_ub():
                counterpart.upper = Block()
                if root:
                    expr = det + sqrt(padding) <= c.upper
                    robust = Constraint(expr=expr)
                else:
                    counterpart.upper.padding = Var(bounds=(0, float('inf')))
                    pvar = counterpart.upper.padding
                    robust = Constraint(expr=det + pvar <= c.upper())
                    deterministic = Constraint(expr=padding <= pvar**2)
                    counterpart.upper.det = deterministic
                counterpart.upper.rob = robust
            # For lower bound: det - padding >= b
            if c.has_lb():
                counterpart.lower = Block()
                if root:
                    expr = det - sqrt(padding) >= c.lower
                    robust = Constraint(expr=expr)
                else:
                    counterpart.lower.padding = Var(bounds=(0, float('inf')))
                    pvar = counterpart.lower.padding
                    robust = Constraint(expr=c.lower() <= det - pvar)
                    deterministic = Constraint(expr=padding <= pvar**2)
                    counterpart.lower.det = deterministic
                counterpart.lower.rob = robust
        else:
            # For minimization: min det + padding
            # For maximization: max det - padding
            sense = c.sense
            if root:
                expr = det + c.sense*sqrt(padding)
                robust = Objective(expr=expr, sense=sense)
            else:
                counterpart.padding = Var(bounds=(0, float('inf')))
                pvar = counterpart.padding
                robust = Objective(expr=det + sense*pvar, sense=sense)
                deterministic = Constraint(expr=padding <= pvar**2)
                counterpart.det = deterministic
            counterpart.rob = robust

