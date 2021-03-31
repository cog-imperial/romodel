from pyomo.environ import Constraint, Var, quicksum, sqrt, Objective, Block
from romodel.reformulate import BaseRobustTransformation
from pyomo.core import TransformationFactory
from itertools import chain
from romodel.util import collect_uncparam


@TransformationFactory.register('romodel.ellipsoidal',
                                doc="Ellipsoidal Counterpart")
class EllipsoidalTransformation(BaseRobustTransformation):
    def _reformulate(self, instance, c, param, uncset, counterpart, root=False):
        # Check constraint/objective
        if c.ctype is Constraint:
            assert not c.equality, (
                    "Currently can't handle equality constraints yet.")
        repn = self.generate_repn_param(instance, c)
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

    def _apply_to(self, instance, **kwargs):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            # Collect unc. parameter and unc. set
            param = collect_uncparam(c)
            uncset = param._uncset
            assert uncset is not None, ("No uncertainty set provided for "
                                        "uncertain parameter {}."
                                        .format(param.name))

            # Check if uncertainty set is empty
            assert not uncset.is_empty(), ("{} does not have any "
                                           "constraints.".format(uncset.name))
            # Check if uncertainty set is ellipsoidal
            if not uncset.is_ellipsoidal():
                continue

            counterpart = Block()
            setattr(instance, c.name + '_counterpart', counterpart)
            self._reformulate(instance, c, param, uncset, counterpart, **kwargs)

            c.deactivate()
