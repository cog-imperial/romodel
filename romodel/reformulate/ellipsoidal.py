from pyomo.environ import Constraint, Var, quicksum, sqrt, Objective
from romodel.reformulate import BaseRobustTransformation
from pyomo.core import TransformationFactory
from itertools import chain
from romodel.util import collect_uncparam


@TransformationFactory.register('romodel.ellipsoidal',
                                doc="Ellipsoidal Counterpart")
class EllipsoidalTransformation(BaseRobustTransformation):
    def _apply_to(self, instance, root=False):
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
                    name = c.name + '_counterpart_upper'
                    if root:
                        expr = det + sqrt(padding) <= c.upper
                        counterpart = Constraint(expr=expr)
                    else:
                        setattr(instance,
                                c.name + '_padding',
                                Var(bounds=(0, float('inf'))))
                        pvar = getattr(instance, c.name + '_padding')
                        counterpart = Constraint(expr=det + pvar <= c.upper())
                        deterministic = Constraint(expr=padding <= pvar**2)
                        setattr(instance, c.name + '_det_upper', deterministic)
                    setattr(instance, name, counterpart)
                # For lower bound: det - padding >= b
                if c.has_lb():
                    name = c.name + '_counterpart_lower'
                    if root:
                        expr = det - sqrt(padding) >= c.lower
                        counterpart = Constraint(expr=expr)
                    else:
                        setattr(instance,
                                c.name + '_padding',
                                Var(bounds=(0, float('inf'))))
                        pvar = getattr(instance, c.name + '_padding')
                        counterpart = Constraint(expr=c.lower() <= det - pvar)
                        deterministic = Constraint(expr=padding <= pvar**2)
                        setattr(instance, c.name + '_det_lower', deterministic)
                    setattr(instance, name, counterpart)
            else:
                # For minimization: min det + padding
                # For maximization: max det - padding
                sense = c.sense
                name = c.name + '_counterpart'
                if root:
                    expr = det + c.sense*sqrt(padding)
                    counterpart = Objective(expr=expr, sense=sense)
                else:
                    setattr(instance,
                            c.name + '_padding',
                            Var(bounds=(0, float('inf'))))
                    pvar = getattr(instance, c.name + '_padding')
                    counterpart = Objective(expr=det + sense*pvar, sense=sense)
                    deterministic = Constraint(expr=padding <= pvar**2)
                    setattr(instance,
                            c.name + '_det',
                            deterministic)
                setattr(instance, name, counterpart)
            c.deactivate()
