from pyomo.environ import Constraint, Var, quicksum, sqrt, Objective, Block
from romodel.reformulate import BaseRobustTransformation
from pyomo.core import TransformationFactory
from pyomo.repn import generate_standard_repn
from romodel.uncset import UncSet, EllipsoidalSet
import numpy as np


@TransformationFactory.register('romodel.ellipsoidal',
                                doc="Ellipsoidal Counterpart")
class EllipsoidalTransformation(BaseRobustTransformation):
    def _check_applicability(self, uncset):
        """
        Returns `True` if the reformulation is applicable to `uncset`

            uncset: UncSet

        """
        # Check for library set
        if uncset.__class__ == EllipsoidalSet:
            return True
        # Check generic set
        elif uncset.__class__ == UncSet:
            first_constraint = True
            is_ellipsoidal = False
            for c in uncset.component_data_objects(Constraint, active=True):
                # make sure set has only one constraint
                if first_constraint:
                    first_constraint = False
                else:
                    return False
                # Check if constraint is ellipsoidal
                repn = generate_standard_repn(c.body)
                if not repn.is_quadratic():
                    return False
                # TODO: assumes implicitly that there is one UncParam per UncSet
                param = repn.quadratic_vars[0][0].parent_component()
                # Collect covariance matrix and mean
                quadratic_coefs = {(id(x[0]), id(x[1])): c for x, c in
                                   zip(repn.quadratic_vars, repn.quadratic_coefs)}
                invcov = [[quadratic_coefs.get((id(param[i]), id(param[j])), 0)
                           for i in param] for j in param]
                invcov = np.array(invcov)
                invcov = 1/2*(invcov + invcov.T)
                eig, _ = np.linalg.eig(invcov)
                cov = np.linalg.inv(invcov)
                mean = -1/2*np.matmul(cov, np.array(repn.linear_coefs))
                uncset.mean = {x: mean[i] for i, x in enumerate(param)}
                uncset.cov = cov.tolist()
                uncset.invcov = invcov
                # TODO: need to check repn.constant == mean^T * cov * mean?

                is_ellipsoidal = ((c.has_ub() and np.all(eig > 0))
                                  or (c.has_lb() and np.all(eig < 0)))

            return is_ellipsoidal

    def _check_constraint(self, c):
        """
        Raise an error if the constraint is inappropriate for this
        reformulation

            c: Constraint

        """
        assert not c.equality, (
                "Currently can't handle equality constraints yet.")

    def _check_objective(self, o):
        """
        Raise an error if the objective is inappropriate for this
        reformulation

            c: Objective

        """
        pass

    def _reformulate(self, c, param, uncset, counterpart, root=False):
        """
        Reformulate an uncertain constraint or objective

            c: Constraint or Objective
            param: UncParam
            uncset: UncSet
            counterpart: Block

        """

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

