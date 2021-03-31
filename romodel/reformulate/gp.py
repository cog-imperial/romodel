from pyomo.environ import (Constraint,
                           Objective)
from pyomo.environ import sqrt as pyomo_sqrt
from pyomo.core import TransformationFactory
from romodel.uncset import GPSet
from romodel.reformulate import BaseRobustTransformation
import numpy as np


@TransformationFactory.register('romodel.gp',
                                doc="Reformulate Gaussian Process set.")
class GPTransformation(BaseRobustTransformation):
    def _check_applicability(self, uncset):
        """
        Returns `True` if the reformulation is applicable to `uncset`

            uncset: UncSet

        """
        # Only proceed if uncertainty set is GPSet
        return uncset.__class__ == GPSet

    def _check_constraint(self, c):
        """
        Raise an error if the constraint is inappropriate for this
        reformulation

            c: Constraint

        """
        if c.equality:
            raise RuntimeError(
                    "'UncParam's cannot appear in equality constraints, "
                    "unless the constraint also contains adjustable "
                    "variables.")

    def _reformulate(self, c, param, uncset, counterpart):
        """
        Reformulate an uncertain constraint or objective

            c: Constraint or Objective
            param: UncParam
            uncset: UncSet
            counterpart: Block

        """
        from rogp.util.numpy import _pyomo_to_np, _to_np_obj_array

        repn = self.generate_repn_param(c)

        assert repn.is_linear(), ("Only constraints which are linear in "
                                  "the uncertain parameters are valid for "
                                  "uncertainty set 'GPSet'.")

        # Constraint may contain subset of elements of UncParam
        index_set = [p.index() for p in repn.linear_vars]
        x = [[xi] for xi in repn.linear_coefs]
        x = _to_np_obj_array(x)

        # Calculate matrices
        gp = uncset.gp
        var = uncset.var
        if type(var) is dict:
            pass
            z = [var[i] for i in index_set]
            z = _to_np_obj_array(z)
        else:
            var = var[0]
            assert var.index_set() == param.index_set(), (
                    "Index set of `UncParam` and `var` in `GPSet` "
                    "should be the same. Alternatively use "
                    "var = {index: [list of vars]}"
                    )
            z = _pyomo_to_np(var, ind=index_set)

        Sig = gp.predict_cov(z)
        mu = gp.predict_mu(z)

        nominal = np.matmul(mu.T, x)[0, 0] + repn.constant

        padding = np.matmul(x.T, Sig)
        padding = np.matmul(padding, x)
        padding = uncset.F*pyomo_sqrt(padding[0, 0])

        # Counterpart
        if c.ctype is Constraint:
            if c.has_ub():
                e_new = nominal + padding <= c.upper
                c_new = Constraint(expr=e_new)
                counterpart.upper = c_new
            if c.has_lb():
                e_new = nominal - padding >= c.lower
                c_new = Constraint(expr=e_new)
                counterpart.lower = c_new
        else:
            e_new = nominal + padding*c.sense
            c_new = Objective(expr=e_new, sense=c.sense)
            counterpart.obj = c_new
