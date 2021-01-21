from pyomo.core import SimpleBlock, ModelComponentFactory, Component
from pyomo.core import Constraint, quicksum
from pyomo.repn import generate_standard_repn
from romodel.visitor import identify_parent_components
from romodel.uncparam import UncParam
import numpy as np


class BaseSet():
    def __init__():
        pass

    def is_ellipsoidal(self):
        return False

    def is_polyhedral(self):
        return False


@ModelComponentFactory.register("Uncertainty set in a robust problem")
class UncSet(SimpleBlock):
    """
    This model component defines an uncertainty set in a robust optimization
    problem.
    """

    def __init__(self, *args, **kwargs):
        """Constructor"""
        _rule = kwargs.pop('rule', None)
        _fixed = kwargs.pop('fixed', None)
        _param = kwargs.pop('param', None)

        # _var = kwargs.pop('var', None)
        #
        # Initialize the SimpleBlock
        #
        kwargs.setdefault('ctype', UncSet)
        SimpleBlock.__init__(self, *args, **kwargs)
        #
        # Initialize from kwargs
        #
        self._rule = _rule
        if isinstance(_fixed, Component):
            self._fixed = [_fixed]
        else:
            self._fixed = _fixed
        if isinstance(_param, Component):
            self._param = [_param]
        else:
            self._param = _param

        self._lib = False

    def is_ellipsoidal(self):
        # TODO: assumes there is only one constraint on UncSet
        for c in self.component_data_objects(Constraint, active=True):
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
            self.mean = {x: mean[i] for i, x in enumerate(param)}
            self.cov = cov.tolist()
            self.invcov = invcov
            # TODO: need to check repn.constant == mean^T * cov * mean?

            return ((c.has_ub() and np.all(eig > 0))
                    or (c.has_lb() and np.all(eig < 0)))

    def is_polyhedral(self):
        mat = []
        rhs = []
        param = None
        for c in self.component_data_objects(Constraint, active=True):
            # Collect uncertain parameter
            for p in identify_parent_components(c.body, [UncParam]):
                if param is None:
                    param = p
                else:
                    assert param is p, ("Uncertainty set {} should "
                                        "only contain one UncParam "
                                        "component.".format(self.name))
            # Generate standard repn
            repn = generate_standard_repn(c.body)
            # If uncertainty set contains a non-linear constraint it's not
            # polyhedral.
            if not repn.is_linear():
                return False
            coef_dict = {id(x): y for x, y in zip(repn.linear_vars,
                                                  repn.linear_coefs)}
            if c.has_ub():
                mat.append([coef_dict.get(id(param[i]), 0) for i in param])
                rhs.append(c.upper - repn.constant)
            elif c.has_lb():
                mat.append([-coef_dict.get(id(param[i]), 0) for i in param])
                rhs.append(repn.constant - c.lower)
        self.mat = mat
        self.rhs = rhs
        return True

    def is_empty(self):
        if self._lib:
            return False
        for c in self.component_data_objects(Constraint, active=True):
            return False
        return True

    def is_lib(self):
        return self._lib

    def get_uncertain_param(self):
        param = None
        for p in self.component_objects(UncParam, active=True):
            if param is None:
                param = p
            else:
                assert param is p


class EllipsoidalSet(UncSet):
    '''
    Defines an ellipsoidal uncertainty set of shape:
        (param - mu)^T * A * (param - mu) <= 1
    '''
    def __init__(self, mean, cov, *args, **kwargs):
        rhs = kwargs.pop('rhs', 1)
        self.mean = mean
        self.cov = cov
        self.rhs = rhs
        super().__init__(*args, **kwargs)
        self._lib = True

    def generate_cons_from_lib(self, param):
        assert len(param) == len(self.mean)
        expr = 0
        invcov = np.linalg.inv(self.cov)
        for i, ind_i in enumerate(param):
            for j, ind_j in enumerate(param):
                expr += ((param[ind_i] - self.mean[i])
                         * invcov[i, j]
                         * (param[ind_j] - self.mean[j]))
        yield None, expr, self.rhs

    def is_ellipsoidal(self):
        return True

    def is_polyhedral(self):
        return False


class PolyhedralSet(UncSet):
    '''
    Defines a polyhedral uncertainty set of shape:
        P * param <= b
    '''
    def __init__(self, mat, rhs, *args, **kwargs):
        self.mat = mat
        self.rhs = rhs
        super().__init__(*args, **kwargs)
        self._lib = True

    def generate_cons_from_lib(self, param):
        for i, row in enumerate(self.mat):
            yield (None,
                   quicksum(row[j]*param[ind]
                            for j, ind in enumerate(param)),
                   self.rhs[i])

    def is_polyhedral(self):
        return True

    def is_ellipsoidal(self):
        return False


class WarpedGPSet(UncSet):
    def __init__(self, gp, var, alpha, *args, **kwargs):
        import scipy as sp
        from rogp.util.numpy import _to_np_obj_array
        self.gp = gp
        self.var = var
        self.alpha = alpha
        z = _to_np_obj_array(var)
        self.Sig = self.gp.predict_cov_latent(z)
        self.mu = self.gp.predict_mu_latent(z)
        self.F = sp.stats.chi2.ppf(alpha, len(self.var))
        super().__init__(*args, **kwargs)
        self._lib = True

    def is_polyhedral(self):
        return False

    def is_ellipsoidal(self):
        return False
