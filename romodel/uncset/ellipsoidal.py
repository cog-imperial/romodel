import numpy as np
from romodel.uncset import UncSet


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
