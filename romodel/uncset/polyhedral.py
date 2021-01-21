from pyomo.core import quicksum
from romodel.uncset import UncSet


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
