from romodel.uncset import UncSet
from pyomo.environ import Var


class WarpedGPSet(UncSet):
    def __init__(self, gp, var, alpha, tanh=False, *args, **kwargs):
        import scipy as sp
        import rogp
        self.gp = rogp.from_gpy(gp, tanh=tanh)
        if type(var) is dict:
            self.var = var
        else:
            if var.ctype is not Var:
                raise ValueError("Parameter 'var' has to be either a dict,"
                                 " or a Pyomo variable.")
            self.var = [var]
        self.alpha = alpha
        self.F = sp.stats.chi2.ppf(alpha, len(self.var))
        super().__init__(*args, **kwargs)
        self._lib = True

    def is_polyhedral(self):
        return False

    def is_ellipsoidal(self):
        return False


class GPSet(UncSet):
    def __init__(self, gp, var, alpha, *args, **kwargs):
        import scipy as sp
        import rogp
        self.gp = rogp.from_gpy(gp)
        if type(var) is dict:
            self.var = var
        else:
            if var.ctype is not Var:
                raise ValueError("Parameter 'var' has to be either a dict,"
                                 " or a Pyomo variable.")
            self.var = [var]
        self.alpha = alpha
        self.F = sp.stats.norm.ppf(alpha)
        super().__init__(*args, **kwargs)
        self._lib = True

    def is_polyhedral(self):
        return False

    def is_ellipsoidal(self):
        return False
