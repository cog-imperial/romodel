from romodel.uncset import UncSet


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
