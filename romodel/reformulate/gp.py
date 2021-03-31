from pyomo.environ import (Constraint,
                           Var,
                           Objective,
                           maximize,
                           ConstraintList,
                           NonNegativeReals,
                           NonPositiveReals,
                           Block)
from pyomo.environ import sqrt as pyomo_sqrt
from pyomo.core import TransformationFactory
from romodel.uncset import WarpedGPSet, GPSet
from itertools import chain
from romodel.util import collect_uncparam
from pyomo.core.expr.visitor import replace_expressions
from romodel.reformulate import BaseRobustTransformation
import numpy as np


@TransformationFactory.register('romodel.warpedgp',
                                doc="Reformulate warped Gaussian Process set.")
class WGPTransformation(BaseRobustTransformation):
    def _apply_to(self, instance, initialize_wolfe=False):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            if c.ctype is Constraint and c.equality:
                raise RuntimeError(
                        "'UncParam's cannot appear in equality constraints, "
                        "unless the constraint also contains adjustable "
                        "variables.")

            # Collect uncertain parameters and uncertainty set
            param = collect_uncparam(c)
            uncset = param.uncset

            # Only proceed if uncertainty set is WarpedGPSet
            if not uncset.__class__ == WarpedGPSet:
                continue
            from rogp.util.numpy import _pyomo_to_np, _to_np_obj_array

            repn = self.generate_repn_param(instance, c)

            assert repn.is_linear(), ("Only constraints which are linear in "
                                      "the uncertain parameters are valid for "
                                      "uncertainty set WarpedGPSet.")

            # Constraint may contain subset of elements of UncParam
            index_set = [p.index() for p in repn.linear_vars]
            x = [[xi] for xi in repn.linear_coefs]
            x = _to_np_obj_array(x)

            # Set up Block for Wolfe duality constraints
            b = Block()
            setattr(instance, c.name + '_counterpart', b)
            # Set up extra variables
            b.y = Var(param.index_set())
            # Set bounds for extra variables based on UncParam bounds
            for i in param:
                lb, ub = param[i].lb, param[i].ub
                b.y[i].setlb(lb)
                b.y[i].setub(ub)
                if ((lb is not None and lb is not float('-inf'))
                        and (ub is not None and ub is not float('inf'))):
                    b.y[i].value = (ub + lb)/2
            y = _pyomo_to_np(b.y, ind=index_set)
            # Setup dual vars based on sense
            if c.ctype is Constraint:
                if c.has_ub() and not c.has_lb():
                    b.u = Var(within=NonPositiveReals)
                elif not c.has_ub() and c.has_lb():
                    b.u = Var(within=NonNegativeReals)
                else:
                    raise RuntimeError(
                            "Uncertain constraints with 'WarpedGPSet' "
                            "currently only support either an upper or a "
                            "lower bound, not both."
                            )
            elif c.ctype is Objective:
                if c.sense is maximize:
                    b.u = Var(within=NonPositiveReals)
                else:
                    b.u = Var(within=NonNegativeReals)
            u = _pyomo_to_np(b.u)

            # Primal constraint
            sub_map = {id(param[i]): b.y[i] for i in param}
            if c.ctype is Constraint:
                e_new = replace_expressions(c.body, substitution_map=sub_map)
                b.primal = Constraint(rule=lambda x: (c.lower, e_new, c.upper))
            else:
                e_new = replace_expressions(c.expr, substitution_map=sub_map)
                b.primal = Objective(expr=e_new, sense=c.sense)

            # Calculate matrices
            gp = uncset.gp
            var = uncset.var
            if type(var) is dict:
                z = [var[i] for i in index_set]
                z = _to_np_obj_array(z)
            else:
                var = var[0]
                assert var.index_set() == param.index_set(), (
                        "Index set of `UncParam` and `var` in `WarpedGPSet` "
                        "should be the same. Alternatively use "
                        "var = {index: [list of vars]}"
                        )
                z = _pyomo_to_np(var, ind=index_set)

            Sig = gp.predict_cov_latent(z)
            mu = gp.predict_mu_latent(z)
            dHinv = 1/gp.warp_deriv(y)
            dHinv = np.diag(dHinv[:, 0])
            hz = gp.warp(y)

            LHS = np.matmul(Sig, dHinv)
            LHS = np.matmul(LHS, x)
            RHS = LHS
            LHS = LHS + 2*u*(hz - mu)
            # Add stationarity condition
            b.stationarity = ConstraintList()
            for lhs in np.nditer(LHS, ['refs_ok']):
                b.stationarity.add(lhs.item() == 0)
            RHS = np.matmul(dHinv, RHS)
            rhs = np.matmul(x.T, RHS)[0, 0]
            lhs = 4*u[0, 0]**2*uncset.F
            # Set consistent initial value for u (helps convergence)
            if initialize_wolfe:
                u0 = np.sqrt(rhs()/4/uncset.F)
                if u[0, 0].ub == 0:
                    u[0, 0].value = -u0
                elif u[0, 0].lb == 0:
                    u[0, 0].value = u0
            # Dual variable constraint
            b.dual = Constraint(expr=lhs == rhs)

            c.deactivate()


@TransformationFactory.register('romodel.gp',
                                doc="Reformulate Gaussian Process set.")
class GPTransformation(BaseRobustTransformation):
    def _apply_to(self, instance):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            if c.ctype is Constraint and c.equality:
                raise RuntimeError(
                        "'UncParam's cannot appear in equality constraints, "
                        "unless the constraint also contains adjustable "
                        "variables.")

            # Collect uncertain parameters and uncertainty set
            param = collect_uncparam(c)
            uncset = param.uncset

            # Only proceed if uncertainty set is GPSet
            if not uncset.__class__ == GPSet:
                continue
            from rogp.util.numpy import _pyomo_to_np, _to_np_obj_array

            repn = self.generate_repn_param(instance, c)

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
                    setattr(instance, c.name + '_counterpart_upper', c_new)
                if c.has_lb():
                    e_new = nominal - padding >= c.lower
                    c_new = Constraint(expr=e_new)
                    setattr(instance, c.name + '_counterpart_lower', c_new)
            else:
                e_new = nominal + padding*c.sense
                c_new = Objective(expr=e_new, sense=c.sense)
                setattr(instance, c.name + '_counterpart', c_new)

            c.deactivate()
