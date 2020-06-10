from pyomo.environ import (Constraint,
                           Var,
                           quicksum,
                           sqrt,
                           Objective,
                           maximize,
                           minimize)
from pyomo.core import Transformation
from pyomo.repn import generate_standard_repn
from pro import UncParam
from pro.visitor import _expression_is_uncertain, identify_parent_components
from pao.duality import create_linear_dual_from


def solve_robust(m):
    for c in m.component_data_objects(Constraint, active=True):
        if contains_uncparam(c):
            reformulate(c)


def contains_uncparam(c):
    pass


def reformulate(c):
    pass


class BaseRobustTransformation(Transformation):
    def __init__(self):
        self._fixed_unc_params = []
        self._fixed_components = {}

    def fix_component(self, instance, component=Var):
        fixed = []
        for c in instance.component_data_objects(component, active=True):
            if not c.is_fixed():
                c.fix()
                fixed.append(c)
        self._fixed_components[component] = fixed

    def unfix_component(self, component=Var):
        for c in self._fixed_components[component]:
            c.unfix()
        self._fixed_components[component] = None

    def get_uncertain_constraints(self, instance):
        """ Return all uncertain constraints on the model. """
        constraints = instance.component_data_objects(Constraint, active=True)
        # TODO: generator
        return [c for c in constraints if _expression_is_uncertain(c.expr)]

    def generate_repn_param(self, instance, cons):
        self.fix_component(instance, component=Var)
        repn = generate_standard_repn(cons.body, compute_values=False)
        self.unfix_component(component=Var)
        return repn

    def is_linear_in_param(self, instance, cons):
        """ Checks if a constraint is linear in the uncertain parameters. """
        # TODO: Check if this can lead to problems if Var components have no
        # initial value. Are they evaluated in generate_standard_repn?
        return self.generate_repn_param(instance, cons).is_linear()


class EllipsoidalTransformation(BaseRobustTransformation):
    def _apply_to(self, instance, root=False):
        for c in self.get_uncertain_constraints(instance):

            assert not c.equality, (
                    "Currently can't handle equality constraints yet.")
            repn = self.generate_repn_param(instance, c)
            assert repn.is_linear(), (
                    "Constraint {} should be linear in "
                    "unc. parameters".format(c.name))
            param = list(identify_parent_components(c.expr, [UncParam]))
            assert len(param) == 1, (
                    "Constraint {} should not contain more than one UncParam "
                    "component".format(c.name))

            # Collect UncParam and UncSet
            param = param[0]
            uncset = param._uncset
            assert uncset.is_ellipsoidal(), (
                    "Uncertainty set {} is not "
                    "ellipsoidal.".format(uncset.name))

            # Set parameter value to deterministic
            for key in param.keys():
                param[key].value = uncset.mean[key]

            # Generate robust counterpart
            det = quicksum(x[0]*x[1].value for x in zip(repn.linear_coefs,
                                                        repn.linear_vars))
            det += repn.constant
            param_var_dict = {id(param): var
                              for param, var
                              in zip(repn.linear_vars, repn.linear_coefs)}
            # padding = sqrt( var^T * cov^-1 * var )
            padding = quicksum(param_var_dict[id(x)]
                               * uncset.cov[i, j]
                               * param_var_dict[id(y)]
                               for i, x in param.iteritems()
                               for j, y in param.iteritems())
            # For upper bound: det + padding <= b
            if c.upper:
                name = c.name + '_counterpart_upper'
                if root:
                    cp = Constraint(expr=det + sqrt(padding) <= c.upper)
                else:
                    cp = Constraint(expr=padding <= (c.upper() - det)**2)
                    c_det = Constraint(expr=det <= c.upper)
                    setattr(instance, c.name + '_det_upper', c_det)
                setattr(instance, name, cp)
            # For lower bound: det - padding >= b
            if c.lower:
                name = c.name + '_counterpart_lower'
                if root:
                    cp = Constraint(expr=det - sqrt(padding) >= c.lower)
                else:
                    cp = Constraint(expr=padding <= (det - c.lower())**2)
                    c_det = Constraint(expr=c.lower <= det)
                    setattr(instance, c.name + '_det_lower', c_det)
                setattr(instance, name, cp)
            c.deactivate()


class PolyhedralTransformation(BaseRobustTransformation):
    def _apply_to(self, instance):
        for c in self.get_uncertain_constraints(instance):

            assert not c.equality, (
                    "Currently can't handle equality constraints yet.")
            repn = self.generate_repn_param(instance, c)
            assert repn.is_linear(), (
                    "Constraint {} should be linear in "
                    "unc. parameters".format(c.name))
            param = list(identify_parent_components(c.expr, [UncParam]))
            assert len(param) == 1, (
                    "Constraint {} should not contain more than one UncParam "
                    "component".format(c.name))

            # Collect UncParam and UncSet
            param = param[0]
            uncset = param._uncset
            assert uncset.is_polyhedral(), (
                    "Uncertainty set {} is not "
                    "polyhedral.".format(uncset.name))

            # Create dual variables
            block = c.parent_block()
            setattr(block, c.name + '_dual', Var())

            # Add dual constraints d^T * v <= b, P^T * v = x
            if c.upper:
                uncset.obj = Objective(expr=c.body, sense=maximize)
                dual = create_linear_dual_from(uncset, unfixed=param.values())
                o_expr = dual.o.expr
                del dual.o
                dual.o = Constraint(expr=o_expr <= c.upper)
                del uncset.obj
                setattr(instance, c.name + '_counterpart_upper', dual)
            if c.lower:
                uncset.obj = Objective(expr=c.body, sense=minimize)
                dual = create_linear_dual_from(uncset, unfixed=param.values())
                o_expr = dual.o.expr
                del dual.o
                dual.o = Constraint(expr=c.lower <= o_expr)
                del uncset.obj
                setattr(instance, c.name + '_counterpart_lower', dual)
            # Deactivate original constraint
            c.deactivate()
