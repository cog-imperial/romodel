from pyomo.core import Transformation, TransformationFactory
from pyomo.core import Objective, Var, Constraint, quicksum, ConstraintList
from pyomo.core.expr.visitor import replace_expressions
from pyomo.repn import generate_standard_repn
from pyomo.core.expr.numvalue import nonpyomo_leaf_types
from itertools import chain
from romodel.util import collect_adjustable
from romodel.visitor import _expression_is_adjustable


class BaseAdjustableTransformation(Transformation):
    def __init__(self):
        self._fixed_components = {}

    def get_adjustable_components(self, instance, component=Constraint):
        """ Return all uncertain components of type `component`. """
        comp_list = instance.component_data_objects(component, active=True)
        for c in comp_list:
            if hasattr(c, 'body'):
                # Constraints
                expr = c.body
            else:
                # Objective
                expr = c.expr

            if _expression_is_adjustable(expr):
                yield c

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

    def generate_repn_param(self, instance, expr):
        self.fix_component(instance, component=Var)
        repn = generate_standard_repn(expr, compute_values=False)
        self.unfix_component(component=Var)
        return repn


@TransformationFactory.register('romodel.adjustable.ldr',
                                doc=("Replace adjustable variables by Linear"
                                     "decision rules"))
class LDRAdjustableTransformation(BaseAdjustableTransformation):
    def __init__(self):
        super().__init__()
        self._coef_dict = {}
        self._uncparam_dict = {}

    def _apply_to(self, instance):
        for c in chain(self.get_adjustable_components(instance),
                       self.get_adjustable_components(instance,
                                                      component=Objective)):
            # Collect adjustable vars and uncparams
            adjvar = collect_adjustable(c)
            uncparams = adjvar._uncparams
            # Create variables for LDR coefficients
            for u in uncparams:
                if not self._coef_dict.get((adjvar.name, u.name), False):
                    coef = Var(adjvar.index_set(), u.index_set())
                    coef_name = adjvar.name + '_' + u.name + '_coef'
                    setattr(instance, coef_name, coef)
                    self._coef_dict[adjvar.name, u.name] = coef

            # Create substitution map
            def coef(u):
                return self._coef_dict[adjvar.name, u.name]
            sub_map = {id(adjvar[i]): quicksum(u[j] * coef(u)[i, j]
                                               for u in uncparams
                                               for j in u)
                       for i in adjvar}
            # Replace AdjustableVar by LDR
            # Objectives
            if c.ctype is Objective:
                e_new = replace_expressions(c.expr, substitution_map=sub_map)
                c_new = Objective(expr=e_new, sense=c.sense)
                setattr(instance, c.name + '_ldr', c_new)
            # Constraints
            elif c.type is Constraint:
                e_new = replace_expressions(c.body, substitution_map=sub_map)

                if c.equality:
                    repn = self.generate_repn_param(instance, e_new)

                    c_new = ConstraintList()
                    setattr(instance, c.name + '_ldr', c_new)
                    # Check if repn.constant is an expression
                    cons = repn.constant
                    if cons.__class__ in nonpyomo_leaf_types:
                        if cons != 0:
                            raise ValueError("Can't reformulate constraint {} "
                                             "with numeric constant "
                                             "{}".format(c.name, cons))
                    elif cons.is_potentially_variable():
                        c_new.add(cons == 0)
                    else:
                        raise ValueError("Can't reformulate constraint {} with"
                                         " constant "
                                         "{}".format(c.name, cons))
                    # Add constraints for each uncparam
                    for coef in repn.linear_coefs:
                        c_new.add(coef == 0)
                    for coef in repn.quadratic_coefs:
                        c_new.add(coef == 0)
                else:
                    def c_rule(x):
                        return (c.lower, e_new, c.upper)
                    c_new = Constraint(rule=c_rule)
                    setattr(instance, c.name + '_ldr', c_new)

            c.deactivate()

            # Do some checks?
            # Check c is linear in adj_var?
