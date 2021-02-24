from pyomo.core import Transformation, TransformationFactory
from pyomo.core import Objective, Var, Constraint, quicksum, ConstraintList
from pyomo.core.expr.visitor import replace_expressions
from pyomo.repn import generate_standard_repn
from pyomo.core.expr.numvalue import nonpyomo_leaf_types
from pyomo.environ import inequality
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
        self._adjvars = {}
        self._expr_dict = {}

    def _apply_to(self, instance):
        for c in chain(self.get_adjustable_components(instance),
                       self.get_adjustable_components(instance,
                                                      component=Objective)):
            # Collect adjustable vars and uncparams
            adjvar = collect_adjustable(c)
            if id(adjvar) not in self._adjvars:
                self._adjvars[id(adjvar)] = adjvar
            # Create variables for LDR coefficients
            for i in adjvar:
                for u in adjvar[i].uncparams:
                    parent = u.parent_component()
                    if (adjvar.name, parent.name) not in self._coef_dict:
                        coef = Var(adjvar.index_set(), parent.index_set())
                        coef_name = adjvar.name + '_' + parent.name + '_coef'
                        setattr(instance, coef_name, coef)
                        self._coef_dict[adjvar.name, parent.name] = coef

            # Create substitution map
            def coef(u):
                return self._coef_dict[adjvar.name, u.parent_component().name]

            def gen_index(u):
                if hasattr(u, 'index'):
                    yield u.index()
                else:
                    for i in u:
                        yield i

            sub_map = {id(adjvar[i]): sum(u.parent_component()[j]
                                          * coef(u)[i, j]
                                          for u in adjvar[i].uncparams
                                          for j in gen_index(u))
                       for i in adjvar}
            self._expr_dict[adjvar.name] = sub_map
            # Replace AdjustableVar by LDR
            # Objectives
            if c.ctype is Objective:
                e_new = replace_expressions(c.expr, substitution_map=sub_map)
                c_new = Objective(expr=e_new, sense=c.sense)
                setattr(instance, c.name + '_ldr', c_new)
            # Constraints
            elif c.ctype is Constraint:
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

        # Add constraints for bounds on AdjustableVar
        for name, sub_map in self._expr_dict.items():
            adjvar = instance.find_component(name)
            cl = ConstraintList()
            setattr(instance, adjvar.name + '_bounds', cl)
            for i in adjvar:
                if adjvar[i].has_lb():
                    cl.add(adjvar[i].lb <= sub_map[id(adjvar[i])])
                if adjvar[i].has_ub():
                    cl.add(adjvar[i].ub >= sub_map[id(adjvar[i])])


@TransformationFactory.register('romodel.adjustable.nominal',
                                doc=("Replace adjustable variables by "
                                     "regular Pyomo variables"))
class NominalAdjustableTransformation(BaseAdjustableTransformation):
    def __init__(self):
        super().__init__()
        self._adjvar_dict = {}
        self._cons_dict = {}

    def _apply_to(self, instance):
        for c in chain(self.get_adjustable_components(instance),
                       self.get_adjustable_components(instance,
                                                      component=Objective)):
            # Collect adjustable var
            adjvar = collect_adjustable(c)
            # Get regular var
            if adjvar.name not in self._adjvar_dict:
                var = Var(adjvar.index_set(), bounds=adjvar._bounds_init_value)
                setattr(instance, adjvar.name + '_nominal', var)
                self._adjvar_dict[adjvar.name] = var
                for i in adjvar:
                    var[i].fixed = adjvar[i].fixed
                    var[i].setlb(adjvar[i].lb)
                    var[i].setub(adjvar[i].ub)
                    var[i].value = adjvar[i].value
            else:
                var = self._adjvar_dict[adjvar.name]
            # Construct substitution map
            sub_map = {id(adjvar[i]): var[i] for i in adjvar}
            # Replace AdjustableVar with Var
            if c.ctype is Objective:
                e_new = replace_expressions(c.expr, substitution_map=sub_map)
                c_new = Objective(expr=e_new, sense=c.sense)
            else:
                e_new = replace_expressions(c.body, substitution_map=sub_map)
                if c.equality:
                    c_new = Constraint(expr=e_new == c.upper)
                else:
                    c_new = Constraint(expr=inequality(c.lower, e_new, c.upper))
            setattr(instance, c.name + '_nominal', c_new)

            self._cons_dict[c.name] = (c, c_new)

            c.deactivate()
