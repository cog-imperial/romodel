from pyomo.core.base.block import declare_custom_block, _BlockData
from pyomo.environ import (ConstraintList,
                           Constraint,
                           ConcreteModel,
                           Objective,
                           Var,
                           quicksum,
                           maximize,
                           value,
                           SolverFactory)
from pyomo.core.expr.current import identify_variables
from pyomo.core.expr.visitor import replace_expressions
from pyomo.repn import generate_standard_repn
from pro import UncParam
from pro.visitor import identify_parent_components
from collections import defaultdict


@declare_custom_block(name='RobustConstraint')
class RobustConstraintData(_BlockData):
    """ RobustConstraint contains:
            - attribute _cons: constraint
            - _uncset: uncertainty set
            - _uncparam: uncertain parameter
            - _vars: variables the constraint contains
    """
    def __init__(self, component, cons=None):
        super().__init__(component)
        self._cons = None
        self._uncparam = None
        self._uncset = None
        self._vars = []
        self._sep = None

    def build(self, cons):
        # Collect uncertain parameter and uncertainty set
        self.lower = cons.lower
        self.upper = cons.upper
        self._uncparam = _collect_uncparam(cons)
        self._uncset = [self._uncparam[0]._uncset]
        self._rule = self.construct_rule(cons.body)
        self._bounds = (cons.lower, cons.upper)
        # Generate nominal constraint
        nominal_expr = self.nominal_constraint_expr()
        self._constraints = ConstraintList()
        self._constraints.add(nominal_expr)

    def add_cut(self):
        """ Solve separation problem and add cut. """
        sep = self.construct_separation_problem()
        # TODO: pass option
        opt = SolverFactory('gurobi')
        res = opt.solve(sep)
        # Check results are okay

        # Check feasibility?

        # Generate cut expression:
        # uncparam = self._uncparam[0]
        uncparam = sep.uncparam
        expr = self._rule({i: uncparam[i].value for i in uncparam})

        self._constraints.add((self.lower, expr, self.upper))
        self.feasible = value(sep.obj <= self.upper)

        return self.feasible

    def construct_separation_problem(self):
        m = ConcreteModel()
        uncparam = self._uncparam[0]
        index = uncparam.index_set()
        # Create inner problem variables
        # TODO: use setattr to give uncparam same name as in model
        m.uncparam = Var(index)
        # collect current coefficient values
        expr = self._rule(m.uncparam, compute_values=True)
        # construct objective with current coefficient values
        m.obj = Objective(expr=expr, sense=maximize)

        # construct constraints from uncertainty set
        uncset = self._uncset[0]
        m.cons = ConstraintList()
        substitution_map = {id(uncparam[i]): m.uncparam[i] for i in index}
        for c in uncset.component_data_objects(Constraint):
            m.cons.add((c.lower,
                        replace_expressions(c.body, substitution_map),
                        c.upper))
        return m
        # What should we do with the constraints? Replace UncParams by the
        # Vars? Or restructure UncParam so that we can solve directly.k

    # !!!
    def nominal_constraint_expr(self):
        """ Generate the nominal constraint. """
        # TODO: replace p.value by p.nominal
        uncparam = self._uncparam[0]
        expr = self._rule({i: uncparam[i].value for i in uncparam})
        return (self._bounds[0], expr, self._bounds[1])

    # !!!
    def is_feasible(self):
        return False

    def construct_rule(self, expr):
        repn = generate_linear_repn(expr)
        linear_vars = repn.linear_vars
        linear_coefs = repn.linear_coefs
        constant = repn.constant
        id_coef_dict = {id(v): c for v, c in zip(linear_vars,
                                                 linear_coefs)}
        param = self._uncparam[0]
        index_coef_dict = {i: id_coef_dict.get(id(param[i]), 0) for i in param}

        def rule(x, compute_values=False):
            if compute_values:
                return (quicksum(value(index_coef_dict[i])*x[i] for i in x)
                        + constant)
            else:
                return quicksum(index_coef_dict[i]*x[i] for i in x) + constant

        return rule


def _collect_uncparam(c):
    uncparam = [i for i in identify_parent_components(c.expr, [UncParam])]
    return uncparam


def generate_linear_repn(expr, evaluate=False):
    """
    Given an expression containing UncParam return its linear representation.

    :param c: A pyomo expression
    :type c: class:`pyomo.core.expr.numeric_expr.ExpressionBase`
    :param evaluate: If true, evaluate fixed expressions
    :type c: bool, optional

    :return: Standard representation containing a constant term (wrt UncParam),
    linear coefficients, and the corresponding UncParam objects.
    :rtype: pyomo.repn.StandardRepn


    """
    # Fix all Var types (and set values if necessary)
    uncparam_list = []
    var_list = []
    _fixed = []
    for v in identify_variables(expr):
        if isinstance(v.parent_component(), UncParam):
            uncparam_list.append(v)
        else:
            var_list.append(v)
            if not v.fixed:
                v.fix()
                _fixed.append(v)

    repn = generate_standard_repn(expr, compute_values=False, quadratic=False)

    for v in _fixed:
        v.unfix()

    return repn
