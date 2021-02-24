from pyomo.core.base.block import declare_custom_block, _BlockData
from pyomo.environ import (ConstraintList,
                           Constraint,
                           ConcreteModel,
                           Objective,
                           Var,
                           quicksum,
                           minimize,
                           maximize,
                           value,
                           SolverFactory)
from pyomo.core.expr.current import identify_variables
from pyomo.core.expr.visitor import replace_expressions
from pyomo.repn import generate_standard_repn
from pyomo.opt import TerminationCondition
from romodel import UncParam
from romodel.visitor import identify_parent_components


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

    def build(self, lower, expr, upper):
        # Collect uncertain parameter and uncertainty set
        self.lower = lower
        self.upper = upper
        self._uncparam = _collect_uncparam(expr)
        self._uncset = [self._uncparam[0]._uncset]
        self._rule = self.construct_rule(expr)
        self._bounds = (lower, upper)
        # Generate nominal constraint
        nominal_expr = self.nominal_constraint_expr()
        self._constraints = ConstraintList()
        self._constraints.add(nominal_expr)

    def has_lb(self):
        if self.lower is None or self.lower is float('-inf'):
            return False
        else:
            return True

    def has_ub(self):
        if self.upper is None or self.upper is float('inf'):
            return False
        else:
            return True

    def add_cut(self, solver='gurobi', options={}):
        """ Solve separation problem and add cut. """
        opt = SolverFactory(solver)
        for key, val in options.items():
            opt.options[key] = val

        feasible = True
        if self.has_ub():
            sep = self.construct_separation_problem(sense=maximize)
            res = opt.solve(sep)
            if (res.solver.termination_condition
                    is not TerminationCondition.optimal):
                raise RuntimeError(
                        "Solver '{}' failed to solve separation "
                        "problem.".format('gurobi')
                        )

            uncparam = sep.uncparam
            expr = self._rule({i: uncparam[i].value for i in uncparam})

            self._constraints.add((self.lower, expr, self.upper))
            feasible = feasible and value(sep.obj <= self.upper)

        if self.has_lb():
            sep = self.construct_separation_problem(sense=minimize)
            res = opt.solve(sep)
            if (res.solver.termination_condition
                    is not TerminationCondition.optimal):
                raise RuntimeError(
                        "Solver '{}' failed to solve separation "
                        "problem.".format(opt.name)
                        )

            uncparam = sep.uncparam
            expr = self._rule({i: uncparam[i].value for i in uncparam})

            self._constraints.add((self.lower, expr, self.upper))
            feasible = feasible and value(self.lower <= sep.obj)

        self.feasible = feasible

        return feasible

    def construct_separation_problem(self, sense=maximize):
        m = ConcreteModel()
        uncparam = self._uncparam[0]
        index = uncparam.index_set()
        # Create inner problem variables
        # TODO: use setattr to give uncparam same name as in model
        m.uncparam = Var(index)
        # collect current coefficient values
        expr = self._rule(m.uncparam, compute_values=True)
        # construct objective with current coefficient values
        m.obj = Objective(expr=expr, sense=sense)

        # construct constraints from uncertainty set
        uncset = self._uncset[0]
        m.cons = ConstraintList()
        substitution_map = {id(uncparam[i]): m.uncparam[i] for i in index}
        if not uncset.is_lib():
            for c in uncset.component_data_objects(Constraint):
                m.cons.add((c.lower,
                            replace_expressions(c.body, substitution_map),
                            c.upper))
        else:
            for cons in uncset.generate_cons_from_lib(m.uncparam):
                m.cons.add(cons)
        return m
        # What should we do with the constraints? Replace UncParams by the
        # Vars? Or restructure UncParam so that we can solve directly.k

    # !!!
    def nominal_constraint_expr(self):
        """ Generate the nominal constraint. """
        # TODO: replace p.value by p.nominal
        uncparam = self._uncparam[0]
        expr = self._rule({i: uncparam[i].nominal for i in uncparam})
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
                        + value(constant))
            else:
                return quicksum(index_coef_dict[i]*x[i] for i in x) + constant

        return rule


def _collect_uncparam(expr):
    uncparam = [i for i in identify_parent_components(expr, [UncParam])]
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
