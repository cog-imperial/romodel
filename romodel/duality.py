#  ___________________________________________________________________________
#
#  Pyomo: Python Optimization Modeling Objects
#  Copyright 2017 National Technology and Engineering Solutions of Sandia, LLC
#  Under the terms of Contract DE-NA0003525 with National Technology and
#  Engineering Solutions of Sandia, LLC, the U.S. Government retains certain
#  rights in this software.
#  This software is distributed under the 3-clause BSD License.
#  ___________________________________________________________________________

#  ----------------------
#
#  I've taken this file from the PAO repository so that we don't have to add
#  PAO as a dependency just for this function while it's still under heavy
#  development.
#
#  ----------------------

"""
pao.duality.collect

This module defines the collect_dual_representation() function, which
collects information that is used to compute a linear dual of a given
block.
"""

__all__ = ("create_linear_dual_from",)

#pylint: disable-msg=invalid-name
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-branches
#pylint: disable-msg=too-many-statements

from pyutilib.misc import Bunch
from pyomo.repn import generate_standard_repn
from pyomo.core import (Var,
                        Constraint,
                        Objective,
                        minimize,
                        maximize,
                        NonNegativeReals,
                        NonPositiveReals,
                        Reals,
                        Block,
                        Model,
                        ConcreteModel)
from pyomo.core.expr.visitor import identify_variables


def collect_dual_representation(block, fixed_modelvars):
    """
    Process linear terms from a block and return information that is
    used to define the dual. This function does not change the block.

    Note that variable bounds are treated as constraints unless they are zero.  For example,
    a variable X with bounds (-1, 1) is treated as bounded with two additional inequality
    constraints for the lower and upper bounds.  However, a variable Y with bounds (0, 3)
    is treated as a nonnegative variable with an additional constraint that defines its
    upper bound.


    Arguments:
        block: The SubModel object that is dualized
        fixed_modelvars: A map from variable ids to VarData objects, which will be
                fixed before dualization

    Returns: Tuple with the following values:
        A:        The dual matrix
        b_coef:   The coefficients of the dual objective
        obj_offset: The offset for the dual objective
        c_rhs:    The dual constraint right-hand side
        c_sense:  The sense of each constraint in the dual
        d_sense:  The sense of the dual objective
        v_domain: A dictionary that indicates the domain of the dual variable
                      (-1: Nonpositive, 0: Unbounded, 1: Nonnegative)
    """
    #
    # fix variables
    #
    for vdata in fixed_modelvars.values():
        vdata.fixed = True

    all_vars = {}

    A = {}
    b_coef = {}
    obj_offset = 0
    c_rhs = {}
    c_sense = {}
    d_sense = None
    v_domain = {}
    #
    # Collect objective
    #
    nobj = 0
    for odata in block.component_objects(Objective, active=True):
        for ndx in odata:
            o_terms = generate_standard_repn(odata[ndx].expr, compute_values=False)
            obj_offset = o_terms.constant
            if odata[ndx].sense == maximize:
                d_sense = minimize
            else:
                d_sense = maximize
            for var, coef in zip(o_terms.linear_vars, o_terms.linear_coefs):
                # Variables should not be fixed
                #if var.fixed:
                #    continue
                parent = var.parent_component()
                if parent is None:
                    raise RuntimeError("ERROR: Variable %s encountered that is not owned by a Pyomo model" % str(var))
                try:
                    # The variable is in the subproblem
                    varname = parent.getname(fully_qualified=True,
                                                             relative_to=block)
                except RuntimeError:
                    # The variable is somewhere else in the model
                    varname = parent.getname(fully_qualified=True,
                                                             relative_to=block.model())
                varndx = var.index()
                all_vars[varname, varndx] = var
                c_rhs[varname, varndx] = coef
            nobj += 1
    if nobj == 0:
        raise RuntimeError("Error dualizing block.  No objective expression.")
    if nobj > 1:
        raise RuntimeError("Error dualizing block.  Multiple objective expressions.")
    if not c_rhs:
        # If len(c_rhs) == 0, then the objective is constant
        raise RuntimeError("Error dualizing block.  Objective is constant.")
    #
    # Collect constraints
    #
    #print("Y1")
    for data in block.component_objects(Constraint, active=True):
        name = data.getname(relative_to=block)
        #print("Y2")
        for ndx in data:
            con = data[ndx]
            if not (con.equality or con.lower is None or con.upper is None):
                dualvars = [name+"_lb_", name+"_ub_"]
            else:
                dualvars = [name]
            body_terms = generate_standard_repn(con.body, compute_values=False)
            #print("HERE")
            #print(body_terms)
            if body_terms.is_fixed():
                #
                # If a constraint has a fixed body, then don't collect it.
                #
                continue
            nvars = 0
            for var, coef in zip(body_terms.linear_vars, body_terms.linear_coefs):
                #if not unfixed and ((len(unfixed_vars) > 0  and (id(var) not in unfixed_vars)) or (id(var) in fixed_vars)):
                #if var.fixed:
                    # Variable is fixed
                    #body_terms.constant += coef*var
                    #continue
                nvars += 1
                try:
                    # The variable is in the subproblem
                    varname = var.parent_component().getname(fully_qualified=True,
                                                             relative_to=block)
                except RuntimeError:
                    # The variable is somewhere else in the model
                    varname = var.parent_component().getname(fully_qualified=True,
                                                             relative_to=block.model())
                varndx = var.index()
                all_vars[varname, varndx] = var
                for dvar in dualvars:
                    A.setdefault(varname, {}).setdefault(varndx, []).append(
                                                Bunch(coef=coef, var=dvar, ndx=ndx))
            #
            lower_terms = generate_standard_repn(con.lower, compute_values=False) \
                                                    if not con.lower is None else None
            upper_terms = generate_standard_repn(con.upper, compute_values=False) \
                                                    if not con.upper is None else None
            #
            # It looks like Pyomo ensures that the lower or upper bounds are constant.
            # No additional checks are done.
            #
            assert(lower_terms is None or lower_terms.is_constant())
            assert(upper_terms is None or upper_terms.is_constant())
            #
            if not con.equality:
                #
                # Inequality constraint
                #
                if lower_terms is None:
                    #
                    # body <= upper
                    #
                    v_domain[name, ndx] = -1
                    b_coef[name, ndx] = upper_terms.constant - body_terms.constant
                elif upper_terms is None:
                    #
                    # lower <= body
                    #
                    v_domain[name, ndx] = 1
                    b_coef[name, ndx] = lower_terms.constant - body_terms.constant
                else:
                    #
                    # lower <= body <= upper
                    #
                    # Dual for lower bound
                    #
                    name_ = name + '_lb_'
                    v_domain[name_, ndx] = 1
                    b_coef[name_, ndx] = lower_terms.constant - body_terms.constant
                    #
                    # Dual for upper bound
                    #
                    name_ = name + '_ub_'
                    v_domain[name_, ndx] = -1
                    b_coef[name_, ndx] = upper_terms.constant - body_terms.constant
            else:
                #
                # Equality constraint
                #
                v_domain[name, ndx] = 0
                b_coef[name, ndx] = lower_terms.constant - body_terms.constant
    #
    # Collect bound constraints
    #
    for name, ndx in all_vars:
        var = all_vars[name, ndx]
        #
        # Skip fixed variables
        #
        # NOTE: This shouldn't happen because of the way we collect terms
        ##if var.fixed:
        ##    continue
        #
        # Iterate over all variable indices
        #
        bounds = var.bounds
        if bounds[0] is None and bounds[1] is None:
            c_sense[name, ndx] = 'e'
        elif bounds[0] is None:
            if bounds[1] == 0.0:
                c_sense[name, ndx] = 'g'
            else:
                c_sense[name, ndx] = 'e'
                #
                # Add constraint that defines the upper bound
                #
                name_ = name + "_upper_"
                A.setdefault(name, {}).setdefault(ndx, []).append(
                    Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = -1
                b_coef[name_, ndx] = bounds[1]
        elif bounds[1] is None:
            if bounds[0] == 0.0:
                c_sense[name, ndx] = 'l'
            else:
                c_sense[name, ndx] = 'e'
                #
                # Add constraint that defines the lower bound
                #
                name_ = name + "_lower_"
                A.setdefault(name, {}).setdefault(ndx, []).append(
                    Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = 1
                b_coef[name_, ndx] = bounds[0]
        else:
            # Bounded above and below
            if bounds[0] == 0:
                c_sense[name, ndx] = 'l'
                #
                # Add constraint that defines the upper bound
                #
                name_ = name + "_upper_"
                A.setdefault(name, {}).setdefault(ndx, []).append(Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = -1
                b_coef[name_, ndx] = bounds[1]
            elif bounds[1] == 0:
                c_sense[name, ndx] = 'g'
                #
                # Add constraint that defines the lower bound
                #
                name_ = name + "_lower_"
                A.setdefault(name, {}).setdefault(ndx, []).append(Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = 1
                b_coef[name_, ndx] = bounds[0]
            else:
                c_sense[name, ndx] = 'e'
                #
                # Add constraint that defines the upper bound
                #
                name_ = name + "_upper_"
                A.setdefault(name, {}).setdefault(ndx, []).append(Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = -1
                b_coef[name_, ndx] = bounds[1]
                #
                # Add constraint that defines the lower bound
                #
                name_ = name + "_lower_"
                A.setdefault(name, {}).setdefault(ndx, []).append(Bunch(coef=1.0, var=name_, ndx=ndx))
                #
                v_domain[name_, ndx] = 1
                b_coef[name_, ndx] = bounds[0]

    #
    # Unfix the variables that were fixed
    #
    for vdata in fixed_modelvars.values():
        vdata.fixed = False

    return (A, b_coef, obj_offset, c_rhs, c_sense, d_sense, v_domain)




def create_linear_dual_from(block, fixed=None, unfixed=None):
    """
    Construct a block that represents the dual of the given block.

    The resulting block contains variables and constraints whose names are
    the dual names of the primal block.  Note that this involves a many
    string operations.  A quicker operations could be executed, but it
    would generate a dual representation that is difficult to interpret.

    Note that the dualization of a maximization problem is performed by
    negating objective and right-hand side coefficients after dualizing
    the corresponding minimization problem.  This suggestion is made
    by Dimitri Bertsimas and John Tsitsiklis in section 4.2 page 143 of
    "Introduction to Linear Optimization"

    Arguments:
        block: A Pyomo block or model
        unfixed: An iterable object with Variable and VarData values that are 
                not fixed variables.  All other variables are assumed to be fixed.
        fixed: An iterable object with Variable and VarData values that are fixed.  
                All other variables are assumed not fixed.

    Returns:
        If the block is a model object, then this returns a ConcreteModel.
        Otherwise, it returns a Block.
    """
    #
    # Collect vardata that needs to be fixed
    #
    fixed_modelvars = {}
    if fixed or unfixed:
        #
        # Collect model variables
        #
        modelvars = {}
        #
        # vardata in objectives
        #
        for obj in block.component_objects(Objective, active=True):
            for ndx in obj:
                #odata = generate_standard_repn(obj[ndx].expr, compute_values=False)
                for vdata in identify_variables(obj[ndx].expr, include_fixed=False):
                    id_ = id(vdata)
                    if not id_ in modelvars:
                        modelvars[id_] = vdata
        #
        # vardata in constraints
        #
        for con in block.component_objects(Constraint, active=True):
            for ndx in con:
                #cdata = generate_standard_repn(con[ndx].body, compute_values=False)
                for vdata in identify_variables(con[ndx].body, include_fixed=False):
                    id_ = id(vdata)
                    if not id_ in modelvars:
                        modelvars[id_] = vdata
        #
        # Fix everything that isn't specified as unfixed
        #
        if unfixed:
            unfixed_vars = set()
            for v in unfixed:
                if v.is_indexed():
                    for vardata in v.values():
                        unfixed_vars.add( id(vardata) )
                else:
                    unfixed_vars.add( id(v) )
            for id_, vdata in modelvars.items():
                if id_ not in unfixed_vars:
                    fixed_modelvars[id_] = vdata
        #
        # ... or fix everything that is specified as fixed
        #
        elif fixed:
            fixed_vars = set()
            for v in fixed:
                if v.is_indexed():
                    for vardata in v.values():
                        fixed_vars.add( id(vardata) )
                else:
                    fixed_vars.add( id(v) )
            for id_ in fixed_vars:
                if id_ in modelvars:
                    fixed_modelvars[id_] = modelvars[id_]

    A, b_coef, obj_constant, c_rhs, c_sense, d_sense, v_domain =\
                    collect_dual_representation(block, fixed_modelvars)

    #
    # Construct the block
    #
    if isinstance(block, Model):
        dual = ConcreteModel()
    else:
        dual = Block()
    dual.construct()
    _vars = {}

    # Return variable object from name and index (if applicable)
    def getvar(name, ndx=None):
        v = _vars.get((name, ndx), None)
        if v is None:
            v = Var()
            if ndx is None:
                v_name = name
            elif isinstance(ndx, tuple):
                v_name = "%s[%s]" % (name, ','.join(map(str, ndx)))
            else:
                v_name = "%s[%s]" % (name, str(ndx))
            setattr(dual, v_name, v)
            _vars[name, ndx] = v
        return v
    #
    # Construct the objective
    # The dualization of a maximization problem is handled by simply negating the
    # objective and left-hand side coefficients while keeping the dual sense.
    #
    if d_sense == minimize:
        dual.o = Objective(expr=obj_constant + sum(- b_coef[name, ndx]*getvar(name, ndx)
                                    for name, ndx in b_coef), sense=d_sense)
        rhs_multiplier = -1
    else:
        dual.o = Objective(expr=obj_constant + sum(b_coef[name, ndx]*getvar(name, ndx)
                                    for name, ndx in b_coef), sense=d_sense)
        rhs_multiplier = 1
    #
    # Construct the constraints from dual A matrix
    #
    for cname in A:
        for ndx, terms in A[cname].items():

            # Build left-hand side of constraint
            expr = 0
            for term in terms:
                expr += term.coef * getvar(term.var, term.ndx)

            #
            # Assign right-hand side coefficient
            # Note that rhs_multiplier is 1 if the dual is a maximization problem and -1 otherwise
            #
            rhsval = rhs_multiplier*c_rhs.get((cname, ndx), 0.0)

            # Using the correct inequality or equality
            if c_sense[cname, ndx] == 'e':
                e = expr - rhsval == 0
            elif c_sense[cname, ndx] == 'l':
                e = expr - rhsval <= 0
            else:
                e = expr - rhsval >= 0
            c = Constraint(expr=e)

            # Build constraint name
            if ndx is None:
                c_name = cname
            elif isinstance(ndx, tuple):
                c_name = "%s[%s]" % (cname, ','.join(map(str, ndx)))
            else:
                c_name = "%s[%s]" % (cname, str(ndx))

            # Add new constraint along with its name to the dual
            setattr(dual, c_name, c)

        # Set variable domains
        for (name, ndx), domain in v_domain.items():
            v = getvar(name, ndx)
            #flag = type(ndx) is tuple and (ndx[-1] == 'lb' or ndx[-1] == 'ub')
            if domain == 1:
                v.domain = NonNegativeReals
            elif domain == -1:
                v.domain = NonPositiveReals
            else:
                # This is possible when the variable's corresponding constraint is an equality
                v.domain = Reals

    return dual
