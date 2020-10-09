from pyomo.environ import (Constraint,
                           Var,
                           quicksum,
                           sqrt,
                           Objective,
                           maximize,
                           minimize)
from pyomo.core import Transformation, TransformationFactory
from pyomo.repn import generate_standard_repn
from romodel.visitor import _expression_is_uncertain
from romodel.generator import RobustConstraint
from romodel.duality import create_linear_dual_from
from itertools import chain
from romodel.util import collect_uncparam
from pyomo.core.expr.visitor import replace_expressions


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

    def get_uncertain_components(self, instance, component=Constraint):
        """ Return all uncertain components of type `component`. """
        comp_list = instance.component_data_objects(component, active=True)
        for c in comp_list:
            if hasattr(c, 'body'):
                # Constraints
                expr = c.body
            else:
                # Objective
                expr = c.expr

            if _expression_is_uncertain(expr):
                yield c

    def generate_repn_param(self, instance, cdata):
        self.fix_component(instance, component=Var)
        if hasattr(cdata, 'body'):
            expr = cdata.body
        else:
            expr = cdata.expr
        repn = generate_standard_repn(expr, compute_values=False)
        self.unfix_component(component=Var)
        return repn


@TransformationFactory.register('romodel.ellipsoidal',
                                doc="Ellipsoidal Counterpart")
class EllipsoidalTransformation(BaseRobustTransformation):
    def _apply_to(self, instance, root=False):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            # Collect unc. parameter and unc. set
            param = collect_uncparam(c)
            uncset = param._uncset
            assert uncset is not None, ("No uncertainty set provided for "
                                        "uncertain parameter {}."
                                        .format(param.name))

            # Check if uncertainty set is empty
            assert not uncset.is_empty(), ("{} does not have any "
                                           "constraints.".format(uncset.name))
            # Check if uncertainty set is ellipsoidal
            if not uncset.is_ellipsoidal():
                continue

            if c.ctype is Constraint:
                assert not c.equality, (
                        "Currently can't handle equality constraints yet.")
            repn = self.generate_repn_param(instance, c)
            assert repn.is_linear(), (
                    "Constraint {} should be linear in "
                    "unc. parameters".format(c.name))

            # Generate robust counterpart
            det = quicksum(x[0]*x[1].nominal for x in zip(repn.linear_coefs,
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
            if c.ctype is Constraint:
                # For upper bound: det + padding <= b
                if c.has_ub():
                    name = c.name + '_counterpart_upper'
                    if root:
                        expr = det + sqrt(padding) <= c.upper
                        counterpart = Constraint(expr=expr)
                    else:
                        setattr(instance,
                                c.name + '_padding',
                                Var(bounds=(0, float('inf'))))
                        pvar = getattr(instance, c.name + '_padding')
                        counterpart = Constraint(expr=det + pvar <= c.upper())
                        deterministic = Constraint(expr=padding <= pvar**2)
                        setattr(instance, c.name + '_det_upper', deterministic)
                    setattr(instance, name, counterpart)
                # For lower bound: det - padding >= b
                if c.has_lb():
                    name = c.name + '_counterpart_lower'
                    if root:
                        expr = det - sqrt(padding) >= c.lower
                        counterpart = Constraint(expr=expr)
                    else:
                        setattr(instance,
                                c.name + '_padding',
                                Var(bounds=(0, float('inf'))))
                        pvar = getattr(instance, c.name + '_padding')
                        counterpart = Constraint(expr=c.lower() <= det - pvar)
                        deterministic = Constraint(expr=padding <= pvar**2)
                        setattr(instance, c.name + '_det_lower', deterministic)
                    setattr(instance, name, counterpart)
            else:
                # For minimization: min det + padding
                # For maximization: max det - padding
                sense = c.sense
                name = c.name + '_counterpart'
                if root:
                    expr = det + c.sense*sqrt(padding)
                    counterpart = Objective(expr=expr, sense=sense)
                else:
                    setattr(instance,
                            c.name + '_padding',
                            Var(bounds=(0, float('inf'))))
                    pvar = getattr(instance, c.name + '_padding')
                    counterpart = Objective(expr=det + sense*pvar, sense=sense)
                    deterministic = Constraint(expr=padding <= pvar)
                    setattr(instance,
                            c.name + '_det',
                            deterministic)
                setattr(instance, name, counterpart)
            c.deactivate()


@TransformationFactory.register('romodel.polyhedral',
                                doc="Polyhedral Counterpart")
class PolyhedralTransformation(BaseRobustTransformation):
    def _apply_to(self, instance):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            # Collect UncParam and UncSet
            param = collect_uncparam(c)
            uncset = param.uncset
            # Check if uncertainty set is empty
            assert not uncset.is_empty(), ("{} does not have any "
                                           "constraints.".format(uncset.name))
            # Check if uncertainty set is polyhedral
            if not uncset.is_polyhedral():
                continue

            if c.ctype is Constraint:
                assert not c.equality, (
                        "Currently can't handle equality constraints yet.")
            repn = self.generate_repn_param(instance, c)
            assert repn.is_linear(), (
                    "Constraint {} should be linear in "
                    "unc. parameters".format(c.name))

            # Create dual variables
            block = c.parent_block()
            setattr(block, c.name + '_dual', Var())

            # Add dual constraints d^T * v <= b, P^T * v = x
            # Constraint
            if c.ctype is Constraint:
                # LEQ
                if c.upper:
                    uncset.obj = Objective(expr=c.body, sense=maximize)
                    dual = create_linear_dual_from(uncset,
                                                   unfixed=param.values())
                    o_expr = dual.o.expr
                    del dual.o
                    dual.o = Constraint(expr=o_expr <= c.upper)
                    del uncset.obj
                    setattr(instance, c.name + '_counterpart_upper', dual)
                # GEQ
                if c.lower:
                    uncset.obj = Objective(expr=c.body, sense=minimize)
                    dual = create_linear_dual_from(uncset,
                                                   unfixed=param.values())
                    o_expr = dual.o.expr
                    del dual.o
                    dual.o = Constraint(expr=c.lower <= o_expr)
                    del uncset.obj
                    setattr(instance, c.name + '_counterpart_lower', dual)
            # Objective
            else:
                setattr(instance, c.name + '_epigraph', Var())
                epigraph = getattr(instance, c.name + '_epigraph')
                sense = c.sense
                uncset.obj = Objective(expr=c.expr, sense=-sense)
                dual = create_linear_dual_from(uncset,
                                               unfixed=param.values())
                o_expr = dual.o.expr
                del dual.o
                dual.o = Constraint(expr=sense*o_expr <= sense*epigraph)
                del uncset.obj
                setattr(instance, c.name + '_counterpart', dual)
                setattr(instance,
                        c.name + '_new',
                        Objective(expr=epigraph, sense=sense))
            # Deactivate original constraint
            c.deactivate()


@TransformationFactory.register('romodel.generators',
                                doc=("Replace uncertain constraints by"
                                     " cutting plane generators"))
class GeneratorTransformation(BaseRobustTransformation):
    """ Replace all uncertain constraints by RobustConstraint objects. """
    def _apply_to(self, instance):
        self._instance = instance
        cons = self.get_uncertain_components(instance)
        objs = self.get_uncertain_components(instance, component=Objective)

        tdata = instance._transformation_data['romodel.generators']
        tdata.generators = []

        for c in cons:
            generator = RobustConstraint()
            setattr(instance, c.name + '_generator', generator)

            generator.build(c.lower, c.body, c.upper)
            tdata.generators.append(generator)

            c.deactivate()

        for o in objs:
            generator = RobustConstraint()
            setattr(instance, o.name + '_epigraph', Var())
            epigraph = getattr(instance, o.name + '_epigraph')
            setattr(instance, o.name + '_generator', generator)

            if o.is_minimizing():
                generator.build(None, o.expr - epigraph, 0)
                setattr(instance,
                        o.name + '_new',
                        Objective(expr=epigraph, sense=minimize))
            else:
                generator.build(0, o.expr - epigraph, None)
                setattr(instance,
                        o.name + '_new',
                        Objective(expr=epigraph, sense=maximize))

            tdata.generators.append(generator)

            o.deactivate()

    def get_generator(self, c):
        pass

    def get_uncset(self, c):
        pass


@TransformationFactory.register('romodel.nominal',
                                doc="Transform robust to nominal model.")
class NominalTransformation(BaseRobustTransformation):
    def _apply_to(self, instance):
        cons = self.get_uncertain_components(instance)
        objs = self.get_uncertain_components(instance, component=Objective)

        smap = {}

        for c in cons:
            param = collect_uncparam(c)
            for i in param:
                smap[id(param[i])] = param[i].nominal
            body_nominal = replace_expressions(c.body, smap)
            c.set_value((c.lower, body_nominal, c.upper))

        for o in objs:
            param = collect_uncparam(o)
            for i in param:
                smap[id(param[i])] = param[i].nominal
            expr_nominal = replace_expressions(o.expr, smap)
            o.expr = expr_nominal


@TransformationFactory.register('romodel.unknown',
                                doc="Check for unknown uncertainty sets.")
class UnknownTransformation(BaseRobustTransformation):
    def _apply_to(self, instance):
        for c in chain(self.get_uncertain_components(instance),
                       self.get_uncertain_components(instance,
                                                     component=Objective)):
            # Collect UncParam and UncSet
            param = collect_uncparam(c)
            uncset = param.uncset
            raise RuntimeError("Cannot reformulate UncSet with unknown "
                               "geometry: {}".format(uncset.name))
