from pyomo.environ import (Constraint,
                           Var,
                           Objective,
                           maximize,
                           minimize)
from pyomo.core import Transformation, TransformationFactory
from pyomo.repn import generate_standard_repn
from romodel.visitor import _expression_is_uncertain
from romodel.generator import RobustConstraint
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
