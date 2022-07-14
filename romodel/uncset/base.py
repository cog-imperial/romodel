from pyomo.core import ScalarBlock, ModelComponentFactory, Component
from pyomo.core import Constraint
from romodel.uncparam import UncParam


@ModelComponentFactory.register("Uncertainty set in a robust problem")
class UncSet(ScalarBlock):
    """
    This model component defines an uncertainty set in a robust optimization
    problem.
    """

    def __init__(self, *args, **kwargs):
        """Constructor"""
        _rule = kwargs.pop('rule', None)
        _fixed = kwargs.pop('fixed', None)
        _param = kwargs.pop('param', None)

        # _var = kwargs.pop('var', None)
        #
        # Initialize the ScalarBlock
        #
        kwargs.setdefault('ctype', UncSet)
        ScalarBlock.__init__(self, *args, **kwargs)
        #
        # Initialize from kwargs
        #
        self._rule = _rule
        if isinstance(_fixed, Component):
            self._fixed = [_fixed]
        else:
            self._fixed = _fixed
        if isinstance(_param, Component):
            self._param = [_param]
        else:
            self._param = _param

        self._lib = False

    def is_empty(self):
        if self._lib:
            return False
        for c in self.component_data_objects(Constraint, active=True):
            return False
        return True

    def is_lib(self):
        return self._lib

    def get_uncertain_param(self):
        param = None
        for p in self.component_objects(UncParam, active=True):
            if param is None:
                param = p
            else:
                assert param is p

    def generate_cons_from_lib(self, param):
        name = self.__class__.__name__
        raise NotImplementedError(
                "Looks like the cutting plane solver is not applicable to "
                "library set '{}'. Try 'romodel.reformulate'".format(name)
            )
