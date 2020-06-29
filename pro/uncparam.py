from pyomo.core import ModelComponentFactory, Component, Var


@ModelComponentFactory.register("Uncertain param in a robust problem")
class UncParam(Var):
    # def __new__(cls, *args, **kwds):
    #     if cls != UncParam:
    #         return super(UncParam, cls).__new__(cls)
    #     if not args or (args[0] is UnindexedComponent_set and len(args)==1):
    #         return SimpleVar.__new__(SimpleVar)
    #     else:
    #         return IndexedVar.__new__(IndexedVar)

    def __init__(self, *args, **kwargs):
        """Constructor"""
        #
        # Collect kwargs for SubModel
        #
        _rule = kwargs.pop('rule', None)
        _fixed = kwargs.pop('fixed', None)
        _uncset = kwargs.pop('uncset', None)
        _nominal = kwargs.pop('nominal', None)

        # _var = kwargs.pop('var', None)
        #
        # Initialize the SimpleBlock
        #
        kwargs.setdefault('ctype', UncParam)
        Var.__init__(self, *args, **kwargs)
        #
        # Initialize from kwargs
        #
        self._rule = _rule
        if isinstance(_fixed, Component):
            self._fixed = [_fixed]
        else:
            self._fixed = _fixed
        self._uncset = _uncset
        self._nominal = _nominal
        # if isinstance(_var, Component):
        #     self._var = [_var]
        # else:
        #     self._var = _var

    @property
    def uncset(self):
        return self._uncset

    @uncset.setter
    def uncset(self, uncset):
        # TODO: refactor uncset and add assert uncset.type() is UncSet here
        self._uncset = uncset

    @property
    def nominal(self):
        return self._nominal
