from pyomo.core import ModelComponentFactory, Component, Var
from pyomo.core.base.component import ComponentData
from pyomo.core.base.indexed_component import (IndexedComponent,
                                               UnindexedComponent_set)
from pyomo.core.base.numvalue import NumericValue
from weakref import ref as weakref_ref
from pyomo.common.timing import ConstructionTimer
from collections import defaultdict


@ModelComponentFactory.register("Uncertain param in a robust problem")
class UncParam_old(Var):
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


class _UncParamData(ComponentData, NumericValue):
    """
    This class defines the data for an uncertain parameter.

    Constructor Arguments:
        owner       The UncParam object that owns this data.

    Public Class Attributes
        value       The (worst case) value of this parameter.
        nominal     The nominal value of this parameter.
    """

    __slots__ = ('_value', '_nominal', '_fixed')

    def __init__(self, component):
        #
        # The following is equivalent to calling
        # the base ComponentData constructor.
        #
        self._component = weakref_ref(component)
        #
        # The following is equivalent to calling the
        # base NumericValue constructor.
        #
        self._value = None
        self._nominal = None
        self._fixed = False

    def __getstate__(self):
        """This method must be defined because this class uses slots."""
        state = super(_UncParamData, self).___getstate__()
        for i in _UncParamData.__slots__:
            state[i] = getattr(self, i)
        return state

    def __call__(self, exception=True):
        """ Return worst case value of this uncertain parameter."""
        return self.value

    @property
    def value(self):
        """Return the value for this uncertain parameter."""
        return self._value

    # @value.setter
    # def value(self, val):
    #     """Set the value for this uncertain parameter."""
    #     self._value = val

    @property
    def nominal(self):
        """Return the nominal value for this uncertain parameter."""
        return self._nominal

    @nominal.setter
    def nominal(self, val):
        """Set the nominal value for this uncertain parameter."""
        self._nominal = val

    @property
    def fixed(self):
        return self._fixed

    @fixed.setter
    def fixed(self, val):
        self._fixed = val

    def is_fixed(self):
        """Returns True because this value is fixed."""
        return False

    def is_potentially_variable(self):
        # TODO: this is kind of a hack to make things work with
        # generate_standard_repn.
        return True

    def is_variable_type(self):
        # TODO: this is kind of a hack to make things work with
        # generate_standard_repn.
        return True

    def is_parameter_type(self):
        """Returns True because this is an (uncertain) Parameter."""
        return True

    def is_constant(self):
        """Returns False because this is not a constant in an expression."""
        return False

    def _compute_polynomial_degree(self, result):
        """Returns 0 because this is a parameter object."""
        return 0

    def __nonzero__(self):
        """Return True if the value is defined and non-zero."""
        if self.value:
            return True
        if self.value is None:
            raise ValueError("UncParam value is undefined")
        return False

    __bool__ = __nonzero__


@ModelComponentFactory.register("Uncertain parameters.")
class UncParam(IndexedComponent):
    """An uncertain parameter value, which may be defined over an index.

    Constructor Arguments:
        name
            The name of this parameter
        index
            The index set that defines the distinct uncertain parameters.
            By default this is None, indicating that there is a single
            uncertain parameter.
        uncset
            An uncertainty set which defines which values the uncertain
            parameter can take.
        nominal
            A list of nominal values.
    """
    def __new__(cls, *args, **kwds):
        if cls != UncParam:
            return super(UncParam, cls).__new__(cls)
        if not args or (args[0] is UnindexedComponent_set and len(args) == 1):
            return SimpleUncParam.__new__(SimpleUncParam)
        else:
            return IndexedUncParam.__new__(IndexedUncParam)

    def __init__(self, *args, **kwd):
        uncset = kwd.pop('uncset', None)
        self._uncset = uncset

        kwd.setdefault('ctype', UncParam)
        IndexedComponent.__init__(self, *args, **kwd)

    def construct(self, data=None):
        """
        Initialize this component.
        """
        if self._constructed:
            return
        timer = ConstructionTimer(self)

        nom = self._nominal
        #TODO: finish this
        self._constructed = True
        timer.report()

        if not self.is_indexed():
            self._data[None] = self

        else:
            self_weakref = weakref_ref(self)
            for ndx in self._index:
                if ndx not in self._data:
                    self._data[ndx] = _UncParamData(self)
                self._data[ndx]._nominal = nom[ndx]
                self._data[ndx]._value = nom[ndx]
                self._component = self_weakref

    def _pprint(self):
        """
        Return data that will be printed for this component.
        """
        default = None  # stub
        dataGen = None  # stub
        return ([("Size", len(self)),
                 ("Index", self._index if self.is_indexed() else None),
                 ("Domain", self.domain.name),
                 ("Default", default),
                 ("Mutable", self._mutable),
                 ],
                self.sparse_iteritems(),
                ("Value",),
                dataGen,
                )


class SimpleUncParam(_UncParamData, UncParam):
    """A single uncertain parameter."""
    def __init__(self, *args, **kwd):
        _UncParamData.__init__(self, component=self)
        nominal = kwd.pop('nominal', None)
        self._nominal = nominal
        UncParam.__init__(self, *args, **kwd)


class IndexedUncParam(UncParam):
    def __init__(self, *args, **kwd):
        nominal = kwd.pop('nominal', defaultdict(lambda: None))
        self._nominal = nominal
        super().__init__(*args, **kwd)
