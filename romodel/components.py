from pyomo.core.base.component import ComponentData
from pyomo.core.base.numvalue import NumericValue
from weakref import ref as weakref_ref
from pyomo.core import ModelComponentFactory
from pyomo.core.base.indexed_component import (IndexedComponent,
                                               UnindexedComponent_set)
from pyomo.common.timing import ConstructionTimer
from collections import defaultdict


class _AdjustableVarData(ComponentData, NumericValue):
    """
    This class defines the data for an uncertain parameter.

    Constructor Arguments:
        owner       The UncParam object that owns this data.

    Public Class Attributes
        value       The (worst case) value of this parameter.
        nominal     The nominal value of this parameter.
    """

    __slots__ = ('_value', '_nominal', '_fixed', '_bounds')

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
        self._bounds = (None, None)

    def __getstate__(self):
        """This method must be defined because this class uses slots."""
        state = super(_AdjustableVarData, self).___getstate__()
        for i in _AdjustableVarData.__slots__:
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

    @property
    def bounds(self):
        return self._bounds

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
        return False

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


@ModelComponentFactory.register("Adjustable variable in a robust problem")
class AdjustableVar(IndexedComponent):
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
        if cls != AdjustableVar:
            return super(AdjustableVar, cls).__new__(cls)
        if not args or (args[0] is UnindexedComponent_set and len(args) == 1):
            return SimpleAdjustableVar.__new__(SimpleAdjustableVar)
        else:
            return IndexedAdjustableVar.__new__(IndexedAdjustableVar)

    def __init__(self, *args, **kwd):
        uncparams = kwd.pop('uncparams', None)
        self._uncparams = uncparams

        kwd.setdefault('ctype', AdjustableVar)
        IndexedComponent.__init__(self, *args, **kwd)

    def construct(self, data=None):
        """
        Initialize this component.
        """
        if self._constructed:
            return
        timer = ConstructionTimer(self)

        nom = self._nominal

        if not self.is_indexed():
            self._data[None] = self

        else:
            self_weakref = weakref_ref(self)
            for ndx in self._index:
                if ndx not in self._data:
                    self._data[ndx] = _AdjustableVarData(self)
                self._data[ndx]._nominal = nom[ndx]
                self._data[ndx]._value = nom[ndx]
                self._component = self_weakref

        self._constructed = True
        timer.report()

    def _pprint(self):
        """
        Return data that will be printed for this component.
        """
        dataGen = None  # stub

        def dataGen(i, x):
            return (x.nominal, x.value)

        return ([("Size", len(self)),
                 ("Index", self._index if self.is_indexed() else None),
                 ],
                self.iteritems(),
                ("Nominal", "Value"),
                dataGen
                )


class SimpleAdjustableVar(_AdjustableVarData, AdjustableVar):
    """A single uncertain parameter."""
    def __init__(self, *args, **kwd):
        _AdjustableVarData.__init__(self, component=self)
        nominal = kwd.pop('nominal', None)
        self._nominal = nominal
        AdjustableVar.__init__(self, *args, **kwd)


class IndexedAdjustableVar(AdjustableVar):
    def __init__(self, *args, **kwd):
        nominal = kwd.pop('nominal', defaultdict(lambda: None))
        self._nominal = nominal
        super().__init__(*args, **kwd)
