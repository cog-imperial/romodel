import pyutilib.th as unittest
import pyomo.environ as pe
import romodel as ro
from romodel.util import collect_uncparam


class TestUtil(unittest.TestCase):
    def test_collect_uncparam(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam()
        m.u = ro.UncParam(range(2))
        m.x = pe.Var()
        m.c = pe.Constraint(expr=3*m.x + 4*m.w <= 1)
        m.o = pe.Objective(expr=m.x**2 + pe.sin(m.u[0]))
        self.assertIs(collect_uncparam(m.c), m.w)
        self.assertIs(collect_uncparam(m.o), m.u)
