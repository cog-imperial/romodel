import pyutilib.th as unittest
import pyomo.environ as pe
import romodel.examples
import romodel as ro
from romodel.adjustable import (LDRAdjustableTransformation)
from pyomo.opt import check_available_solvers
from pyomo.repn import generate_standard_repn

solvers = check_available_solvers('gurobi')


class TestLDR(unittest.TestCase):
    @unittest.skipIf('gurobi' not in solvers,
                     'gurobi not available')
    def test_ldr(self):
        m = pe.ConcreteModel()
        m.w = ro.UncParam([0, 1, 2])
        m.y = ro.AdjustableVar([0, 1, 2], uncparams=[m.w])

        t = LDRAdjustableTransformation()
        t.apply_to(m)
