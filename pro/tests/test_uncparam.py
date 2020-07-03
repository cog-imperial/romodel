import pyomo.environ as pe
import pyutilib.th as unittest
import pro


class TestUncParam(unittest.TestCase):
    def test_simple_uncparam(self):
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.pnom = pro.UncParam(nominal=3)
        self.assertEqual(m.pnom.nominal, 3)

    def test_indexed_uncparam(self):
        m = pe.ConcreteModel()
        m.p = pro.UncParam(range(2), nominal=[3, 4])
        m.p.construct()
        self.assertEqual(m.p[0].nominal, 3)
        self.assertEqual(m.p[1].nominal, 4)
