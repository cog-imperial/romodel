import pyomo.environ as pe
import pyutilib.th as unittest
import pro


class TestUncParam(unittest.TestCase):
    def test_simple_uncparam(self):
        m = pe.ConcreteModel()
        m.p = pro.UncParam()
        m.pnom = pro.UncParam(nominal=3)
        self.assertEqual(m.pnom.nominal, 3)
        self.assertEqual(m.p.nominal, None)
        self.assertFalse(m.p.is_constant())
        self.assertTrue(m.p.is_potentially_variable())
        self.assertTrue(m.p.is_variable_type())
        self.assertTrue(m.p.is_parameter_type())
        self.assertIs(m.p.ctype, pro.UncParam)

    def test_indexed_uncparam(self):
        m = pe.ConcreteModel()
        m.p = pro.UncParam(range(2), nominal=[3, 4])
        m.p.construct()
        self.assertEqual(m.p[0].nominal, 3)
        self.assertEqual(m.p[1].nominal, 4)
        self.assertIs(m.p[0].ctype, pro.UncParam)
        self.assertIs(m.p[1].ctype, pro.UncParam)
        self.assertIs(m.p.ctype, pro.UncParam)
        self.assertFalse(m.p[0].is_constant())
        self.assertTrue(m.p[0].is_potentially_variable())
        self.assertTrue(m.p[0].is_variable_type())
        self.assertTrue(m.p[0].is_parameter_type())
