import pyomo.environ as pe
import pyutilib.th as unittest
import romodel as ro


class TestUncParam(unittest.TestCase):
    def test_simple_uncparam(self):
        m = pe.ConcreteModel()
        m.p = ro.UncParam()
        m.pnom = ro.UncParam(nominal=3)
        self.assertEqual(m.pnom.nominal, 3)
        self.assertEqual(m.p.nominal, None)
        self.assertFalse(m.p.is_constant())
        self.assertTrue(m.p.is_potentially_variable())
        self.assertTrue(m.p.is_variable_type())
        self.assertTrue(m.p.is_parameter_type())
        self.assertIs(m.p.ctype, ro.UncParam)

    def test_indexed_uncparam(self):
        m = pe.ConcreteModel()
        m.p = ro.UncParam(range(2), nominal=[3, 4])
        m.p.construct()
        self.assertEqual(m.p[0].nominal, 3)
        self.assertEqual(m.p[1].nominal, 4)
        self.assertIs(m.p[0].ctype, ro.UncParam)
        self.assertIs(m.p[1].ctype, ro.UncParam)
        self.assertIs(m.p.ctype, ro.UncParam)
        self.assertFalse(m.p[0].is_constant())
        self.assertTrue(m.p[0].is_potentially_variable())
        self.assertTrue(m.p[0].is_variable_type())
        self.assertTrue(m.p[0].is_parameter_type())

    def test_bounds(self):
        m = pe.ConcreteModel()
        m.p = ro.UncParam(range(2), nominal=[3, 4], bounds=(0, 1))
        m.p.construct()
        self.assertEqual(m.p[0].lb, 0)
        self.assertEqual(m.p[0].ub, 1)
        self.assertEqual(m.p[1].lb, 0)
        self.assertEqual(m.p[1].ub, 1)
        m.p[0].setlb(-1)
        self.assertEqual(m.p[0].lb, -1)
        m.p[1].setub(2)
        self.assertEqual(m.p[1].ub, 2)
