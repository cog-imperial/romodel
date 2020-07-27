from pro.visitor import identify_parent_components
from pro import UncParam


def collect_uncparam(o):
    param = list(identify_parent_components(o.expr, [UncParam]))
    assert len(param) == 1, (
            "Constraint {} should not contain more than one UncParam "
            "component".format(o.name))
    return param[0]
