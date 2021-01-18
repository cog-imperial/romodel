from romodel.visitor import identify_parent_components
from romodel import UncParam
from romodel.components import AdjustableVar


def collect_uncparam(o):
    param = list(identify_parent_components(o.expr, [UncParam]))
    assert len(param) == 1, (
            "Constraint {} should not contain more than one UncParam "
            "component".format(o.name))
    return param[0]


def collect_adjustable(o):
    param = list(identify_parent_components(o.expr, [AdjustableVar]))
    assert len(param) == 1, (
            "Constraint {} should not contain more than one AdjustableVar"
            "component".format(o.name))
    return param[0]
