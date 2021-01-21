from pyomo.core.expr.visitor import ExpressionValueVisitor
from pyomo.core.expr.visitor import SimpleExpressionVisitor
from pyomo.core.expr.numvalue import nonpyomo_leaf_types, native_types
from romodel.uncparam import UncParam
from romodel.components import AdjustableVar


class _IsUncertainVisitor(ExpressionValueVisitor):
    def visit(self, node, values):
        return any(values)

    def visiting_potential_leaf(self, node):
        if (node.__class__ in nonpyomo_leaf_types
                or not node.is_potentially_variable()):
            return True, False

        elif not node.is_expression_type():
            # return True, isinstance(node.parent_component(), UncParam)
            return True, node.ctype is UncParam

        return False, None


def _expression_is_uncertain(node):
    """
    Return True if expression contains UncParam component.

    Args:
        node: The root node of an expression tree.
    Returns:
        True if expression contains at least one leaf of type UncParam, False
        otherwise.
    """
    visitor = _IsUncertainVisitor()
    return visitor.dfs_postorder_stack(node)


class _ParentComponentVisitor(SimpleExpressionVisitor):

    def __init__(self, types):
        self.seen = set()
        if types.__class__ is set:
            self.types = types
        else:
            self.types = set(types)

    def visit(self, node):
        if node.__class__ not in native_types:
            try:
                parent = node.parent_component()
            except AttributeError:
                return
            if parent.ctype in self.types:
                if id(parent) in self.seen:
                    return
                self.seen.add(id(parent))
                return parent


def identify_parent_components(expr, component_types):
    """
    A generator that yields a sequence of nodes
    in an expression tree that belong to a specified set.
    Args:
        expr: The root node of an expression tree.
        component_types (set or list): A set of class
            types that will be matched during the search.
    Yields:
        Each node that is found.
    """
    #
    # OPTIONS:
    # component_types - set (or list) of class types to find
    # in the expression.
    #
    visitor = _ParentComponentVisitor(component_types)
    for v in visitor.xbfs_yield_leaves(expr):
        yield v


class _IsAdjustableVisitor(ExpressionValueVisitor):
    def visit(self, node, values):
        return any(values)

    def visiting_potential_leaf(self, node):
        if (node.__class__ in nonpyomo_leaf_types
                or not node.is_potentially_variable()):
            return True, False

        elif not node.is_expression_type():
            return True, node.ctype is AdjustableVar

        return False, None


def _expression_is_adjustable(node):
    """
    Return True if expression contains UncParam component.

    Args:
        node: The root node of an expression tree.
    Returns:
        True if expression contains at least one leaf of type UncParam, False
        otherwise.
    """
    visitor = _IsAdjustableVisitor()
    return visitor.dfs_postorder_stack(node)
