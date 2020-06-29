from pyomo.core.base.block import declare_custom_block, _BlockData
from pyomo.environ import SolverFactory, ConcreteModel


def cutting_plane(instance, solver):
    # Replace constraints which contain uncertain parameters by
    # RobustConstraint blocks
    xfrm = TransformationFactory('pro.robust.generators')
    xfrm.apply_to(instance)
    tdata = instance._transformation_data['pro.robust.generators']
    generators = tdata.generators
    # Solve model with deterministic values
    opt = SolverFactory(solver)
    results = opt.solver(instance)
    # Add first round of cuts to check feasibility
    feasible = {}
    for g in generators:
        feasible[g.name] = g.add_cut()
    # Keep adding cuts until feasible
    while not all(feasible.values()):
        results = opt.solve(instance)
        for g in generators:
            # Only add cut if uncertain constraint isnt already feasible
            if not feasible[g.name]:
                feasible[g.name] = g.add_cut()


def transform_to_master(instance):
    # 1. Replace uncertain constraint by RobustConstraint block
    # 2. Add uncertainty set to RobustConstraint block
    # 3. Setup separation problem on this block
    # 4. Return
    #
    # This should all be fairly similar to what Francesco suggested
    #
    # A model may have multiple uncertain constraints.
    # Each of these should be replaced by a RobustConstraint block
    # Ideally you can solve the reformulated model with the RobustConstraint
    # block as the deterministic model with any solver.
    # The cutting plane solver checks each RobustConstraint block for
    # feasibility and, if infeasible, generates a cut.
    #
    # Maybe this RobustConstraint block idea is a good idea in general?
    # How do model transformations work in terms of copying data?
    pass


@declare_custom_block(name='RobustConstraint')
class RobustConstraintData(_BlockData):
    def __init__(self, component):
        super().__init__(component)
        self._constraints = None
        self._uset = None
        self._rule = None
        self._vars = None

    def add_cut(self):
        for v in self._vars:
            v.fix()
        m = ConcreteModel()
        m.obj = Objective(expr=)
        m.c =
        opt = SolverFactory(solver)
        res = opt.solve(m)

        values = [value(m.uv[i]) for i in m.uv]
        new_constraint = self._rule(*values)
        self._constraints.add(new_constraint)
        for v in self._vars:
            v.unfix()



