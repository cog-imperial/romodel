""" Reformulation solver. """
import time
import pyutilib.misc
import pyomo.opt
from pyomo.core import TransformationFactory


@pyomo.opt.SolverFactory.register('romodel.nominal',
                                  doc='Nominal solver.')
class NominalSolver(pyomo.opt.OptSolver):
    def __init__(self, **kwargs):
        kwargs['type'] = 'romodel.nominal'
        pyomo.opt.OptSolver.__init__(self, **kwargs)
        self._metasolver = True

    def _presolve(self, *args, **kwargs):
        self._instance = args[0]
        super()._presolve(*args, **kwargs)

    def _apply_solver(self, keep_reformulation=False):
        start_time = time.time()
        instance = self._instance

        xfrm = TransformationFactory('romodel.nominal')
        xfrm.apply_to(instance)

        xfrm = TransformationFactory('romodel.adjustable.nominal')
        xfrm.apply_to(instance)

        if not self.options.solver:
            # Use glpk instead
            solver = 'gurobi'
        else:
            solver = self.options.solver

        with pyomo.opt.SolverFactory(solver) as opt:
            self.results = []
            opt.options = self.options
            results = opt.solve(instance,
                                tee=self._tee,
                                timelimit=self._timelimit)
            self.results.append(results)

        for adjvar_name in xfrm._adjvar_dict:
            adjvar = getattr(instance, adjvar_name)
            var = xfrm._adjvar_dict[adjvar_name]
            for i in adjvar:
                adjvar[i].value = var[i].value
            instance.del_component(adjvar_name + '_nominal')

        for cons_name in xfrm._cons_dict:
            cons = getattr(instance, cons_name)
            cons_nominal = self._cons_dict[cons_name]
            cons.activate()
            instance.del_component(cons_nominal.name)


        stop_time = time.time()
        self.wall_time = stop_time - start_time
        self.results_obj = self._setup_results_obj()
        #
        # Return the sub-solver return condition value and log
        #
        return pyutilib.misc.Bunch(rc=getattr(opt, '_rc', None),
                                   log=getattr(opt, '_log', None))

    def _postsolve(self):
        self._instance = None
        return self.results_obj

    def _setup_results_obj(self):
        #
        # Create a results object
        #
        results = pyomo.opt.SolverResults()
        #
        # SOLVER
        #
        solv = results.solver
        solv.name = self.options.subsolver
        solv.wallclock_time = self.wall_time
        cpu_ = []
        for res in self.results:
            if not getattr(res.solver, 'cpu_time', None) is None:
                cpu_.append(res.solver.cpu_time)
        if cpu_:
            solv.cpu_time = sum(cpu_)
        #
        # TODO: detect infeasibilities, etc
        solv.termination_condition = pyomo.opt.TerminationCondition.optimal
        #
        # PROBLEM
        #
        prob = results.problem
        prob.name = self._instance.name
        prob.number_of_constraints = self._instance.statistics.number_of_constraints
        prob.number_of_variables = self._instance.statistics.number_of_variables
        prob.number_of_binary_variables = self._instance.statistics.number_of_binary_variables
        prob.number_of_integer_variables =\
            self._instance.statistics.number_of_integer_variables
        prob.number_of_continuous_variables =\
            self._instance.statistics.number_of_continuous_variables
        prob.number_of_objectives = self._instance.statistics.number_of_objectives
        #
        # SOLUTION(S)
        #
        self._instance.solutions.store_to(results)
        return results
