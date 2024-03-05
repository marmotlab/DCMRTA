import sys
sys.path.insert(1, '../build/')

from TeamPlannerPy import TeamPlannerPyWrapper

param_file = '../config/mission0/planner_param.yaml'
sample_file = 'sampleLog.yaml'
log_file = 'testLog.yaml'

planner = TeamPlannerPyWrapper()
flag_form = planner.formProblem(param_file, sample_file)
flag_optimize = planner.optimize()
veh_path, task_team, task_team_dense = planner.printSolution(True)
planner.saveSolution(log_file, param_file)
print('veh_path = ', veh_path)
print('task_team = ', task_team)
