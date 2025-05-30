#! /usr/bin/python3

from unicycle_solver import solver as uni_solver
from acker_cycle_solver import solver as acker_solver
from bicycle_solver_rw import solver as bi_solver
import shutil
import os

def gen_mv_mpc_cpp(controller, controller_name:str, src_path:str, inc_path:str) -> None:
    controller.generate_dependencies(controller_name + '_mpc_solver.cpp', {'cpp': True, 'with_header': True})
    shutil.move(controller_name + '_mpc_solver.cpp', src_path + controller_name + '_mpc_solver.cpp')
    shutil.move(controller_name + '_mpc_solver.h', inc_path + controller_name + '_mpc_solver.hpp')


src_path = os.path.join(os.getenv("VTRSRC"), 'main/src/vtr_path_planning/src/mpc/')
if not os.path.exists(src_path):
    os.mkdir(src_path)

inc_path = os.path.join(os.getenv("VTRSRC"), 'main/src/vtr_path_planning/include/vtr_path_planning/mpc/')
if not os.path.exists(inc_path):
    os.mkdir(inc_path)

gen_mv_mpc_cpp(uni_solver, 'unicycle', src_path, inc_path)
gen_mv_mpc_cpp(acker_solver, 'ackermann', src_path, inc_path)
gen_mv_mpc_cpp(bi_solver, 'bicycle', src_path, inc_path)
