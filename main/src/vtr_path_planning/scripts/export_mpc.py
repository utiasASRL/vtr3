#! /usr/bin/python3

from unicycle_solver import solver as uni_solver
from unicycle_follower_solver import solver as uni_solver_follower
from bicycle_solver_pt import solver as bi_solver
from bicycle_follower_solver_pt import solver as bi_solver_follower
from bicycle_follower_joint_solver_pt import solver as joint_solver
import shutil
import os

def gen_mv_mpc_cpp(controller, controller_name:str, src_path:str, inc_path:str) -> None:
    controller.generate_dependencies(controller_name + '_mpc_solver.cpp', {'cpp': True, 'with_header': True})
    shutil.move(controller_name + '_mpc_solver.cpp', src_path + controller_name + '_mpc_solver.cpp')
    shutil.move(controller_name + '_mpc_solver.h', inc_path + controller_name + '_mpc_solver.hpp')


src_path = os.path.join(os.getenv("VTRSRC"), 'main/src/vtr_path_planning/src/mpc/generated/')
if not os.path.exists(src_path):
    os.mkdir(src_path)

inc_path = os.path.join(os.getenv("VTRSRC"), 'main/src/vtr_path_planning/include/vtr_path_planning/mpc/generated/')
if not os.path.exists(inc_path):
    os.mkdir(inc_path)

gen_mv_mpc_cpp(uni_solver, 'unicycle', src_path, inc_path)
gen_mv_mpc_cpp(uni_solver_follower, 'unicycle_follower', src_path, inc_path)
gen_mv_mpc_cpp(bi_solver, 'bicycle', src_path, inc_path)
gen_mv_mpc_cpp(bi_solver_follower, 'bicycle_follower', src_path, inc_path)
gen_mv_mpc_cpp(joint_solver, 'bicycle_joint', src_path, inc_path)
