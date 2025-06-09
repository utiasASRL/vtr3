#! /usr/bin/python3

from unicycle_solver import solver as uni_solver
from acker_cycle_solver import solver as acker_solver
from unicycle_follower_solver import solver as uni_solver_follower
import shutil
import os

os.makedirs(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/generated', True)
os.makedirs(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/generated', True)

uni_solver.generate_dependencies("unicycle_mpc_solver.cpp", {"cpp": True, "with_header": True})
shutil.move('unicycle_mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/generated/unicycle_mpc_solver.cpp')
shutil.move('unicycle_mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/generated/unicycle_mpc_solver.hpp')

acker_solver.generate_dependencies("ackermann_mpc_solver.cpp", {"cpp": True, "with_header": True})
shutil.move('ackermann_mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/generated/ackermann_mpc_solver.cpp')
shutil.move('ackermann_mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/generated/ackermann_mpc_solver.hpp')

uni_solver_follower.generate_dependencies("unicycle_mpc_solver_follower.cpp", {"cpp": True, "with_header": True})
shutil.move('unicycle_mpc_solver_follower.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/generated/unicycle_mpc_solver_follower.cpp')
shutil.move('unicycle_mpc_solver_follower.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/generated/unicycle_mpc_solver_follower.hpp')