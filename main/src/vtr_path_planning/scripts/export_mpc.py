#! /usr/bin/python3

from unicycle_solver import solver as uni_solver
from acker_cycle_solver import solver as acker_solver
import shutil
import os

uni_solver.generate_dependencies("unicycle_mpc_solver.cpp", {"cpp": True, "with_header": True})
shutil.move('unicycle_mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/unicycle_mpc_solver.cpp')
shutil.move('unicycle_mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/unicycle_mpc_solver.hpp')

acker_solver.generate_dependencies("ackermann_mpc_solver.cpp", {"cpp": True, "with_header": True})
shutil.move('ackermann_mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/ackermann_mpc_solver.cpp')
shutil.move('ackermann_mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/ackermann_mpc_solver.hpp')