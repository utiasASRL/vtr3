#! /usr/bin/python3

from unicycle_solver import solver
import shutil
import os

solver.generate_dependencies("mpc_solver.cpp", {"cpp": True, "with_header": True})
if os.path.exists('mpc_solver.cpp'):
    shutil.move('mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/mpc_solver.cpp')
elif not os.path.exists(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/mpc_solver.cpp'):
    raise FileNotFoundError("An error occurred while exporting the MPC.")

if os.path.exists('mpc_solver.h'):
    shutil.move('mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/mpc_solver.hpp')
elif not os.path.exists(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/mpc_solver.hpp'):
    raise FileNotFoundError("An error occurred while exporting the MPC.")