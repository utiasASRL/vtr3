#! /usr/bin/python3

import sys
from unicycle_solver import solver
import shutil
import os

solver.generate_dependencies("mpc_solver.cpp", {"cpp": True, "with_header": True})
print("Generated MPC files.", file=sys.stderr)
try:
    if os.path.exists('mpc_solver.cpp'):
        shutil.move(os.getcwd() + '/mpc_solver.cpp', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/mpc_solver.cpp')
    elif not os.path.exists(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/src/mpc/mpc_solver.cpp'):
        raise FileNotFoundError("An error occurred while exporting the MPC.")


    if os.path.exists('mpc_solver.h'):
        shutil.move(os.getcwd() + '/mpc_solver.h', os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/mpc_solver.hpp')
    elif not os.path.exists(os.getenv("VTRSRC") + '/main/src/vtr_path_planning/include/vtr_path_planning/mpc/mpc_solver.hpp'):
        raise FileNotFoundError("An error occurred while exporting the MPC.")

    print("Copied files successfully", file=sys.stderr)
except Exception as e:
    print(f"Unknown error. Debug info pwd: {os.getcwd()}\nls: {os.listdir(os.getcwd())}", file=sys.stderr)
    raise e
