import sys
sys.dont_write_bytecode = True
import sklearn.gaussian_process as gp
from sklearn.gaussian_process.kernels import ConstantKernel, RBF, WhiteKernel
from sklearn.preprocessing import StandardScaler
import numpy as np
import os
import csv
import argparse
import pandas as pd
from numpy import cos, sin, tan

GP_INPUTS = ["x_k[0]", "x_k[1]", "x_k[2]", "v_k_m1[0]", "v_k_m1[1]", "u_k[0]", "u_k[1]", "u_k_m1[0]", "u_k_m1[1]"]
OBSERVED_STATE = ["x_k_p1[0]", "x_k_p1[1]", "x_k_p1[2]"]

parser = argparse.ArgumentParser(description="Fit a Gaussian Process model to disturbance data - one per dimensions (x, y, yaw)")
parser.add_argument("--data_file", type=str, required=True, help="Path to the CSV file containing disturbance data")
parser.add_argument("--num-restarts", type=int, default=0, help="Number of runs to perform to find optimal hyperparams")
parser.add_argument("--eval", action='store_true', help="Whether to run evaluation and plotting for manual verification after fitting")
args = parser.parse_args()


df = pd.read_csv(args.data_file)
print(df.columns)
print(df.describe())

# Default params taken for Hunter robot and associated MPC
def kinematicModel(x, u, u_km1, alpha=[0.4, 0.9], L=0.65):
  weighted_v = alpha[0]*u_km1[0] + (1-alpha[0])*u[0]
  weighted_psi = alpha[1]*u_km1[1] + (1-alpha[1])*u[1]
  theta = x[2]
  dx_kp1_bar = np.array([weighted_v*cos(theta), weighted_v*sin(theta), weighted_v/L * tan(weighted_psi)])
  return  x + dx_kp1_bar

def constructInputTargetsPairs(data):
  X = data[GP_INPUTS]

  # construct expected state from nominal model
  x_ks = data[['x_k[0]', 'x_k[1]', 'x_k[2]']].values
  u_ks = data[['u_k[0]', 'u_k[1]']].values
  u_k_m1s = data[['u_k_m1[0]', 'u_k_m1[1]']].values
  x_nxt_bar = np.array([
      kinematicModel(x, u_k, u_km1) for x, u_k, u_km1 in zip(x_ks, u_ks, u_k_m1s)
  ])
  x_nxt_true = data[OBSERVED_STATE].values
  y = x_nxt_true - x_nxt_bar     # shape (N, 3)
  y[:, 2] = np.arctan2(np.sin(y[:, 2]), np.cos(y[:, 2]))

  return X, y

def fitAndGetParams(X, y):
  kernel = ConstantKernel(1.0, (1e-3, 1e3)) * RBF(length_scale=np.ones(X.shape[1]), length_scale_bounds=(1e-2, 1e2)) \
         + WhiteKernel(noise_level=1e-2, noise_level_bounds=(1e-5, 1e1))
  model= gp.GaussianProcessRegressor(n_restarts_optimizer=args.num_restarts, kernel=kernel, normalize_y=False)
  scaler = StandardScaler()
  X_scaled = scaler.fit_transform(X)
  model.fit(X_scaled, y)
  kernel = model.kernel_
  
  product_term = kernel.k1      # ConstantKernel * RBF
  noise_term = kernel.k2        # WhiteKernel
  
  constant_kernel = product_term.k1    # ConstantKernel
  rbf_kernel = product_term.k2         # RBF
  
  sigma_f2 = constant_kernel.constant_value      # scalar, output/signal variance
  length_scales = rbf_kernel.length_scale        # array of shape (p,) if ARD, scalar if isotropic
  sigma_n2 = noise_term.noise_level              # scalar, measurement noise variancekernel_

  return {'LScale': length_scales, 'outVar' : sigma_f2, 'measVar': sigma_n2, 'scaleMu': scaler.mean_, 'scaleSig': scaler.scale_}
  
def writeModelParams(params, prefix):
  headers = params.keys()

  with open(prefix + "_model.csv", "w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=headers)
    writer.writeheader()
    writer.writerow(params)


X, y = constructInputTargetsPairs(df)
x_kernel_params = fitAndGetParams(X, y[:,0])
writeModelParams(x_kernel_params, 'x')

y_kernel_params = fitAndGetParams(X, y[:, 1])
writeModelParams(y_kernel_params, 'y')

yaw_kernel_params = fitAndGetParams(X, y[:,2])
writeModelParams(yaw_kernel_params, 'yaw')
