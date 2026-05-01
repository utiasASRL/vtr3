/**
 * @file daicp_qp_lib.hpp
 * @brief CasADi-based QP solver for Degeneracy-Aware ICP
 * 
 * This library implements a constrained quadratic programming solver for handling
 * degeneracy in ICP pose estimation problems using CasADi's optimization framework.
 * 
 * The core optimization problem is:
 *     min (1/2) x^T F x + f^T x
 *     s.t. -ε_i ≤ v_i^T x ≤ ε_i  for each degenerate direction v_i
 * 
 * where:
 *     F = 2 A^T W A (weighted Hessian)
 *     f = -2 A^T W b (weighted gradient)
 *     v_i = columns of Vd (pre-computed degenerate directions)
 *     ε_i = constraint bounds based on epsilon_dx per-DOF limits
 */

#pragma once

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include "vtr_logging/logging.hpp"


namespace vtr {
namespace lidar {
namespace daicp_qp {

// =================== QP Solver Result Structure ===================
struct QPSolverResult {
    bool success;
    Eigen::VectorXd x_optimal;
    double objective_value;
    double solve_time;
    std::string solver_status;
    int iterations;
    
    QPSolverResult() 
        : success(false), 
          x_optimal(Eigen::VectorXd::Zero(6)),
          objective_value(std::numeric_limits<double>::infinity()),
          solve_time(0.0),
          solver_status("not_solved"),
          iterations(0) {}
};
// =================== Helper Functions ===================
// Direct memory access for faster conversion
inline casadi::DM eigenToCasadiDM(const Eigen::MatrixXd& eigen_mat) {
    std::vector<double> data(eigen_mat.data(), eigen_mat.data() + eigen_mat.size());
    return casadi::DM(casadi::Sparsity::dense(eigen_mat.rows(), eigen_mat.cols()), data);
}

// inline Eigen::VectorXd casadiDMToEigen(const casadi::DM& casadi_vec) {
//     std::vector<double> data = casadi_vec.get_elements();
//     return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
// }
inline Eigen::VectorXd casadiDMToEigen(const casadi::DM& casadi_vec) {
    // Safe conversion: copy element by element
    const int size = casadi_vec.size1() * casadi_vec.size2();
    Eigen::VectorXd result(size);
    for (int i = 0; i < size; ++i) {
        result(i) = static_cast<double>(casadi_vec(i));
    }
    return result;
}

inline void printQPProblemInfo(const Eigen::MatrixXd& F, 
                               const Eigen::VectorXd& /* f */,
                               const Eigen::MatrixXd& Vd,
                               const Eigen::VectorXd& epsilon_dx) {
    CLOG(DEBUG, "lidar.localization_daicp") << "=== DA-ICP QP Problem Setup ===";
    CLOG(DEBUG, "lidar.localization_daicp") << "Problem size: " << F.rows() << " variables";
    CLOG(DEBUG, "lidar.localization_daicp") << "Degenerate directions: " << Vd.cols();
    CLOG(DEBUG, "lidar.localization_daicp") << "Constraint bounds (epsilon_dx):";
    CLOG(DEBUG, "lidar.localization_daicp") << "  Translation [dx,dy,dz]: [" 
        << epsilon_dx(0) << ", " << epsilon_dx(1) << ", " << epsilon_dx(2) << "] m";
    CLOG(DEBUG, "lidar.localization_daicp") << "  Rotation [dr,dp,dy]: [" 
        << epsilon_dx(3) << ", " << epsilon_dx(4) << ", " << epsilon_dx(5) << "] rad";
    CLOG(DEBUG, "lidar.localization_daicp") << "  Rotation [dr,dp,dy]: [" 
        << epsilon_dx(3) * 180.0 / M_PI << "°, " 
        << epsilon_dx(4) * 180.0 / M_PI << "°, " 
        << epsilon_dx(5) * 180.0 / M_PI << "°]";
}

// =================== Static Solver Cache ===================
// Cache multiple solvers for different problem dimensions to avoid repeated creation/destruction
struct QRQPSolverCache {
    std::map<std::pair<int, int>, casadi::Function> solvers;  // Map from (n, k) to solver
    
    casadi::Function getSolver(int n, int k, bool verbose) {
        auto key = std::make_pair(n, k);
        
        // Check if solver for this dimension already exists
        auto it = solvers.find(key);
        if (it != solvers.end()) {
            if (verbose) {
                CLOG(DEBUG, "lidar.localization_daicp") << "Reusing cached solver for n=" << n << ", k=" << k;
            }
            return it->second;
        }
        
        // Create new solver for this dimension
        if (verbose) {
            CLOG(DEBUG, "lidar.localization_daicp") << "Creating new solver for n=" << n << ", k=" << k;
        }
        
        casadi::Sparsity H_sparsity = casadi::Sparsity::dense(n, n);
        casadi::Sparsity A_sparsity = (k > 0) ? casadi::Sparsity::dense(k, n) : casadi::Sparsity(0, n);
        
        casadi::SpDict qp;
        qp["h"] = H_sparsity;
        qp["a"] = A_sparsity;
        
        casadi::Dict opts;
        opts["max_iter"] = 100;
        opts["constr_viol_tol"] = 1e-8;
        opts["dual_inf_tol"] = 1e-8;
        opts["print_problem"] = verbose;
        opts["print_header"] = verbose;
        opts["print_iter"] = verbose;
        
        casadi::Function solver = casadi::conic("qrqp_solver", "qrqp", qp, opts);
        
        // Store in cache and return
        solvers[key] = solver;
        return solver;
    }
};

static QRQPSolverCache& getQRQPCache() {
    static QRQPSolverCache cache;
    return cache;
}

// =================== Constrained QP Solver (CasADi Conic Interface) ===================
inline QPSolverResult solveConstrainedQPConic(
    const Eigen::MatrixXd& F,
    const Eigen::VectorXd& f,
    const Eigen::MatrixXd& Vd,
    const Eigen::VectorXd& epsilon_dx,
    const std::string& solver_name = "osqp",
    bool verbose = false) {
    
    QPSolverResult result;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        const int n = F.rows();
        const int k = Vd.cols();
        
        // ===== INPUT VALIDATION (defense against qrqp abort/SIGSEGV) =====
        // CasADi's qrqp plugin can hard-abort (calls std::abort, not throw)
        // when given non-finite, near-zero, or rank-deficient inputs. We
        // pre-screen here and bail out cleanly so the caller can fall back
        // to the unconstrained solution.
        if (!F.allFinite() || !f.allFinite() || !Vd.allFinite() || !epsilon_dx.allFinite()) {
            CLOG(WARNING, "lidar.localization_daicp")
                << "QP input contains non-finite values (F.finite=" << F.allFinite()
                << ", f.finite=" << f.allFinite()
                << ", Vd.finite=" << Vd.allFinite()
                << ", eps.finite=" << epsilon_dx.allFinite()
                << "), skipping constrained QP";
            result.success = false;
            result.solver_status = "input_non_finite";
            auto end_time = std::chrono::high_resolution_clock::now();
            result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
            return result;
        }
        if (n != F.cols() || n <= 0 || k < 0 || k > n) {
            CLOG(WARNING, "lidar.localization_daicp")
                << "QP input has invalid shape (F=" << F.rows() << "x" << F.cols()
                << ", k=" << k << ", n=" << n << ")";
            result.success = false;
            result.solver_status = "input_bad_shape";
            auto end_time = std::chrono::high_resolution_clock::now();
            result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
            return result;
        }

        // Ensure Hessian symmetry
        Eigen::MatrixXd H = 0.5 * (F + F.transpose());

        // Compute constraint bounds as LINEAR PROJECTION
        // For each degenerate direction v_i, the bound is: |v_i^T * epsilon_dx|
        // We take absolute value because v_i can point in either direction
        Eigen::VectorXd epsilon_c(k);
        for (int i = 0; i < k; ++i) {
            Eigen::VectorXd v_i = Vd.col(i);
            // Linear projection: |v_i^T * epsilon_dx|
            epsilon_c(i) = std::abs(v_i.dot(epsilon_dx));
        }

        // Floor extremely small bounds. Constraint widths < 1e-6 effectively
        // pin x to zero in that direction and have triggered hard aborts in
        // qrqp's active-set logic on rank-deficient problems. The caller's
        // `computeUpdateStep` fallback handles these directions correctly.
        constexpr double kMinBoundWidth = 1e-6;
        bool tightened_bounds = false;
        for (int i = 0; i < k; ++i) {
            if (!std::isfinite(epsilon_c(i)) || epsilon_c(i) < kMinBoundWidth) {
                epsilon_c(i) = kMinBoundWidth;
                tightened_bounds = true;
            }
        }
        if (tightened_bounds) {
            CLOG(WARNING, "lidar.localization_daicp")
                << "QP: clamped degenerate-direction bound(s) to " << kMinBoundWidth
                << " to avoid qrqp instability";
        }
        // ================================================================

        
        if (verbose) {
            CLOG(DEBUG, "lidar.localization_daicp") << "Constraint bounds per degenerate direction:";
            for (int i = 0; i < k; ++i) {
                CLOG(DEBUG, "lidar.localization_daicp") << "  Direction " << (i+1) 
                    << ": ±" << epsilon_c(i);
            }
            CLOG(DEBUG, "lidar.localization_daicp") << "Converting matrices to CasADi format...";
            CLOG(DEBUG, "lidar.localization_daicp") << "  H: " << H.rows() << "x" << H.cols();
            CLOG(DEBUG, "lidar.localization_daicp") << "  f: " << f.size();
            CLOG(DEBUG, "lidar.localization_daicp") << "  Vd: " << Vd.rows() << "x" << Vd.cols();
        }
        
        // Convert Eigen matrices to CasADi DM
        casadi::DM H_casadi = eigenToCasadiDM(H);
        if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "  H_casadi created";
        
        casadi::DM g_casadi = eigenToCasadiDM(f);
        if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "  g_casadi created";
        
        casadi::DM A_casadi = eigenToCasadiDM(Vd.transpose()); // Constraint matrix A = Vd^T
        if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "  A_casadi created";
        
        // Create structured QP using sparsity patterns
        if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "Creating QP structure...";
        casadi::SpDict qp;
        // OSQP requires the Hessian sparsity to be upper-triangular (it internally
        // stores only the upper triangle). Passing a full dense pattern segfaults
        // inside the plugin. For other CasADi conic plugins (qrqp, qpoases, nlpsol)
        // we keep the dense pattern, which is fastest for small problems.
        if (solver_name == "osqp") {
            qp["h"] = casadi::Sparsity::upper(n);
        } else {
            qp["h"] = H_casadi.sparsity();
        }
        qp["a"] = A_casadi.sparsity();
        
        // Solver options
        casadi::Dict opts;
        if (solver_name == "osqp") {
            casadi::Dict osqp_opts;
            osqp_opts["verbose"] = verbose;
            osqp_opts["polish"] = true;
            osqp_opts["eps_abs"] = 1e-8;
            osqp_opts["eps_rel"] = 1e-8;
            opts["osqp"] = osqp_opts;
        } else if (solver_name == "qrqp") {
            // qrqp is a QR-based active-set QP solver (pure C++, no external dependencies)
            opts["max_iter"] = 100;
            opts["constr_viol_tol"] = 1e-8;
            opts["dual_inf_tol"] = 1e-8;
        } else if (solver_name == "nlpsol") {
            // nlpsol wraps NLP solvers (like IPOPT) for the conic interface
            opts["nlpsol"] = "ipopt";  // Use IPOPT as the backend
            casadi::Dict ipopt_opts;
            ipopt_opts["ipopt.print_level"] = verbose ? 5 : 0;
            ipopt_opts["ipopt.max_iter"] = 100;
            ipopt_opts["ipopt.tol"] = 1e-6;
            ipopt_opts["ipopt.acceptable_tol"] = 1e-4;
            ipopt_opts["print_time"] = false;
            opts["nlpsol_options"] = ipopt_opts;
        } else if (solver_name == "qpoases") {
            opts["printLevel"] = verbose ? "high" : "none";
        }
        
        // Create or retrieve cached solver
        casadi::Function solver;
        if (solver_name == "qrqp") {
            // Use cached solver for qrqp to avoid repeated creation/destruction
            if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "Getting qrqp solver (n=" << n << ", k=" << k << ")...";
            solver = getQRQPCache().getSolver(n, k, verbose);
            if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "qrqp solver ready";
        } else {
            // For other solvers, create fresh instance
            if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << "Creating " << solver_name << " solver...";
            solver = casadi::conic("qp_solver", solver_name, qp, opts);
            if (verbose) CLOG(DEBUG, "lidar.localization_daicp") << solver_name << " solver created successfully";
        }
        
        // ========== Prepare Constraint Bounds ==========
        // CasADi conic interface uses the standard form:
        //   minimize:   (1/2) x^T H x + g^T x
        //   subject to: lba ≤ A*x ≤ uba    (general linear constraints)
        //               lbx ≤  x  ≤ ubx    (simple box constraints)
        //
        // Since we set A = Vd^T, the constraint "lba ≤ A*x ≤ uba" becomes:
        //   lba ≤ Vd^T*x ≤ uba
        //
        // For each degenerate direction v_i (the i-th column of Vd):
        //   lba(i) ≤ v_i^T * x ≤ uba(i)
        //
        // We want to constrain the projection of x onto each degenerate direction
        // to be within ±(projection of epsilon_dx onto that direction):
        //   -(v_i^T * epsilon_dx) ≤ v_i^T * x ≤ (v_i^T * epsilon_dx)
        //
        // Since v_i can point in either direction, we use absolute value:
        // Therefore: lba(i) = -epsilon_c(i), uba(i) = epsilon_c(i)
        // where epsilon_c(i) = |v_i^T * epsilon_dx| (computed above, always positive)
        
        // Lower and upper bounds on A*x (i.e., on Vd^T*x)
        casadi::DM lba_casadi = casadi::DM::zeros(k, 1);  // Bounds on A*x = Vd^T*x
        casadi::DM uba_casadi = casadi::DM::zeros(k, 1);  // Bounds on A*x = Vd^T*x
        for (int i = 0; i < k; ++i) {
            lba_casadi(i) = -epsilon_c(i);  // Lower bound: -(v_i^T * epsilon_dx)
            uba_casadi(i) = epsilon_c(i);   // Upper bound: +(v_i^T * epsilon_dx)
        }
        
        // Lower and upper bounds on x itself (unbounded)
        casadi::DM lbx_casadi = casadi::DM::zeros(n, 1);  // Bounds on x
        casadi::DM ubx_casadi = casadi::DM::zeros(n, 1);  // Bounds on x
        for (int i = 0; i < n; ++i) {
            lbx_casadi(i) = -std::numeric_limits<double>::infinity();  // No lower bound on x
            ubx_casadi(i) = std::numeric_limits<double>::infinity();   // No upper bound on x
        }
        
        // Solve QP problem with CasADi conic interface
        casadi::DMDict arg;
        // For OSQP we declared an upper-triangular Hessian sparsity above; the
        // numeric H must match that sparsity, otherwise the plugin will misread
        // memory and segfault.
        if (solver_name == "osqp") {
            casadi::DM H_upper = casadi::DM::zeros(casadi::Sparsity::upper(n));
            for (int j = 0; j < n; ++j) {
                for (int i = 0; i <= j; ++i) {
                    H_upper(i, j) = H(i, j);
                }
            }
            arg["h"] = H_upper;
        } else {
            arg["h"] = H_casadi;      // Hessian matrix (full / dense)
        }
        arg["g"] = g_casadi;      // Linear term
        arg["a"] = A_casadi;      // Constraint matrix A = Vd^T
        arg["lba"] = lba_casadi;  // Lower bounds on A*x (i.e., Vd^T*x)
        arg["uba"] = uba_casadi;  // Upper bounds on A*x (i.e., Vd^T*x)
        arg["lbx"] = lbx_casadi;  // Lower bounds on x (unbounded)
        arg["ubx"] = ubx_casadi;  // Upper bounds on x (unbounded)
        
        casadi::DMDict sol = solver(arg);
        
        // Extract solution
        result.x_optimal = casadiDMToEigen(sol.at("x"));
        result.objective_value = static_cast<double>(sol.at("cost"));

        // ===== POST-SOLVE SANITY CHECK =====
        // Even when the solver returns "ok", the result can be NaN/Inf on
        // ill-posed problems (esp. with active-set qrqp on near-rank-deficient
        // KKT). Treat that as failure so the caller falls back.
        if (!result.x_optimal.allFinite() || !std::isfinite(result.objective_value)) {
            CLOG(WARNING, "lidar.localization_daicp")
                << "QP returned non-finite solution (status=ok), treating as failure";
            result.success = false;
            result.solver_status = "non_finite_solution";
            result.x_optimal.setZero();
            auto end_time = std::chrono::high_resolution_clock::now();
            result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
            return result;
        }
        result.success = true;
        result.solver_status = solver_name + ": optimal";
        
        // Get solver statistics
        casadi::Dict stats = solver.stats();
        if (stats.find("iter_count") != stats.end()) {
            result.iterations = static_cast<int>(stats.at("iter_count"));
        } else if (stats.find("iterations") != stats.end()) {
            result.iterations = static_cast<int>(stats.at("iterations"));
        }
        
    } catch (const std::exception& e) {
        CLOG(ERROR, "lidar.localization_daicp") << "QP solver failed: " << e.what();
        // Also write to stderr in case the spdlog buffer is lost on later abort
        std::cerr << "[ERROR] DA-ICP QP solver threw std::exception: "
                  << e.what() << std::endl;
        result.success = false;
        result.solver_status = std::string("error: ") + e.what();
        result.x_optimal.setZero();
    } catch (...) {
        // Catch any non-std exception (e.g. raw casadi internals or
        // implementation-specific types). Without this, an unknown throw
        // type calls std::terminate -> abort.
        CLOG(ERROR, "lidar.localization_daicp")
            << "QP solver failed with unknown (non-std) exception";
        std::cerr << "[ERROR] DA-ICP QP solver threw unknown exception type"
                  << std::endl;
        result.success = false;
        result.solver_status = "error: unknown_exception";
        result.x_optimal.setZero();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
    
    return result;
}

// =================== Unconstrained QP Solver ===================
inline QPSolverResult solveUnconstrainedQP(
    const Eigen::MatrixXd& F,
    const Eigen::VectorXd& f,
    bool verbose = false) {
    
    QPSolverResult result;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Solve F x + f = 0 => x = -F^(-1) f
        Eigen::VectorXd x_optimal = -F.ldlt().solve(f);
        
        result.x_optimal = x_optimal;
        result.objective_value = (0.5 * x_optimal.transpose() * F * x_optimal)(0,0)
                                + (f.transpose() * x_optimal)(0,0);
        result.success = true;
        result.solver_status = "optimal";
        result.iterations = 1;
        
    } catch (const std::exception& e) {
        if (verbose) {
            CLOG(WARNING, "lidar.localization_daicp") 
                << "Hessian is singular, using pseudo-inverse";
        }
        
        // Use pseudo-inverse for singular F
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd x_optimal = -svd.solve(f);
        
        result.x_optimal = x_optimal;
        result.objective_value = (0.5 * x_optimal.transpose() * F * x_optimal)(0,0)
                                + (f.transpose() * x_optimal)(0,0);
        result.success = true;
        result.solver_status = "optimal_pseudoinverse";
        result.iterations = 1;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
    
    return result;
}


// =================== Main QP Solver Interface ===================
inline QPSolverResult solveDaicpQP(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& W_inv,
    const Eigen::MatrixXd& Vd,
    const Eigen::VectorXd& epsilon_dx,
    bool verbose = false) {
    
    // Input validation
    if (epsilon_dx.size() != 6) {
        CLOG(ERROR, "lidar.localization_daicp") 
            << "epsilon_dx must be size 6, got " << epsilon_dx.size();
        return QPSolverResult();
    }
    
    if (A.cols() != 6) {
        CLOG(ERROR, "lidar.localization_daicp") 
            << "A must have 6 columns, got " << A.cols();
        return QPSolverResult();
    }
    
    if (Vd.rows() != 6) {
        CLOG(ERROR, "lidar.localization_daicp") 
            << "Vd must have 6 rows, got " << Vd.rows();
        return QPSolverResult();
    }
    
    // Compute QP matrices using weighted least squares
    Eigen::MatrixXd F = 2.0 * A.transpose() * W_inv.asDiagonal() * A;  // Weighted Hessian
    Eigen::VectorXd f = -2.0 * A.transpose() * W_inv.asDiagonal() * b; // Weighted gradient
    
    if (verbose) {
        printQPProblemInfo(F, f, Vd, epsilon_dx);
    }
    
    // Check if we have any degenerate directions to constrain
    QPSolverResult result;
    
    if (Vd.cols() > 0) {
        // Solve constrained QP using OSQP via CasADi conic interface
        if (verbose) {
            CLOG(DEBUG, "lidar.localization_daicp") 
                << "Solving constrained QP using Conic (OSQP)";
        }
        result = solveConstrainedQPConic(F, f, Vd, epsilon_dx, "osqp", verbose);
        
    } else {
        // No constraints needed, solve unconstrained QP
        if (verbose) {
            CLOG(DEBUG, "lidar.localization_daicp") 
                << "No degenerate directions found, solving unconstrained problem";
        }
        result = solveUnconstrainedQP(F, f, verbose);
    }
    
    if (verbose) {
        CLOG(DEBUG, "lidar.localization_daicp") << "=== QP Solver Result ===";
        CLOG(DEBUG, "lidar.localization_daicp") << "Success: " << (result.success ? "true" : "false");
        CLOG(DEBUG, "lidar.localization_daicp") << "Status: " << result.solver_status;
        CLOG(DEBUG, "lidar.localization_daicp") << "Iterations: " << result.iterations;
        CLOG(DEBUG, "lidar.localization_daicp") << "Solve time: " << result.solve_time << " seconds";
        CLOG(DEBUG, "lidar.localization_daicp") << "Objective value: " << result.objective_value;
        CLOG(DEBUG, "lidar.localization_daicp") << "Solution: [" << result.x_optimal.transpose() << "]";
    }
    
    return result;
}


}  // namespace daicp_qp
}  // namespace lidar
}  // namespace vtr    