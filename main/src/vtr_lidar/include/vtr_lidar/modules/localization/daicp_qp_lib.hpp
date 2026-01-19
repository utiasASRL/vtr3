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

inline Eigen::VectorXd casadiDMToEigen(const casadi::DM& casadi_vec) {
    std::vector<double> data = casadi_vec.get_elements();
    return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
}
// Helper function for in-place update without reconstruction
inline void updateCasadiDM(casadi::DM& casadi_mat, const Eigen::MatrixXd& eigen_mat) {
    const double* src = eigen_mat.data();
    double* dst = casadi_mat.ptr();
    std::copy(src, src + eigen_mat.size(), dst);
}

inline void printQPProblemInfo(const Eigen::MatrixXd& F, 
                               const Eigen::VectorXd& f,
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

// =================== Cached QP Solver Class ===================
/**
 * @brief Cached QP solver that reuses solver objects across iterations
 * 
 * This class dramatically improves performance by:
 * 1. Caching the CasADi solver function (avoids expensive reconstruction)
 * 2. Pre-allocating CasADi DM matrices (avoids repeated memory allocation)
 * 3. Using in-place updates when possible
 * 
 * Usage:
 *   CachedQPSolver solver;
 *   for (int iter = 0; iter < max_iters; ++iter) {
 *       auto result = solver.solve(A, b, W_inv, Vd, epsilon_dx);
 *   }
 */
class CachedQPSolver {
private:
    // Cached solver objects
    casadi::Function osqp_solver_;
    
    // Pre-allocated matrices (updated in-place)
    casadi::DM H_casadi_;
    casadi::DM g_casadi_;
    casadi::DM A_casadi_;
    casadi::DM lba_casadi_;
    casadi::DM uba_casadi_;
    casadi::DM lbx_casadi_;
    casadi::DM ubx_casadi_;
    
    // Cache state
    int cached_n_;           // Problem size (should be 6)
    int cached_k_;           // Number of constraints
    bool osqp_initialized_;
    bool verbose_;
    
    void initializeOSQP(int n, int k) {
        if (osqp_initialized_ && cached_n_ == n && cached_k_ == k) {
            return;  // Already initialized with correct dimensions
        }
        
        // Create QP structure
        casadi::Sparsity H_sparsity = casadi::Sparsity::dense(n, n);
        casadi::Sparsity A_sparsity = (k > 0) ? casadi::Sparsity::dense(k, n) : casadi::Sparsity(0, n);
        
        casadi::SpDict qp;
        qp["h"] = H_sparsity;
        qp["a"] = A_sparsity;
        
        // OSQP options
        casadi::Dict opts;
        casadi::Dict osqp_opts;
        osqp_opts["verbose"] = verbose_;
        osqp_opts["polish"] = true;
        osqp_opts["eps_abs"] = 1e-8;
        osqp_opts["eps_rel"] = 1e-8;
        opts["osqp"] = osqp_opts;
        
        osqp_solver_ = casadi::conic("qp_solver_osqp", "osqp", qp, opts);
        
        // Pre-allocate matrices
        H_casadi_ = casadi::DM::zeros(n, n);
        g_casadi_ = casadi::DM::zeros(n, 1);
        A_casadi_ = casadi::DM::zeros(k, n);
        lba_casadi_ = casadi::DM::zeros(k, 1);
        uba_casadi_ = casadi::DM::zeros(k, 1);
        lbx_casadi_ = casadi::DM::zeros(n, 1);
        ubx_casadi_ = casadi::DM::zeros(n, 1);
        
        // Set unbounded box constraints on x (won't change)
        for (int i = 0; i < n; ++i) {
            lbx_casadi_(i) = -std::numeric_limits<double>::infinity();
            ubx_casadi_(i) = std::numeric_limits<double>::infinity();
        }
        
        cached_n_ = n;
        cached_k_ = k;
        osqp_initialized_ = true;
    }
    
public:
    CachedQPSolver(bool verbose = false) 
        : cached_n_(0), cached_k_(0), 
          osqp_initialized_(false), 
          verbose_(verbose) {}
    
    /**
     * @brief Solve QP using cached OSQP solver
     */
    QPSolverResult solveOSQP(
        const Eigen::MatrixXd& F,
        const Eigen::VectorXd& f,
        const Eigen::MatrixXd& Vd,
        const Eigen::VectorXd& epsilon_c) {
        
        QPSolverResult result;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        try {
            const int n = F.rows();
            const int k = Vd.cols();
            
            // Initialize or verify solver
            initializeOSQP(n, k);
            // Ensure Hessian symmetry
            Eigen::MatrixXd H = 0.5 * (F + F.transpose());
            // Update matrices in-place (fast!)
            updateCasadiDM(H_casadi_, H);
            updateCasadiDM(g_casadi_, f);
            if (k > 0) {
                Eigen::MatrixXd Vd_T = Vd.transpose();
                updateCasadiDM(A_casadi_, Vd_T);
                // Update constraint bounds
                for (int i = 0; i < k; ++i) {
                    lba_casadi_(i) = -epsilon_c(i);
                    uba_casadi_(i) = epsilon_c(i);
                }
            }
            // Solve
            casadi::DMDict arg;
            arg["h"] = H_casadi_;
            arg["g"] = g_casadi_;
            arg["a"] = A_casadi_;
            arg["lba"] = lba_casadi_;
            arg["uba"] = uba_casadi_;
            arg["lbx"] = lbx_casadi_;
            arg["ubx"] = ubx_casadi_;
            casadi::DMDict sol = osqp_solver_(arg);
            // Extract solution
            result.x_optimal = casadiDMToEigen(sol.at("x"));
            result.objective_value = static_cast<double>(sol.at("cost"));
            result.success = true;
            result.solver_status = "osqp: optimal";
            // Get statistics
            casadi::Dict stats = osqp_solver_.stats();
            if (stats.find("iter_count") != stats.end()) {
                result.iterations = static_cast<int>(stats.at("iter_count"));
            }
        } catch (const std::exception& e) {
            result.success = false;
            result.solver_status = std::string("osqp error: ") + e.what();
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        result.solve_time = std::chrono::duration<double>(end_time - start_time).count();
        
        return result;
    }
    
    /**
     * @brief Solve DA-ICP QP problem with caching (RECOMMENDED for iteration loops)
     * 
     * This is the high-level interface that handles:
     * - Computing QP matrices from A, b, W_inv
     * - Computing epsilon_c from Vd and epsilon_dx
     * - Calling the appropriate cached solver
     * 
     * @param A Jacobian matrix (n_correspondences × 6)
     * @param b Residual vector (n_correspondences)
     * @param W_inv Information weights (n_correspondences)
     * @param Vd Degenerate directions matrix (6 × k)
     * @param epsilon_dx Per-DOF constraint bounds (6 × 1)
     * @return QPSolverResult with solution and metadata
     */
    QPSolverResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        const Eigen::VectorXd& W_inv,
        const Eigen::MatrixXd& Vd,
        const Eigen::VectorXd& epsilon_dx) {
        
        // Compute QP matrices
        Eigen::MatrixXd F = 2.0 * A.transpose() * W_inv.asDiagonal() * A;
        Eigen::VectorXd f = -2.0 * A.transpose() * W_inv.asDiagonal() * b;
        
        // Compute constraint bounds
        const int k = Vd.cols();
        Eigen::VectorXd epsilon_c(k);
        for (int i = 0; i < k; ++i) {
            epsilon_c(i) = std::abs(Vd.col(i).dot(epsilon_dx));
        }
        if (verbose_ && k > 0) {
            CLOG(DEBUG, "lidar.localization_daicp") << "Cached solver: " << k << " degenerate directions";
        }
        // Solve using cached OSQP solver
        return solveOSQP(F, f, Vd, epsilon_c);
    }
    
    /**
     * @brief Reset cached solvers (call if problem structure changes significantly)
     */
    void reset() {
        osqp_initialized_ = false;
        cached_n_ = 0;
        cached_k_ = 0;
    }
};

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
        
        if (verbose) {
            CLOG(DEBUG, "lidar.localization_daicp") << "Constraint bounds per degenerate direction:";
            for (int i = 0; i < k; ++i) {
                CLOG(DEBUG, "lidar.localization_daicp") << "  Direction " << (i+1) 
                    << ": ±" << epsilon_c(i);
            }
        }
        
        // Convert Eigen matrices to CasADi DM
        casadi::DM H_casadi = eigenToCasadiDM(H);
        casadi::DM g_casadi = eigenToCasadiDM(f);
        casadi::DM A_casadi = eigenToCasadiDM(Vd.transpose()); // Constraint matrix A = Vd^T
        
        // Create structured QP using sparsity patterns
        casadi::SpDict qp;
        qp["h"] = H_casadi.sparsity();
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
        } else if (solver_name == "qpoases") {
            opts["printLevel"] = verbose ? "high" : "none";
        }
        
        // Create conic solver
        casadi::Function solver = casadi::conic("qp_solver", solver_name, qp, opts);
        
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
        arg["h"] = H_casadi;      // Hessian matrix
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
        result.success = false;
        result.solver_status = std::string("error: ") + e.what();
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