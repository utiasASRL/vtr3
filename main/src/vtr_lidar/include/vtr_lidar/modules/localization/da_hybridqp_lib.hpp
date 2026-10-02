#pragma once

// HybridQP: hybrid point-to-point / point-to-plane DA-ICP with a CasADi-constrained QP
// step in degenerate directions.
// Method-specific DA-ICP functions, reproduced verbatim from branch aeva_warthog_qp.
// Functions shared by all methods live in da_lib.hpp (namespace da_lib); they
// are visible here through the enclosing namespace.
// Parameter ordering is [translation, orientation].

#include "vtr_lidar/modules/localization/da_lib.hpp"
#include <casadi/casadi.hpp>
#include <map>
#include <chrono>
#include <cmath>

namespace vtr {
namespace lidar {

class LocalizationDAICPModule;

namespace da_lib {
namespace hybridqp {

// =================== CasADi QP solver (formerly daicp_qp_lib.hpp) ===================
/**
 * @file da_hybridqp_lib.hpp (QP solver section)
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


// =================== Print Functions ===================
inline void printCovarianceInfo(const Eigen::MatrixXd& daicp_cov) {
  const Eigen::VectorXd diagonal = daicp_cov.diagonal();
  const Eigen::VectorXd std_dev = diagonal.cwiseSqrt();
  
  CLOG(DEBUG, "lidar.localization_daicp") << "Final covariance P diagonal (roll, pitch, yaw, x, y, z): [" << diagonal.transpose() << "]";
  CLOG(DEBUG, "lidar.localization_daicp") << "Final std (roll, pitch, yaw, x, y, z): [" << std_dev.transpose() << "]";
}

// =================== Block Scaling Functions ===================
inline std::pair<Eigen::Matrix3d, Eigen::Matrix3d> schurComplementMarginalization(const Eigen::MatrixXd& H) {
  // Apply Schur complement marginalization to obtain marginalized information matrices
  // H is 6x6 with [translation, orientation] ordering:
  // H = [H_tt,           H_t_theta; 
  //      H_theta_t,      H_theta_theta]
  
  // Extract blocks 
  Eigen::Matrix3d H_tt = H.block<3, 3>(0, 0);           // translation block
  Eigen::Matrix3d H_t_theta = H.block<3, 3>(0, 3);      // translation-rotation block
  Eigen::Matrix3d H_theta_t = H.block<3, 3>(3, 0);      // rotation-translation block
  Eigen::Matrix3d H_theta_theta = H.block<3, 3>(3, 3);  // rotation block
  
  const double reg_val = 1e-12;
  
  // Marginalized rotation information: H_marg_theta = H_theta_theta - H_theta_t * H_tt^{-1} * H_t_theta
  Eigen::Matrix3d H_marg_theta;
  try {
    Eigen::Matrix3d H_tt_inv = (H_tt + reg_val * Eigen::Matrix3d::Identity()).inverse();
    H_marg_theta = H_theta_theta - H_theta_t * H_tt_inv * H_t_theta;
  } catch (const std::exception& e) {
    // Use pseudo-inverse if singular
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_tt + reg_val * Eigen::Matrix3d::Identity(), 
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d H_tt_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
    H_marg_theta = H_theta_theta - H_theta_t * H_tt_pinv * H_t_theta;
  }
  
  // Marginalized translation information: H_marg_t = H_tt - H_t_theta * H_theta_theta^{-1} * H_theta_t
  Eigen::Matrix3d H_marg_t;
  try {
    Eigen::Matrix3d H_theta_theta_inv = (H_theta_theta + reg_val * Eigen::Matrix3d::Identity()).inverse();
    H_marg_t = H_tt - H_t_theta * H_theta_theta_inv * H_theta_t;
  } catch (const std::exception& e) {
    // Use pseudo-inverse if singular
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_theta_theta + reg_val * Eigen::Matrix3d::Identity(), 
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d H_theta_theta_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
    H_marg_t = H_tt - H_t_theta * H_theta_theta_pinv * H_theta_t;
  }
  
  // Ensure positive semidefinite
  // H_marg_theta = makePSD(H_marg_theta);
  // H_marg_t = makePSD(H_marg_t);
  
  return std::make_pair(H_marg_theta, H_marg_t);
}

inline double computeScalingFactorTrace(const Eigen::Matrix3d& H_marg_theta, const Eigen::Matrix3d& H_marg_t) {
  // Compute scaling factor using the trace method
  double tr_theta = H_marg_theta.trace();
  double tr_t = H_marg_t.trace();

  if (!std::isfinite(tr_theta) || !std::isfinite(tr_t) ||
      tr_t < 1e-12 || tr_theta < 1e-12) {
    std::cout << "[WARNING] computeScalingFactorTrace: degenerate marginal Hessian "
              << "(tr_theta=" << tr_theta << ", tr_t=" << tr_t
              << "), using default scaling" << std::endl;
    return 20.0; // mid-range for 40 meter lidar
  }

  return std::sqrt(tr_theta / tr_t);
}

inline double computeScalingFactorMax(const Eigen::Matrix3d& H_marg_theta,
                                      const Eigen::Matrix3d& H_marg_t)
{
    constexpr double reg_val = 1e-12;
    constexpr double kDefaultScale = 20.0; // mid-range for 40 meter lidar

    // Reject non-finite inputs up front (Schur complement of a near-singular
    // block can produce inf/NaN entries before we even get to the eigensolver).
    if (!H_marg_theta.allFinite() || !H_marg_t.allFinite()) {
        std::cout << "[WARNING] computeScalingFactorMax: non-finite marginal Hessian, "
                  << "using default scaling\n";
        return kDefaultScale;
    }

    // Add tiny diagonal regularization (fast: avoids a new matrix allocation)
    Eigen::Matrix3d H_theta = H_marg_theta;
    H_theta.diagonal().array() += reg_val;

    Eigen::Matrix3d H_t = H_marg_t;
    H_t.diagonal().array() += reg_val;

    // Fastest 3x3 symmetric eigen decomposition in Eigen
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_theta(H_theta, Eigen::EigenvaluesOnly);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_t(H_t, Eigen::EigenvaluesOnly);

    if (solver_theta.info() != Eigen::Success || solver_t.info() != Eigen::Success) {
        std::cout << "[WARNING] computeScalingFactorMax: eigensolver failed, "
                  << "using default scaling\n";
        return kDefaultScale;
    }

    // Eigenvalues are sorted in ASCENDING order: [λ_min, λ_mid, λ_max]
    const double max_theta = solver_theta.eigenvalues()(2);
    const double max_t     = solver_t.eigenvalues()(2);

    // Guard against degenerate / non-PSD marginalized Hessians.
    // If either max-eigenvalue is non-positive (Schur complement of a near-singular
    // block can lose PSD-ness due to floating-point), or if the translation
    // information is vanishingly small, fall back to a sane default rather than
    // returning NaN (which propagates through D_inv -> H_scaled -> QP and crashes
    // the solver later, see da_lib.hpp::computeThreshold and downstream QP).
    if (max_t < 1e-12 || max_theta < 1e-12 ||
        !std::isfinite(max_theta) || !std::isfinite(max_t)) {
        std::cout << "[WARNING] computeScalingFactorMax: degenerate marginal Hessian "
                  << "(max_theta=" << max_theta << ", max_t=" << max_t
                  << "), using default scaling\n";
        return kDefaultScale;
    }

    const double scale = std::sqrt(max_theta / max_t);
    if (!std::isfinite(scale)) {
        std::cout << "[WARNING] computeScalingFactorMax: non-finite scale "
                  << scale << ", using default scaling\n";
        return kDefaultScale;
    }
    return scale;
}

// ======= compute the adaptive total number of rows ======= 
inline int computeTotalRows(const std::vector<std::pair<size_t, size_t>>& sample_inds,
                            const std::vector<bool>& high_curv_match) {
  if (high_curv_match.empty() || high_curv_match.size() != sample_inds.size()) {
    // All point-to-plane: 1 row per correspondence
    return sample_inds.size();
  }
  
  int total_rows = 0;
  for (size_t i = 0; i < sample_inds.size(); ++i) {
    total_rows += high_curv_match[i] ? 3 : 1;  // 3 rows for P2P, 1 row for P2Plane
  }
  return total_rows;
}

// =================== Covariance Computation ===================
inline Eigen::Matrix<double, 6, 6> makePD(const Eigen::Matrix<double, 6, 6>& cov, 
                                          double min_eigenvalue = 1e-6) {
    // Check if matrix is already positive definite using Cholesky decomposition
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(cov);
    if (llt.info() == Eigen::Success) {
        return cov;  // Already positive definite
    }

    // Try LDLT decomposition (handles semi-definite cases)
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(cov);
    if (ldlt.info() == Eigen::Success) {
        // Regularize by clamping very small or negative diagonal elements of D
        Eigen::Matrix<double, 6, 1> D = ldlt.vectorD();
        bool modified = false;
        for (int i = 0; i < D.size(); ++i) {
            if (D(i) < min_eigenvalue) {
                D(i) = min_eigenvalue;
                modified = true;
            }
        }
        if (modified)
            CLOG(DEBUG, "lidar.localization_daicp") << "Regularized LDLT diagonal values";

        // Reconstruct: Eigen's LDLT satisfies  P * A * P^T = L * D * L^T
        // Therefore                              A = P^T * (L * D * L^T) * P
        // We rebuild with the clamped D.
        const Eigen::Matrix<double, 6, 6> L =
            ldlt.matrixL().toDenseMatrix();             // unit lower triangular
        const Eigen::Matrix<double, 6, 6> LDLt =
            L * D.asDiagonal() * L.transpose();         // L * D * L^T

        // Apply the (inverse) permutation:  cov_pd = P^T * LDLt * P
        Eigen::Matrix<double, 6, 6> cov_pd = LDLt;
        cov_pd = ldlt.transpositionsP().transpose() * cov_pd;  // P^T * LDLt
        cov_pd = cov_pd * ldlt.transpositionsP();              // (P^T * LDLt) * P

        // Symmetrize to remove tiny numerical asymmetry
        cov_pd = 0.5 * (cov_pd + cov_pd.transpose()).eval();
        return cov_pd;
    } 

    // Use eigenvalue decomposition to fix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(cov);
    if (eigensolver.info() != Eigen::Success) {
        CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed, using diagonal regularization";
        return cov + min_eigenvalue * Eigen::Matrix<double, 6, 6>::Identity();
    }
    
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix<double, 6, 6> eigenvectors = eigensolver.eigenvectors();
    
    // Clamp negative eigenvalues
    bool modified = false;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < min_eigenvalue) {
            eigenvalues(i) = min_eigenvalue;
            modified = true;
        }
    }
    
    if (modified) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Fixed " << eigenvalues.size() << " negative eigenvalues";
    }
    
    // Reconstruct the matrix
    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

// =================== point-to-plane Jacobian Computation ===================
inline Eigen::VectorXd computeP2PlaneJacobian(
    const Eigen::Vector3d& source_point, 
    const Eigen::Vector3d& target_normal) {
  
  // Ensure normal is unit vector
  const Eigen::Vector3d n = target_normal.normalized();
  const Eigen::Vector3d ps = source_point;
  
  // Jacobian with respect to rotation parameters 
  const Eigen::Vector3d cross_x(0, -ps[2], ps[1]);   // ∂(R*p)/∂rx
  const Eigen::Vector3d cross_y(ps[2], 0, -ps[0]);   // ∂(R*p)/∂ry  
  const Eigen::Vector3d cross_z(-ps[1], ps[0], 0);   // ∂(R*p)/∂rz
  
  // Rotation Jacobian: dot product with normal
  Eigen::Vector3d rotation_jacobian;
  rotation_jacobian[0] = cross_x.dot(n);  // ∂e/∂rx
  rotation_jacobian[1] = cross_y.dot(n);  // ∂e/∂ry
  rotation_jacobian[2] = cross_z.dot(n);  // ∂e/∂rz
  
  // Translation Jacobian: normal vector
  const Eigen::Vector3d translation_jacobian = n;
  
  // Combine into 6D vector [translation_jacobian, rotation_jacobian]
  Eigen::VectorXd jacobian(6);
  jacobian.head<3>() = translation_jacobian;
  jacobian.tail<3>() = rotation_jacobian;
  return jacobian;
}

inline Eigen::Matrix<double, 3, 6> computeP2PointJacobian(
    const Eigen::Vector3d& source_point) {

  const Eigen::Vector3d ps = source_point;
  // ===== Compute Jacobian =====    
  // Jacobian for translation: ∂e/∂t = I (3×3 identity)
  // Jacobian for rotation: ∂e/∂ω = -[ps]×
  const Eigen::Matrix3d neg_skew = -skewSymmetric(ps);

  // Build full Jacobian: 3×6 matrix
  // [∂e/∂tx, ∂e/∂ty, ∂e/∂tz, ∂e/∂ωx, ∂e/∂ωy, ∂e/∂ωz]
  Eigen::Matrix<double, 3, 6> jacobian;
  jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();   // Translation part
  jacobian.block<3, 3>(0, 3) = neg_skew;                      // Rotation part

  return jacobian;
}

// =================== Unified Computation Function ===================
inline void computeJacobianResidualInformation(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const std::vector<bool>& high_curv_match,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    const Eigen::Matrix4d& T_combined,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b,
    Eigen::VectorXd& W_inv,
    const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_) {
  
  // Compute total rows needed
  const int total_rows = computeTotalRows(sample_inds, high_curv_match);
  
  // Resize matrices
  A.resize(total_rows, 6);
  b.resize(total_rows);
  W_inv.resize(total_rows);
  
  // Check if adaptive mode is enabled
  const bool use_adaptive = !high_curv_match.empty() && 
                            (high_curv_match.size() == sample_inds.size());
  
  if (use_adaptive) {
    const int high_curv_count = std::count(high_curv_match.begin(), high_curv_match.end(), true);
    CLOG(INFO, "lidar.localization_daicp") << "Adaptive mode: " << high_curv_count
                                            << " P2Point, " << (sample_inds.size() - high_curv_count)
                                            << " P2Plane, total rows: " << total_rows;
  }
  
  // ==== Noise model parameters ==== //
  const double sigma_p2p = 0.05;  // point-to-point noise std (meters)
  const double w_inv_p2p = 1.0 / (sigma_p2p * sigma_p2p);
  
  const double sigma_d = config_->sigma_d;
  const double sigma_az = config_->sigma_az;
  const double sigma_el = config_->sigma_el;
  const Eigen::Matrix3d Sigma_du = Eigen::Vector3d(sigma_d * sigma_d,
                                                   sigma_az * sigma_az,
                                                   sigma_el * sigma_el).asDiagonal();
  
  const Eigen::Matrix3d R_combined = T_combined.block<3, 3>(0, 0);
  
  // ===== CRITICAL: Pre-compute row offsets (NOT thread-safe, must be sequential) =====
  std::vector<int> row_offsets(sample_inds.size());
  int current_row = 0;
  for (size_t i = 0; i < sample_inds.size(); ++i) {
    row_offsets[i] = current_row;
    current_row += (use_adaptive && high_curv_match[i]) ? 3 : 1;
  }
  
  // ===== Fill matrices (SEQUENTIAL - cannot parallelize with variable row sizes) =====
  // NOTE: OpenMP parallel for is NOT safe here because:
  // 1. Row indices are data-dependent
  // 2. Multiple threads would write to overlapping/wrong rows
  // 3. The row_offsets computation itself is sequential
  
  for (size_t i = 0; i < sample_inds.size(); ++i) {
    const auto& ind = sample_inds[i];
    const int row_start = row_offsets[i];  // Starting row for this correspondence
    
    // Get points
    const Eigen::Vector3d source_pt_original = query_mat.block<3, 1>(0, ind.first).cast<double>();
    const Eigen::Vector4d source_pt_hom(source_pt_original(0), source_pt_original(1), 
                                       source_pt_original(2), 1.0);
    const Eigen::Vector3d source_pt_transformed = (T_combined * source_pt_hom).head<3>();
    
    const Eigen::Vector3d target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d n_t = target_normal.normalized();
    
    // Determine correspondence type
    const bool is_high_curv = use_adaptive && high_curv_match[i];
    
    if (is_high_curv) {
      // ===== Point-to-Point: 3D residual (fills 3 rows) =====
      Eigen::Matrix<double, 3, 6> jac_p2p = computeP2PointJacobian(source_pt_transformed);
      
      // Fill 3 rows in A
      A.block<3, 6>(row_start, 0) = jac_p2p;
      
      // Compute 3D residual
      Eigen::Vector3d residual_3d = source_pt_transformed - target_pt;
      // Store negative residual: b = -residual = -(ps - pt) 
      b.segment<3>(row_start) = - residual_3d;
      
      // Set uniform weights for all 3 components
      W_inv.segment<3>(row_start).setConstant(w_inv_p2p);
      
    } else {
      // ===== Point-to-Plane: 1D residual (fills 1 row) =====
      Eigen::VectorXd jacobian = computeP2PlaneJacobian(source_pt_transformed, n_t);
      A.row(row_start) = jacobian.transpose();
      
      // Compute 1D residual
      b(row_start) = n_t.dot(target_pt - source_pt_transformed);
      
      // Compute range/bearing covariance
      const double range = source_pt_original.norm();
      const double azimuth = std::atan2(source_pt_original(1), source_pt_original(0));
      const double elevation = std::atan2(source_pt_original(2), 
                               std::sqrt(source_pt_original(0) * source_pt_original(0) + 
                                        source_pt_original(1) * source_pt_original(1)));
      const Eigen::Matrix3d Sigma_pL_i = computeRangeBearingCovariance(range, azimuth, elevation, Sigma_du);
      
      // Compute variance components
      const Eigen::RowVector3d J_p = n_t.transpose() * R_combined;
      const Eigen::RowVector3d J_q = -n_t.transpose();
      const Eigen::Vector3d d = source_pt_transformed - target_pt;
      const Eigen::RowVector3d J_n = d.transpose();
      
      const double sigma_map = 0.01;
      const Eigen::Matrix3d Sigma_q_i = (sigma_map * sigma_map) * Eigen::Matrix3d::Identity();
      
      const double sigma_n = 0.03;
      const Eigen::Matrix3d Pn = Eigen::Matrix3d::Identity() - n_t * n_t.transpose();
      const Eigen::Matrix3d Sigma_n_i = (sigma_n * sigma_n) * Pn;
      
      // Total variance
      double cov_r = (J_p * Sigma_pL_i * J_p.transpose())(0, 0) +
                     (J_q * Sigma_q_i * J_q.transpose())(0, 0) +
                     (J_n * Sigma_n_i * J_n.transpose())(0, 0);
      
      const double sigma_floor = 0.01;
      cov_r = std::max(cov_r, sigma_floor * sigma_floor);
      
      double w = 1.0 / cov_r;
      const double w_cap = 1.0e6;
      W_inv(row_start) = std::min(w, w_cap);
    }
  }
}

inline Eigen::VectorXd computeUpdateStep(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& W_inv,
    const Eigen::Matrix<double, 6, 6>& V,
    const Eigen::Matrix<double, 6, 6>& Vf)
{
    // Solve the WEIGHTED normal equations consistent with the QP cost:
    //   min_x  0.5 * (A x - b)^T W_inv (A x - b)
    //   =>     (A^T W_inv A) Δx = A^T W_inv b
    // Note: previously this routine ignored W_inv, which made the unconstrained
    // branch optimize a different cost than the constrained (QP) branch.
    const Eigen::MatrixXd AtW = A.transpose() * W_inv.asDiagonal();
    const Eigen::MatrixXd H   = AtW * A;
    const Eigen::VectorXd Atb = AtW * b;

    Eigen::VectorXd delta_x_f;
    // LDLT is numerically more forgiving than LLT for near-singular Hessians.
    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() == Eigen::Success) {
        delta_x_f = ldlt.solve(Atb);
    } else {
        // LAST-RESORT fallback: weighted SVD on whitened system
        // sqrt(W) * A * x = sqrt(W) * b
        const Eigen::VectorXd sqrtW = W_inv.cwiseSqrt();
        const Eigen::MatrixXd Aw = sqrtW.asDiagonal() * A;
        const Eigen::VectorXd bw = sqrtW.asDiagonal() * b;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Aw, Eigen::ComputeThinU | Eigen::ComputeThinV);
        delta_x_f = svd.solve(bw);
    }

    // Project onto well-conditioned subspace (solution remapping)
    return V * (Vf.transpose() * delta_x_f);
}

inline bool daGaussNewton(
  const std::vector<std::pair<size_t, size_t>>& sample_inds,
  const std::vector<bool>& high_curv_match,
  const Eigen::Matrix4Xf& query_mat,
  const Eigen::Matrix4Xf& map_mat,
  const Eigen::Matrix4Xf& map_normals_mat,
  steam::se3::SE3StateVar::Ptr T_var,
  const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_,
  const Eigen::Matrix<double, 6, 6>& prior_cov,
  Eigen::Matrix<double, 6, 6>& daicp_cov  ) {

  if (sample_inds.size() < 6) {
    CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
    return false;
  }
  // start with identity transformation for the Gauss-Newton process
  lgmath::se3::Transformation current_transformation = lgmath::se3::Transformation(); // Identity
  Eigen::VectorXd accumulated_params = Eigen::VectorXd::Zero(6);
  // get initial transformation from T_var to apply later
  const lgmath::se3::Transformation initial_T_var = T_var->value();
  // variables for covergence tracking
  double prev_cost = std::numeric_limits<double>::max();
  double curr_cost = 0.0;
  bool converged = false;
  std::string termination_reason = "";

  // inner loop Gauss-Newton iterations with degeneracy-aware updates
  for (int gn_iter = 0; gn_iter < config_-> max_gn_iter && !converged; ++gn_iter) {
    // --- Build jacobian and residual for current transformation
    // the dimension will be computed in "computeJacobianResidualInformation" function
    Eigen::MatrixXd A;
    Eigen::VectorXd b, W_inv;

    // compose with initial transformation: final_T = current_T * initial_T
    const Eigen::Matrix4d T_combined = current_transformation.matrix() * initial_T_var.matrix();

    // compute Jacobian, residuals, and information matrix 
    computeJacobianResidualInformation(sample_inds, high_curv_match,
                                       query_mat, map_mat, map_normals_mat,
                                       T_combined, A, b, W_inv, config_);
    // Compute original Hessian
    Eigen::MatrixXd H_original = A.transpose() * W_inv.asDiagonal() * A;
    // Apply Schur complement marginalization
    auto [H_marg_theta, H_marg_t] = schurComplementMarginalization(H_original);
    // Compute scaling factor
    // ell_mr = computeScalingFactorTrace(H_marg_theta, H_marg_t);
    double ell_mr = computeScalingFactorMax(H_marg_theta, H_marg_t);
    // Per-GN-iter; uncomment for QP scaling debug.
    CLOG(DEBUG, "lidar.localization_daicp") << "----------------------use H_theta/H_t, ell_mr:   " << ell_mr;

    // Compute current weighted cost (0.5 * b^T * W_inv * b) and weighted gradient norm
    curr_cost = 0.5 * b.transpose() * W_inv.asDiagonal() * b;
    const Eigen::VectorXd weighted_gradient = A.transpose() * W_inv.asDiagonal() * b;
    const double grad_norm = weighted_gradient.norm();
    
    // STEAM-style convergence checking
    // 1. Check absolute cost threshold
    if (curr_cost <= config_->abs_cost_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST";
    }
    // 2. Check absolute cost change (after first iteration)
    else if (gn_iter > 0 && std::abs(prev_cost - curr_cost) <= config_->abs_cost_change_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST_CHANGE";
    }
    // 3. Check relative cost change (after first iteration)
    else if (gn_iter > 0 && prev_cost > 0 && 
             std::abs(prev_cost - curr_cost) / prev_cost <= config_->rel_cost_change_thresh) {
      converged = true;
      termination_reason = "CONVERGED_RELATIVE_COST_CHANGE";
    }
    // 4. Check zero gradient
    else if (grad_norm < config_->zero_gradient_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ZERO_GRADIENT";
    }

    // DEGENERACY-AWARE EIGENSPACE PROJECTION
    // --- compute original Hessian
    // Eigen::Matrix<double, 6, 6>  H_original = A.transpose() * W_inv * A;
    // Construct the inverse of the block scaling matrix: D_inv
    // NOTE: Parameter ordering is [translation, orientation]
    // D = [1, 1, 1, ell_mr, ell_mr, ell_mr]
    // D_inv = [1, 1, 1, 1/ell_mr, 1/ell_mr, 1/ell_mr]
    Eigen::Matrix<double, 6, 6> D_inv = Eigen::Matrix<double, 6, 6>::Identity();
    D_inv.block<3, 3>(3, 3) *= (1.0 / ell_mr);  // rotation scaling inverse (last 3x3 block)
    // translation scaling remains 1.0
    // Scale the jacobian
    Eigen::MatrixXd A_scaled = A * D_inv;
    // Degeneracy analysis in eigenspace
    Eigen::Matrix<double, 6, 6> H_scaled = A_scaled.transpose() * W_inv.asDiagonal() * A_scaled;

    Eigen::VectorXd eigenvalues;
    Eigen::Matrix<double, 6, 6> eigenvectors;
    bool eigen_success = computeEigenvalueDecomposition(H_scaled, eigenvalues, eigenvectors);

    if (!eigen_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton eigenvalue decomposition failed";
      return false;
    }
    
    // Compute unified threshold
    const double eigenvalue_threshold = computeThreshold(eigenvalues, config_->degeneracy_thresh);
    
    // Construct well-conditioned directions matrix 
    Eigen::Matrix<double, 6, 6> V, Vf;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Vd;
    Eigen::VectorXd eigen_vf;
    constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, 
                                       V, Vf, eigen_vf, Vd);

    // =================== solve the optimization problem ===================
    // Per-iteration QP logs are *very* spammy (10+ lines per GN inner step,
    // ~200/sec). Keep them off by default; flip to true only when actively
    // debugging the QP path.
    bool verbose = true;
    Eigen::VectorXd delta_params_scaled;
    if (Vd.cols() > 0) {
      // [DIAG] One-shot per-GN-iter notice that we entered the QP path.
      // Kept at INFO so it shows up without flipping `verbose`.
      CLOG(INFO, "lidar.localization_daicp")
          << "[DIAG] Degeneracy detected (" << Vd.cols()
          << " direction(s)); using constrained QP solver ("
          << config_->qp_solver_name << ").";

      // Per-iter cap on |v_i^T x| in the degenerate subspace. x is the GN
      // perturbation in *scaled* coordinates (rotation entries multiplied by
      // ell_mr inside this function), and it is computed about the current
      // iterate (which already incorporates the motion prior), so the bound
      // remains centred at 0. Sourced from config so it can be tuned per
      // environment without recompiling.
      Eigen::VectorXd epsilon_dx(6);
      epsilon_dx << config_->qp_eps_trans, config_->qp_eps_trans, config_->qp_eps_trans,
                    config_->qp_eps_rot,   config_->qp_eps_rot,   config_->qp_eps_rot;
      // Bring the rotation slack into the same scaled space as A_scaled / x.
      epsilon_dx.tail<3>() *= ell_mr;
      if (verbose) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Solving constrained QP with " << Vd.cols() << " degenerate directions";
      }
      
      // Compute QP matrices from least-squares problem
      // minimize: ||A*x - b||^2_W = x^T * (A^T W A) * x - 2 * (A^T W b)^T * x + b^T W b
      // This gives: H = 2*A^T*W*A, g = -2*A^T*W*b
      Eigen::MatrixXd F = 2.0 * A_scaled.transpose() * W_inv.asDiagonal() * A_scaled;
      Eigen::VectorXd f = -2.0 * A_scaled.transpose() * W_inv.asDiagonal() * b;
      
      // Solve the QP problem. Solver chosen via config_->qp_solver_name:
      //   "qrqp"  : QR-based active-set solver (pure C++, no extra deps, very robust
      //             on tiny dense problems). RECOMMENDED for this 6-D problem.
      //   "osqp"  : ADMM-based first-order solver. Requires libcasadi_conic_osqp
      //             built against a matching libosqp ABI; otherwise SIGSEGV at
      //             solver construction. Also requires upper-triangular H sparsity
      //             (handled in solveConstrainedQPConic).
      daicp_qp::QPSolverResult result;
      // Flush any buffered DEBUG logs so that, if qrqp aborts hard inside
      // CasADi (which has historically called std::abort on certain
      // rank-deficient KKT systems), we still see the lead-up in the log file.
      el::Loggers::flushAll();
      try {
        result = daicp_qp::solveConstrainedQPConic(
            F, f, Vd, epsilon_dx, config_->qp_solver_name, verbose);
      } catch (const std::exception& e) {
        // Defense in depth: solveConstrainedQPConic already catches inside,
        // but in case anything escapes we still need a sane state.
        CLOG(ERROR, "lidar.localization_daicp")
            << "QP solver threw at call site: " << e.what()
            << " - falling back to unconstrained solution";
        result.success = false;
        result.solver_status = std::string("call_site_error: ") + e.what();
      } catch (...) {
        CLOG(ERROR, "lidar.localization_daicp")
            << "QP solver threw unknown exception at call site"
            << " - falling back to unconstrained solution";
        result.success = false;
        result.solver_status = "call_site_error: unknown";
      }
      el::Loggers::flushAll();

      // Even on "success", reject NaN/Inf solutions so we don't propagate
      // garbage into the GN update.
      if (result.success && !result.x_optimal.allFinite()) {
        CLOG(WARNING, "lidar.localization_daicp")
            << "QP returned non-finite x_optimal at call site, rejecting";
        result.success = false;
        result.solver_status = "non_finite_at_callsite";
      }

      if (result.success) {
        delta_params_scaled = result.x_optimal;
        if (verbose) {
          CLOG(DEBUG, "lidar.localization_daicp") << "QP solved successfully in " 
                << result.solve_time << "s (" << result.iterations << " iterations)";
        }
      } else {
          // Last resort: revert back to solution remapping without constraints
          CLOG(WARNING, "lidar.localization_daicp")
              << "QP solver failed (" << result.solver_status
              << "), reverting back to solution remapping";
          delta_params_scaled = computeUpdateStep(A_scaled, b, W_inv, V, Vf);

          // Final guard: if even the unconstrained remapping is non-finite,
          // emit a zero step so that this GN iteration is a no-op rather
          // than corrupting the running estimate. The outer loop's
          // convergence/divergence checks will then terminate normally.
          if (!delta_params_scaled.allFinite()) {
            CLOG(ERROR, "lidar.localization_daicp")
                << "Fallback remapping also produced non-finite step; "
                << "using zero step for this GN iteration";
            delta_params_scaled = Eigen::VectorXd::Zero(F.rows());
          }
        }
    } else {
      // No degenerate directions, solve unconstrained
      if (verbose) {
        CLOG(DEBUG, "lidar.localization_daicp") << "No degeneracy detected, using unconstrained update";
      }
      delta_params_scaled = computeUpdateStep(A_scaled, b, W_inv, V, Vf);
    }
    // ======================================================================

    // Compute the scaled covariance matrix.
    // Vd / Vf live in the scaled coordinate frame (rotation entries multiplied
    // by ell_mr), so the prior must be expressed in the same frame before it
    // can be projected onto a degenerate direction:
    //   x_scaled = D * x        =>   Sigma_scaled = D * Sigma * D^T
    // where D = diag(1,1,1, ell_mr, ell_mr, ell_mr).
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
    D.block<3, 3>(3, 3) *= ell_mr;
    const Eigen::Matrix<double, 6, 6> prior_cov_scaled = D * prior_cov * D.transpose();
    Eigen::Matrix<double, 6, 6> daicp_cov_scaled = config_->use_prior_prop_cov
        ? computeDaicpCovariance(Vf, eigen_vf, Vd, prior_cov_scaled, config_->degenerate_cov_alpha)
        : computeDaicpCovarianceDefault(Vf, eigen_vf, Vd);
    
    // Unscale the parameters and covariance
    Eigen::VectorXd delta_params = D_inv * delta_params_scaled;
    daicp_cov = D_inv * daicp_cov_scaled * D_inv.transpose();
    // --- Debug-print covariance information
    // printCovarianceInfo(daicp_cov);

    // === Sanity-check the GN step before applying it ============================
    // Numerical pathologies in W_inv / QP / scaling can occasionally produce a
    // non-finite or absurdly large delta. If that propagates into vec2tran the
    // resulting SE(3) matrix has entries ~1e100+, and the next call to tran2vec
    // throws std::runtime_error from lgmath ("so3 logarithmic map failed..."),
    // which uncaught will SIGABRT the whole vtr_navigation process. Catch it here
    // and exit the GN loop with the last good iterate instead.
    constexpr double kMaxStepNorm = 10.0;  // 10 m / 10 rad is already well beyond
                                           // any reasonable per-iter ICP step
    if (!delta_params.allFinite()) {
      CLOG(WARNING, "lidar.localization_daicp")
          << "GN step is non-finite (iter " << gn_iter
          << "), aborting GN loop with last good iterate";
      termination_reason = "ABORT_NON_FINITE_STEP";
      break;
    }
    if (delta_params.norm() > kMaxStepNorm) {
      CLOG(WARNING, "lidar.localization_daicp")
          << "GN step too large (iter " << gn_iter
          << ", norm=" << delta_params.norm()
          << "), aborting GN loop with last good iterate";
      termination_reason = "ABORT_LARGE_STEP";
      break;
    }
    // ============================================================================

    // Accumulate parameters
    accumulated_params += delta_params;
    // Convert accumulated parameters to transformation
    // [NOTE]: Lgmath uses [tx, ty, tz, rx, ry, rz] ordering.
    current_transformation = lgmath::se3::Transformation(lgmath::se3::vec2tran(accumulated_params));

    // Check parameter change convergence AFTER applying the update
    double param_change = delta_params.norm();
    
    // Check convergence (but allow at least one iteration to see progress)
    if (gn_iter > 0 && param_change < config_->inner_tolerance) {
      converged = true;
      termination_reason = "CONVERGED_PARAMETER_CHANGE";
      break;
    }
    // check convergence
    if (converged) {
      CLOG(DEBUG, "lidar.localization_daicp") << "Converged after " << (gn_iter) << " iterations: " << termination_reason;
      break;
    }
    // Update cost for next iteration
    prev_cost = curr_cost;

  } // --- end of GN inner loop

  // Check if terminated due to max iterations
  if (!converged) {
    // Maximum Gauss-Newton iterations reached
    termination_reason = "MAX_ITERATIONS";
  }
  // Update T_var with the final transformation
  // Compose the delta transformation with the initial transformation: T_final = delta_T * T_initial
  const lgmath::se3::Transformation final_transformation = lgmath::se3::Transformation(
      static_cast<Eigen::Matrix4d>(current_transformation.matrix() * initial_T_var.matrix())
  );
  // Calculate the actual delta from initial to final for STEAM update
  Eigen::Matrix4d delta_for_steam = final_transformation.matrix() * initial_T_var.matrix().inverse();

  // === Final safety check before lgmath::tran2vec ============================
  // tran2vec throws std::runtime_error when the rotation block is not a valid
  // SO(3) (e.g. has NaN/Inf or non-orthonormal entries from a numerical
  // blowup earlier in the GN loop). An uncaught throw here aborts the whole
  // vtr_navigation process, so guard it explicitly and return false instead -
  // the caller (localization_daicp_module) breaks out of the outer ICP loop
  // and keeps the last good T_m_s_var on warning.
  if (!delta_for_steam.allFinite()) {
    CLOG(ERROR, "lidar.localization_daicp")
        << "Final delta transformation is non-finite; skipping STEAM update."
        << " termination_reason=" << termination_reason;
    return false;
  }
  Eigen::Matrix<double, 6, 1> delta_vec_steam;
  try {
    delta_vec_steam = lgmath::se3::tran2vec(delta_for_steam);
  } catch (const std::exception& e) {
    CLOG(ERROR, "lidar.localization_daicp")
        << "lgmath::tran2vec failed (" << e.what()
        << "); skipping STEAM update. termination_reason=" << termination_reason;
    return false;
  }
  if (!delta_vec_steam.allFinite()) {
    CLOG(ERROR, "lidar.localization_daicp")
        << "tran2vec produced non-finite delta; skipping STEAM update.";
    return false;
  }
  // ===========================================================================

  // Apply the update to T_var using STEAM's update mechanism
  T_var->update(delta_vec_steam);

  // ============ [DIAG] One-line summary of what DA-ICP just did ============
  // accumulated_params is the GN delta in [trans, rot] order, in the local
  // tangent space about initial_T_var. This shows how much the lidar moved
  // T_var in this single GN call (one outer ICP step).
  CLOG(INFO, "lidar.localization_daicp")
      << "[DIAG] daGaussNewton: term=" << termination_reason
      << ", final_cost=" << curr_cost
      << ", delta_trans=" << accumulated_params.head<3>().norm() << " m"
      << ", delta_rot=" << accumulated_params.tail<3>().norm() << " rad";
  // =========================================================================

  return true;
}

}  // namespace hybridqp
}  // namespace da_lib
}  // namespace lidar
}  // namespace vtr
