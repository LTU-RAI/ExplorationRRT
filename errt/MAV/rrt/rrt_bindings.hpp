/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Number of decision variables
static const uintptr_t RRT_NUM_DECISION_VARIABLES = 150;

/// Number of parameters
static const uintptr_t RRT_NUM_PARAMETERS = 159;

/// Number of parameters associated with augmented Lagrangian
static const uintptr_t RRT_N1 = 0;

/// Number of penalty constraints
static const uintptr_t RRT_N2 = 0;

/// rrt version of ExitStatus
/// Structure: `rrtExitStatus`
enum class rrtExitStatus {
  /// The algorithm has converged
  ///
  /// All termination criteria are satisfied and the algorithm
  /// converged within the available time and number of iterations
  rrtConverged,
  /// Failed to converge because the maximum number of iterations was reached
  rrtNotConvergedIterations,
  /// Failed to converge because the maximum execution time was reached
  rrtNotConvergedOutOfTime,
  /// If the gradient or cost function cannot be evaluated internally
  rrtNotConvergedCost,
  /// Computation failed and NaN/Infinite value was obtained
  rrtNotConvergedNotFiniteComputation,
};

/// Solver cache (structure `rrtCache`)
///
struct rrtCache;

/// rrt version of AlmOptimizerStatus
/// Structure: `rrtSolverStatus`
///
struct rrtSolverStatus {
  /// Exit status
  rrtExitStatus exit_status;
  /// Number of outer iterations
  unsigned long num_outer_iterations;
  /// Total number of inner iterations
  ///
  /// This is the sum of the numbers of iterations of
  /// inner solvers
  unsigned long num_inner_iterations;
  /// Norm of the fixed-point residual of the the problem
  double last_problem_norm_fpr;
  /// Total solve time
  unsigned long long solve_time_ns;
  /// Penalty value
  double penalty;
  /// Norm of delta y divided by the penalty parameter
  double delta_y_norm_over_c;
  /// Norm of F2(u)
  double f2_norm;
  /// Value of cost function at solution
  double cost;
  /// Lagrange multipliers
  const double *lagrange;
};

extern "C" {

/// Allocate memory and setup the solver
rrtCache *rrt_new();

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// # Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `rrt_new` (and should be destroyed once it is not
///   needed using `rrt_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `RRT_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `RRT_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `RRT_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// # Returns:
/// Instance of `rrtSolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
/// .
/// .
/// .
/// # Safety
/// All arguments must have been properly initialised
rrtSolverStatus rrt_solve(rrtCache *instance,
                          double *u,
                          const double *params,
                          const double *y0,
                          const double *c0);

/// Deallocate the solver's memory, which has been previously allocated
/// using `rrt_new`
///
///
/// # Safety
/// All arguments must have been properly initialised
void rrt_free(rrtCache *instance);

} // extern "C"
