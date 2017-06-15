/*                                                                         80->|
 * MPC.cpp
 *
 * Modifications: James William Dunn
 *          Date: June 15, 2017
 */

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;  // 7 10 12 20
double dt = 0.11; //  0.06835 0.1367 0.1 0.05

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. The following defines
// when one variable starts and another ends.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

double ref_cte = 0.0;
double ref_epsi = 0.0;
double ref_v = 91.0;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implementation of MPC
    // `fg` a vector of the cost constraints, 
    // `vars` is a vector of variable values (state & actuators)

    fg[0] = 0.0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) { // 4000,1,1
      fg[0] += 2500*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 2500*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) { // 100000, 1
      fg[0] += 6*CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 5*CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) { // 1000,1
      fg[0] += 2500*CppAD::pow(vars[delta_start + i + 1] 
               - vars[delta_start + i], 2);
      fg[0] += 10*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    
    // Setup Constraints (straight from the MPC quiz code)

    // Add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // Updated from quiz
      AD<double> x2 = x0 * x0;
      
      // 3rd order polynomial
      AD<double> x3 = x2 * x0;
      AD<double> f0 = coeffs[3] * x3 + coeffs[2] * x2 
                      + coeffs[1] * x0 + coeffs[0];
      AD<double> psides0 = CppAD::atan(x0 * (2 * coeffs[2] 
                      + x0 * 3 * coeffs[3]) + coeffs[1]);

      // 2nd order polynomial
      //AD<double> f0 = coeffs[2] * x2 + coeffs[1] * x0 + coeffs[0];
      //AD<double> psides0 = CppAD::atan(x0 * (2 * coeffs[2]) + coeffs[1]);


      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, 
                          Eigen::VectorXd coeffs,
                          vector<double> &mpc_x_vals,
                          vector<double> &mpc_y_vals) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // The number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start]    = state[0];
  vars[y_start]    = state[1];
  vars[psi_start]  = state[2];
  vars[v_start]    = state[3];
  vars[cte_start]  = state[4];
  vars[epsi_start] = state[5];
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -10 and 10
  // degrees (values in fractions of 25 where 1.0=25 and 0.4=10).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.4;
    vars_upperbound[i] = 0.4;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.02;
    vars_upperbound[i] = 0.91;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]    = state[0];
  constraints_lowerbound[y_start]    = state[1];
  constraints_lowerbound[psi_start]  = state[2];
  constraints_lowerbound[v_start]    = state[3];
  constraints_lowerbound[cte_start]  = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start]    = state[0];
  constraints_upperbound[y_start]    = state[1];
  constraints_upperbound[psi_start]  = state[2];
  constraints_upperbound[v_start]    = state[3];
  constraints_upperbound[cte_start]  = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  std::string options;
  // Uncomment this for more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of higher performance routines.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Set a maximum time limit of 250 milliseconds.
  options += "Numeric max_cpu_time          0.25\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check for error condition
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok) cout << "SOLVER FAILED XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" 
      << "XXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
  //	exit(EXIT_FAILURE);

  // Load the solution path into the visualization variables
  mpc_x_vals.resize(N-1);
  mpc_y_vals.resize(N-1);

  for (int i = 0; i < N-1; i++) {
    mpc_x_vals[i] = solution.x[x_start + i + 1];
    mpc_y_vals[i] = solution.x[y_start + i + 1];
  }

  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {-solution.x[delta_start], solution.x[a_start]};
}
