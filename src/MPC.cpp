#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// The timestep length and duration
size_t N = 10;
double dt = 0.05;

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

// reference velocity
double ref_v = 100;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  explicit FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // Update the state cost with a weighted sum of squared state values.
    // These weights values were chosen from a mixture of trial and error, in regards to simulator performance,
    // and suggestions from Udacity SDC lessons.

    // Reference State Cost
    for (size_t t = 0; t < N; t++) {
      fg[0] += 0.25 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 175.0 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 1.0 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize change-rate.
    // If we're making a turn, we'd like the turn to be smooth, not sharp.
    // Additionally, the vehicle velocity should not change too radically.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 0.1 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 1.0 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    // The goal of this final loop is to make control decisions more consistent,
    // or smoother. The next control input should be similar to the current one.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 4000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      // AD<double> psides0 = CppAD::atan(coeffs[1]);
      // adapt the code used for 1st order polynomials to 3rd order:
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.

      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // According to the tips & tricks section, use psi0 - v0 below (not a +)
      // In the simulator, a positive value implies a right turn and a negative value implies a left turn
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // The number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);
  // The number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // The lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }

  constraints_lowerbound[x_start] = constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          50.0\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

//  std::cout << "vars: " << vars << std::endl;
//  std::cout << "vars_lowerbound: " << vars_lowerbound << std::endl;
//  std::cout << "vars_upperbound: " << vars_upperbound << std::endl;
//  std::cout << "constraints_lowerbound: " << constraints_lowerbound << std::endl;
//  std::cout << "constraints_upperbound: " << constraints_upperbound << std::endl;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // The first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  x_vals.clear();
  y_vals.clear();

  x_vals.push_back(state[0]);
  y_vals.push_back(state[1]);
  for (size_t i = 2; i < N; ++i) {
    x_vals.push_back(solution.x[x_start + i]);
    y_vals.push_back(solution.x[y_start + i]);
  }
  return {solution.x[delta_start], solution.x[a_start]};
}
