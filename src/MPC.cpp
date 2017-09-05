#include "MPC.h"
#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Timestep length and duration
const size_t N = 15;
const auto dt = 0.05;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval 
{
private:
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

  // Fitted polynomial coefficients
  Eigen::VectorXd _coeffs;

  vector<int> _weights;

  double ref_v;
public:


  FG_eval(Eigen::VectorXd coeffs, const vector<int>& weights, double ref_v)
    : _coeffs(coeffs), _weights(weights), ref_v(ref_v)
  {
    assert(_weights.size() == 7);
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    const auto w1 = _weights[0];
    const auto w2 = _weights[1];
    const auto w3 = _weights[2];
    const auto w4 = _weights[3];
    const auto w5 = _weights[4];
    const auto w6 = _weights[5];
    const auto w7 = _weights[6];


    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += w1 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w2 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w3 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += w4 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w5 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += w6 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w7 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    //
    // Add +1 to every position because the cost is a position 0
    fg[1+x_start]     = vars[x_start];
    fg[1+y_start]     = vars[y_start];
    fg[1+psi_start]   = vars[psi_start];
    fg[1+v_start]     = vars[v_start];
    fg[1+cte_start]   = vars[cte_start];
    fg[1+epsi_start]  = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1     = vars[x_start + t];
      AD<double> y1     = vars[y_start + t];
      AD<double> psi1   = vars[psi_start + t];
      AD<double> v1     = vars[v_start + t];
      AD<double> cte1   = vars[cte_start + t];
      AD<double> epsi1  = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0     = vars[x_start + t-1];
      AD<double> y0     = vars[y_start + t-1];
      AD<double> psi0   = vars[psi_start + t-1];
      AD<double> v0     = vars[v_start + t-1];
      AD<double> cte0   = vars[cte_start + t-1];
      AD<double> epsi0  = vars[epsi_start + t-1];

      // Only consider the actuation at time t.
      AD<double> delta0   = vars[delta_start + t-1];
      AD<double> a0       = vars[a_start + t-1];
      AD<double> f0       = _coeffs[0] + _coeffs[1]*x0 + _coeffs[2]*x0*x0 + _coeffs[3]*x0*x0*x0;
      AD<double> psides0  = CppAD::atan(_coeffs[1] + 2*_coeffs[2]*x0 + 3*_coeffs[3]*x0*x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1+x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1+y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1+psi_start + t]   = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1+v_start + t]     = v1 - (v0 + a0 * dt);
      fg[1+cte_start + t]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1+epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};


MpcOutput MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  auto ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  auto x    = state[0];
  auto y    = state[1];
  auto psi  = state[2];
  auto v    = state[3];
  auto cte  = state[4];
  auto epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) 
  {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start]     = x;
  vars[y_start]     = y;
  vars[psi_start]   = psi;
  vars[v_start]     = v;
  vars[cte_start]   = cte;
  vars[epsi_start]  = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; i++) 
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]     = x;
  constraints_lowerbound[y_start]     = y;
  constraints_lowerbound[psi_start]   = psi;
  constraints_lowerbound[v_start]     = v;
  constraints_lowerbound[cte_start]   = cte;
  constraints_lowerbound[epsi_start]  = epsi;

  constraints_upperbound[x_start]     = x;
  constraints_upperbound[y_start]     = y;
  constraints_upperbound[psi_start]   = psi;
  constraints_upperbound[v_start]     = v;
  constraints_upperbound[cte_start]   = cte;
  constraints_upperbound[epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, weights, ref_v);

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
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  MpcOutput output;

  for (size_t i=0; i<N; i++)
  {
    output.ptsx.push_back(solution.x[x_start+ i]);
    output.ptsy.push_back(solution.x[y_start + i]);
    output.psi.push_back(solution.x[psi_start + i]);
    output.v.push_back(solution.x[v_start + i]);
    output.cte.push_back(solution.x[cte_start + i]);
    output.epsi.push_back(solution.x[epsi_start + i]);
  }

  for (size_t i=0; i<N-1; i++)
  {
    output.delta.push_back(solution.x[delta_start + i]);
    output.a.push_back(solution.x[a_start + i]);
  }

  return output;
}
