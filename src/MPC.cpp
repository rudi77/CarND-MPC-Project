#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

#include "MPC.h"
#include "Constants.h"

using CppAD::AD;

class FG_eval 
{
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
    for (size_t t = 0; t < Mpc::N; t++) {
      fg[0] += w1 * CppAD::pow(vars[Mpc::cte_start + t], 2);
      fg[0] += w2 * CppAD::pow(vars[Mpc::epsi_start + t], 2);
      fg[0] += w3 * CppAD::pow(vars[Mpc::v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < Mpc::N - 1; t++) {
      fg[0] += w4 * CppAD::pow(vars[Mpc::delta_start + t], 2);
      fg[0] += w5 * CppAD::pow(vars[Mpc::a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < Mpc::N - 2; t++) {
      fg[0] += w6 * CppAD::pow(vars[Mpc::delta_start + t + 1] - vars[Mpc::delta_start + t], 2);
      fg[0] += w7 * CppAD::pow(vars[Mpc::a_start + t + 1] - vars[Mpc::a_start + t], 2);
    }

    // Initial constraints
    //
    // Add +1 to every position because the cost is a position 0
    fg[1+Mpc::x_start]     = vars[Mpc::x_start];
    fg[1+Mpc::y_start]     = vars[Mpc::y_start];
    fg[1+Mpc::psi_start]   = vars[Mpc::psi_start];
    fg[1+Mpc::v_start]     = vars[Mpc::v_start];
    fg[1+Mpc::cte_start]   = vars[Mpc::cte_start];
    fg[1+Mpc::epsi_start]  = vars[Mpc::epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < Mpc::N; t++) {
      // The state at time t+1 .
      AD<double> x1     = vars[Mpc::x_start + t];
      AD<double> y1     = vars[Mpc::y_start + t];
      AD<double> psi1   = vars[Mpc::psi_start + t];
      AD<double> v1     = vars[Mpc::v_start + t];
      AD<double> cte1   = vars[Mpc::cte_start + t];
      AD<double> epsi1  = vars[Mpc::epsi_start + t];

      // The state at time t.
      AD<double> x0     = vars[Mpc::x_start + t-1];
      AD<double> y0     = vars[Mpc::y_start + t-1];
      AD<double> psi0   = vars[Mpc::psi_start + t-1];
      AD<double> v0     = vars[Mpc::v_start + t-1];
      AD<double> cte0   = vars[Mpc::cte_start + t-1];
      AD<double> epsi0  = vars[Mpc::epsi_start + t-1];

      // Only consider the actuation at time t.
      AD<double> delta0   = vars[Mpc::delta_start + t-1];
      AD<double> a0       = vars[Mpc::a_start + t-1];
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
      fg[1+ Mpc::x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * Mpc::dt);
      fg[1+ Mpc::y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * Mpc::dt);
      fg[1+ Mpc::psi_start + t]   = psi1 - (psi0 + v0 * delta0 / Mpc::Lf * Mpc::dt);
      fg[1+ Mpc::v_start + t]     = v1 - (v0 + a0 * Mpc::dt);
      fg[1+ Mpc::cte_start + t]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * Mpc::dt));
      fg[1+ Mpc::epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Mpc::Lf * Mpc::dt);
    }
  }
};


MpcOutput MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  auto ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const auto x    = state[0];
  const auto y    = state[1];
  const auto psi  = state[2];
  const auto v    = state[3];
  const auto cte  = state[4];
  const auto epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = Mpc::N * 6 + (Mpc::N - 1) * 2;
  // Number of constraints
  size_t n_constraints = Mpc::N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) 
  {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[Mpc::x_start]     = x;
  vars[Mpc::y_start]     = y;
  vars[Mpc::psi_start]   = psi;
  vars[Mpc::v_start]     = v;
  vars[Mpc::cte_start]   = cte;
  vars[Mpc::epsi_start]  = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < Mpc::delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = Mpc::delta_start; i < Mpc::a_start; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = Mpc::a_start; i < n_vars; i++) {
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

  constraints_lowerbound[Mpc::x_start]     = x;
  constraints_lowerbound[Mpc::y_start]     = y;
  constraints_lowerbound[Mpc::psi_start]   = psi;
  constraints_lowerbound[Mpc::v_start]     = v;
  constraints_lowerbound[Mpc::cte_start]   = cte;
  constraints_lowerbound[Mpc::epsi_start]  = epsi;

  constraints_upperbound[Mpc::x_start]     = x;
  constraints_upperbound[Mpc::y_start]     = y;
  constraints_upperbound[Mpc::psi_start]   = psi;
  constraints_upperbound[Mpc::v_start]     = v;
  constraints_upperbound[Mpc::cte_start]   = cte;
  constraints_upperbound[Mpc::epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, weights, ref_v);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solve the problem. For further information see:
  // https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
  CppAD::ipopt::solve<Dvector, FG_eval>
  (
    Mpc::ipopt_options, 
    vars,                     // Specifies the initial point where Ipopt starts the optimization process. 
    vars_lowerbound,          // Specifies the lower limits for the argument in the optimization problem.
    vars_upperbound,          // Specifies the upper limits for the argument in the optimization problem.
    constraints_lowerbound,   // Specifies the lower limits for the constraints in the optimization problem.
    constraints_upperbound,   // Specifies the upper limits for the constraints in the optimization problem.
    fg_eval,                  // Contains the COST function
    solution                  // The final solution
  );

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  return MpcOutput::Instance(solution);
}
