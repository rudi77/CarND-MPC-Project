#pragma once

#include "Eigen-3.3/Eigen/Core"
#include "Constants.h"

// Keeps the vehicle's state
struct VehicleState
{
  double px;
  double py;
  double v;
  double psi;
  double cte;
  double epsi;
  double delta;
  double acceleration;

  // Returns the vehicle's state taking a certain delay into account.
  Eigen::VectorXd FutureState(double latency = 0.1)
  {
    Eigen::VectorXd state(6);

    px = 0 + v*latency;
    py = 0;
    psi = 0 -v*delta / Mpc::Lf*latency;
    v = v + acceleration*latency;
    cte = cte + v * sin(epsi) * latency;
    epsi = epsi + v*(-delta) / Mpc::Lf * latency;

    state << px, py, psi, v, cte, epsi;

    return state;
  }
};
