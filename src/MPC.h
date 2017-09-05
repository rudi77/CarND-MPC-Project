#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include "MPCOutput.h"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MpcOutput Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
