#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPCOutput.h"

using namespace std;

class MPC {
private:
  vector<int> weights; 
  double ref_v;

public:
  MPC(const vector<int>& weights, double ref_v)
    : weights(weights), ref_v(ref_v)
  {}

  virtual ~MPC() {}

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MpcOutput Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
