#pragma once

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "Constants.h"

typedef CPPAD_TESTVECTOR(double) Dvector;

struct MpcOutput
{
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  std::vector<double> psi;
  std::vector<double> v;
  std::vector<double> cte;
  std::vector<double> epsi;
  std::vector<double> delta;
  std::vector<double> a;

  static MpcOutput Instance(const CppAD::ipopt::solve_result<Dvector>& solution )
  {
    MpcOutput output;

    for (size_t i = 0; i<Mpc::N; i++)
    {
      output.ptsx.push_back(solution.x[Mpc::x_start + i]);
      output.ptsy.push_back(solution.x[Mpc::y_start + i]);
      output.psi.push_back(solution.x[Mpc::psi_start + i]);
      output.v.push_back(solution.x[Mpc::v_start + i]);
      output.cte.push_back(solution.x[Mpc::cte_start + i]);
      output.epsi.push_back(solution.x[Mpc::epsi_start + i]);
    }

    for (size_t i = 0; i<Mpc::N - 1; i++)
    {
      output.delta.push_back(solution.x[Mpc::delta_start + i]);
      output.a.push_back(solution.x[Mpc::a_start + i]);
    }

    return output;
  }
};
