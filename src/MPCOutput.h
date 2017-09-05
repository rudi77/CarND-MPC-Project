#pragma once

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

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

  static MpcOutput Instance(const CppAD::ipopt::solve_result<Dvector>& solution, size_t n, std::vector<int> indices )
  {
    MpcOutput output;
    
    for (size_t i = 0; i<n; i++)
    {
      output.ptsx.push_back(solution.x[indices[0] + i]);
      output.ptsy.push_back(solution.x[indices[1] + i]);
      output.psi.push_back(solution.x[indices[2] + i]);
      output.v.push_back(solution.x[indices[3] + i]);
      output.cte.push_back(solution.x[indices[4] + i]);
      output.epsi.push_back(solution.x[indices[5] + i]);
      output.delta.push_back(solution.x[indices[6] + i]);
      output.a.push_back(solution.x[indices[7] + i]);
    }

    return output;
  }
};
