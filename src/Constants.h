#pragma once

#include <string>

namespace Mpc
{
  // Timestep length
  const size_t N    = 15;
  // Duration
  const double dt   = 0.05;
  // Vehicle's center of gravity 
  const double Lf   = 2.67;

  const size_t x_start      = 0;
  const size_t y_start      = x_start + N;
  const size_t psi_start    = y_start + N;
  const size_t v_start      = psi_start + N;
  const size_t cte_start    = v_start + N;
  const size_t epsi_start   = cte_start + N;
  const size_t delta_start  = epsi_start + N;
  const size_t a_start      = delta_start + N - 1;

  // options for IPOPT solver
  const std::string ipopt_options = 
    "Integer print_level  0\nSparse  true        forward\nSparse  true        reverse\nNumeric max_cpu_time          0.5\n";
}