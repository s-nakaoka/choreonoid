/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "PreviewController.h"

using namespace hrp;
using namespace rats;

template <std::size_t dim>
void preview_control_base<dim>::update_x_k(const hrp::Vector3& pr)
{
  zmp_z = pr(2);
  Eigen::Matrix<double, 2, 1> tmpv;
  tmpv(0,0) = pr(0);
  tmpv(1,0) = pr(1);
  p.push_back(tmpv);
  if ( p.size() > 1 + delay ) p.pop_front();
  if ( is_doing() ) calc_x_k();
}

template <std::size_t dim>
void preview_control_base<dim>::update_zc(double zc)
{
  riccati.c(0, 2) = - zc / g; 
  riccati.solve();
}

void preview_control::calc_f()
{
  f.resize(delay+1);
  Eigen::Matrix<double, 1, 1> fa;
  hrp::Matrix33 gsi(hrp::Matrix33::Identity());
  for (size_t i = 0; i < delay; i++) {
    fa = riccati.R_btPb_inv * riccati.b.transpose() * (gsi * riccati.Q * riccati.c.transpose());
    gsi = riccati.A_minus_bKt * gsi;
    f(i+1) = fa(0,0);
  }
}

void preview_control::calc_u()
{
  Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());
  for (size_t i = 0; i < 1 + delay; i++)
    gfp += f(i) * p[i];
  u_k = -riccati.K * x_k + gfp;
};

void preview_control::calc_x_k()
{
  calc_u();
  x_k = riccati.A * x_k + riccati.b * u_k;  
}

void extended_preview_control::calc_f()
{
  f.resize(delay + 1);
  Eigen::Matrix<double, 1, 1> fa;
  Eigen::Matrix<double, 4, 4> gsi(Eigen::Matrix<double, 4, 4>::Identity());
  Eigen::Matrix<double, 4, 1> qt(riccati.Q * riccati.c.transpose());
  for (size_t i = 0; i < delay; i++) {
    if ( i == delay - 1 ) qt = riccati.P * qt;
    fa = riccati.R_btPb_inv * riccati.b.transpose() * (gsi * qt);
    gsi = riccati.A_minus_bKt * gsi;
    f(i+1) = fa(0,0);
  }
}

void extended_preview_control::calc_u()
{
  Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());
  for (size_t i = 0; i < 1 + delay; i++)
    gfp += f(i) * p[i];
  u_k = -riccati.K * x_k_e + gfp;
};

void extended_preview_control::calc_x_k()
{
  calc_u();
  x_k_e = riccati.A * x_k_e + riccati.b * u_k;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 2; j++)
      x_k(i,j) += x_k_e(i+1,j);
}

