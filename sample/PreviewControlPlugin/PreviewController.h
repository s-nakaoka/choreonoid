/* -*- coding:utf-8-unix mode:c++ -*- */
#ifndef PREVIEW_H_
#define PREVIEW_H_
#include <iostream>
#include <queue>
#include <deque>
#include <hrpUtil/Eigen3d.h>
#include <cnoid/Vector3Seq>

namespace rats
{
  template <std::size_t dim>
  struct riccati_equation
  {
    Eigen::Matrix<double, dim, dim> A;
    Eigen::Matrix<double, dim, 1> b;
    Eigen::Matrix<double, 1, dim> c;
    Eigen::Matrix<double, dim, dim> P;
    Eigen::Matrix<double, 1, dim> K;
    Eigen::Matrix<double, dim, dim> A_minus_bKt; /* for buffer */
    double Q, R;
    double R_btPb_inv; /* for buffer */

    riccati_equation() : A(Eigen::Matrix<double, dim, dim>::Zero()), b(Eigen::Matrix<double, dim, 1>::Zero()), c(Eigen::Matrix<double, 1, dim>::Zero()),
                         P(Eigen::Matrix<double, dim, dim>::Zero()), K(Eigen::Matrix<double, 1, dim>::Zero()),
                         A_minus_bKt(Eigen::Matrix<double, dim, dim>::Zero()), Q(0), R(0), R_btPb_inv(0) {};
    riccati_equation(const Eigen::Matrix<double, dim, dim>& _A, const Eigen::Matrix<double, dim, 1>& _b,
                     const Eigen::Matrix<double, 1, dim>& _c, const double _Q, const double _R)
      : A(_A), b(_b), c(_c), P(Eigen::Matrix<double, dim, dim>::Zero()), K(Eigen::Matrix<double, 1, dim>::Zero()), A_minus_bKt(Eigen::Matrix<double, dim, dim>::Zero()), Q(_Q), R(_R), R_btPb_inv(0) {};
    virtual ~riccati_equation() {};
    bool solve() {
      Eigen::Matrix<double, dim, dim> prev_P;
      for (int i = 0; i < 2000; i++) {
        R_btPb_inv = (1.0 / (R + (b.transpose() * P * b)(0,0)));
        Eigen::Matrix<double, dim, dim> tmp_pa(P * A);
        K = R_btPb_inv * b.transpose() * tmp_pa;
        prev_P = A.transpose() * tmp_pa + c.transpose() * Q * c - A.transpose() * P * b * K;
        if ((abs((P - prev_P).array()) < 5.0e-5).all()) {
          A_minus_bKt = (A - b * K).transpose();
          return true;
        }
        P = prev_P;
      }
      return false;
    }
  };

  template <std::size_t dim>
  class preview_control_base
  {
  protected:
    static const double g = 9.80665; /* [m/s^2] */
    riccati_equation<dim> riccati;
    Eigen::Matrix<double, 3, 3> tcA;
    Eigen::Matrix<double, 3, 1> tcb;
    Eigen::Matrix<double, 1, 3> tcc;
    Eigen::Matrix<double, 3, 2> x_k;
    Eigen::Matrix<double, 1, 2> u_k;
    hrp::dvector f;
    std::deque<Eigen::Matrix<double, 2, 1> > p;
    double zmp_z, cog_z;
    size_t delay, ending_count;
    virtual void calc_f() = 0;
    virtual void calc_u() = 0;
    virtual void calc_x_k() = 0;
    void init_riccati(const Eigen::Matrix<double, dim, dim>& A,
                      const Eigen::Matrix<double, dim, 1>& b,
                      const Eigen::Matrix<double, 1, dim>& c,
                      const double q = 1.0, const double r = 1.0e-6)
    {
      riccati = riccati_equation<dim>(A, b, c, q, r);
      riccati.solve();
      calc_f();
    };
    /* inhibit copy constructor and copy insertion not by implementing */
    preview_control_base (const preview_control_base& _p);
    preview_control_base &operator=(const preview_control_base &_p);
  public:
    /* dt = [s], zc = [mm], d = [s] */
    preview_control_base(const double dt, const double zc,
                         const hrp::Vector3& init_xk, const double d = 1.6)
      : riccati(), x_k(Eigen::Matrix<double, 3, 2>::Zero()), u_k(Eigen::Matrix<double, 1, 2>::Zero()), p(),
        zmp_z(0), cog_z(zc), delay(static_cast<size_t>(round(d / dt))), ending_count(1+delay)
    {
      tcA << 1, dt, 0.5 * dt * dt,
        0, 1,  dt,
        0, 0,  1;
      tcb << 1 / 6.0 * dt * dt * dt,
        0.5 * dt * dt,
        dt;
      tcc << 1.0, 0.0, -zc / g;
      x_k(0,0) = init_xk(0);
      x_k(0,1) = init_xk(1);
    };
    virtual ~preview_control_base()
    {
      p.clear();
    };
    virtual void update_x_k(const hrp::Vector3& pr);
    virtual void update_x_k()
    {
      hrp::Vector3 pr;
      pr(0) = p.back()(0);
      pr(1) = p.back()(1);
      pr(2) = zmp_z;
      update_x_k(pr);
      ending_count--;
    };
    void update_zc(double zc);
    size_t get_delay () { return delay; };
    void get_refcog (double* ret)
    {
      ret[0] = x_k(0,0);
      ret[1] = x_k(0,1);
      ret[2] = cog_z;
    };
    void get_cart_zmp (double* ret)
    {
      Eigen::Matrix<double, 1, 2> _p(tcc * x_k);
      ret[0] = _p(0, 0);
      ret[1] = _p(0, 1);
      ret[2] = zmp_z;
    };
    void get_current_refzmp (double* ret)
    {
      ret[0] = p.front()(0);
      ret[1] = p.front()(1);
      ret[2] = zmp_z;
    };
    bool is_doing () { return p.size() >= 1 + delay; };
    bool is_end () { return ending_count <= 0 ; };
    void remove_preview_queue(const size_t remain_length)
    {
      size_t num = p.size() - remain_length;
      for (size_t i = 0; i < num; i++) p.pop_back();
    };
    void print_all_queue ()
    {
      std::cerr << "(list ";
      for (size_t i = 0; i < p.size(); i++) {
        std::cerr << "#f(" << p[i](0) << " " << p[i](1) << ") ";
      }
      std::cerr << ")" << std::endl;
    }
  };

  class preview_control : public preview_control_base<3>
  {
  private:
    void calc_f();
    void calc_u();
    void calc_x_k();
  public:
    preview_control(const double dt, const double zc,
                    const hrp::Vector3& init_xk, const double q = 1.0,
                    const double r = 1.0e-6, const double d = 1.6)
      : preview_control_base<3>(dt, zc, init_xk, d)
    {
      init_riccati(tcA, tcb, tcc, q, r);
    };
    virtual ~preview_control() {};
  };

  class extended_preview_control : public preview_control_base<4>
  {
  private:
    Eigen::Matrix<double, 4, 2> x_k_e;
    void calc_f();
    void calc_u();
    void calc_x_k();
  public:
    extended_preview_control(const double dt, const double zc,
                             const hrp::Vector3& init_xk, const double q = 1.0,
                             const double r = 1.0e-6, const double d = 1.6)
      : preview_control_base<4>(dt, zc, init_xk, d), x_k_e(Eigen::Matrix<double, 4, 2>::Zero())
    {
      Eigen::Matrix<double, 4, 4> A;
      Eigen::Matrix<double, 4, 1> b;
      Eigen::Matrix<double, 1, 4> c;
      Eigen::Matrix<double, 1, 3> tmpca(tcc * tcA);
      Eigen::Matrix<double, 1, 1> tmpcb(tcc * tcb);
      A << 1.0, tmpca(0,0), tmpca(0,1), tmpca(0,2),
        0.0, tcA(0,0), tcA(0,1), tcA(0,2),
        0.0, tcA(1,0), tcA(1,1), tcA(1,2),
        0.0, tcA(2,0), tcA(2,1), tcA(2,2);
      b << tmpcb(0,0),
        tcb(0,0),
        tcb(1,0),
        tcb(2,0);
      c << 1,0,0,0;
      x_k_e(0,0) = init_xk(0);
      x_k_e(0,1) = init_xk(1);
      init_riccati(A, b, c, q, r);
    };
    virtual ~extended_preview_control() {};
  };

  template <class previw_T>
  class preview_dynamics_filter
  {
    previw_T preview_controller;
    bool finishedp;
  public:
    preview_dynamics_filter() {};
    preview_dynamics_filter(const double dt, const double zc, const hrp::Vector3& init_xk, const double q = 1.0, const double r = 1.0e-6, const double d = 1.6)
      : preview_controller(dt, zc, init_xk, q, r, d), finishedp(false) {};
    ~preview_dynamics_filter() {};  
    bool update(hrp::Vector3& p_ret, hrp::Vector3& x_ret, const hrp::Vector3& pr, const bool updatep)
    {
      bool flg;
      if (updatep) {
        preview_controller.update_x_k(pr);
        flg = preview_controller.is_doing();
      } else {
        if ( !preview_controller.is_end() )
          preview_controller.update_x_k();
        flg = !preview_controller.is_end();
      }

      if (flg) {
        preview_controller.get_current_refzmp(p_ret.data());
        preview_controller.get_refcog(x_ret.data());
      }
      return flg;
    };
    void remove_preview_queue(const size_t remain_length)
    {
      preview_controller.remove_preview_queue(remain_length);
    };
    void print_all_queue ()
    {
      preview_controller.print_all_queue();
    }

    void get_cart_zmp (double* ret) { preview_controller.get_cart_zmp(ret);}
    void get_current_refzmp (double* ret) { preview_controller.get_current_refzmp(ret);}
  };
}
#endif /*PREVIEW_H_*/
