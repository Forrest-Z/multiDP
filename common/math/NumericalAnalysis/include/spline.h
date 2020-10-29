/*
 * spline.h
 *
 * simple cubic spline interpolation library
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2011, 2014 Tino Kluge (ttk448 at gmail.com)
 * modified by Hu.ZH(CrossOcean.ai)
 * ---------------------------------------------------------------------
 *
 */

#ifndef TK_SPLINE_H
#define TK_SPLINE_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <cstdio>
#include <iostream>
#include <vector>

namespace ASV::common::math {

// cubic spline interpolation (n = 3), twice continuously differentiable
class spline {
 public:
  enum class bd_type { first_deriv = 1, second_deriv = 2 };

 private:
  Eigen::VectorXd m_x, m_y;  // x,y coordinates of points
  std::size_t n;             // length of m_x and m_y
  double m_b0, m_c0;         // for left extrapol
  bd_type m_left, m_right;
  double m_left_value, m_right_value;
  bool m_force_linear_extrapolation;
  // interpolation parameters
  // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
  Eigen::VectorXd m_a, m_b, m_c;  // spline coefficients

  // maybe sort x and y (pairs)
  void sort_vectorpair(Eigen::VectorXd& _x, Eigen::VectorXd& _y) {
    // Declaring vector of pairs
    std::vector<std::pair<double, double> > vect_pair;
    for (std::size_t i = 0; i != n; ++i)
      vect_pair.push_back(std::make_pair(_x(i), _y(i)));
    // Using simple sort() function to sort
    std::sort(vect_pair.begin(), vect_pair.end());
    for (std::size_t i = 0; i != n; ++i) {
      _x(i) = vect_pair[i].first;
      _y(i) = vect_pair[i].second;
    }
  }  // sort_vectorpair()

  std::size_t find_closestindex(double _x) const {
    // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
    std::size_t idx = n - 1;
    for (std::size_t i = 1; i != n; i++) {
      if (m_x(i) >= _x) {
        idx = (i == 0 ? 0 : i - 1);
        break;
      }
    }
    return idx;
  }  // find_closestindex

 public:
  // set default boundary condition to be zero curvature at both ends
  spline()
      : n(0),
        m_b0(0.0),
        m_c0(0.0),
        m_left(bd_type::second_deriv),
        m_right(bd_type::second_deriv),
        m_left_value(0.0),
        m_right_value(0.0),
        m_force_linear_extrapolation(false) {}
  virtual ~spline() = default;

  // optional, but if called it has to come be before set_points()
  void set_boundary(bd_type left, double left_value, bd_type right,
                    double right_value,
                    bool force_linear_extrapolation = false) {
    assert(n == 0);  // set_points() must not have happened yet
    m_left = left;
    m_right = right;
    m_left_value = left_value;
    m_right_value = right_value;
    m_force_linear_extrapolation = force_linear_extrapolation;
  }  // set_boundary()

  void set_points(const Eigen::VectorXd& x, const Eigen::VectorXd& y,
                  bool cubic_spline = true) {
    assert(x.size() == y.size());
    assert(x.size() > 2);
    n = x.size();
    m_x = x;
    m_y = y;
    sort_vectorpair(m_x, m_y);

    if (cubic_spline == true) {  // cubic spline interpolation
      // setting up the matrix and right hand side of the equation system
      // for the parameters b[]
      Eigen::MatrixXd A(n, n);
      Eigen::VectorXd rhs(n);
      for (std::size_t i = 1; i < n - 1; i++) {
        A(i, i - 1) = 1.0 / 3.0 * (m_x(i) - m_x(i - 1));
        A(i, i) = 2.0 / 3.0 * (m_x(i + 1) - m_x(i - 1));
        A(i, i + 1) = 1.0 / 3.0 * (m_x(i + 1) - m_x(i));
        rhs(i) = (m_y(i + 1) - m_y(i)) / (m_x(i + 1) - m_x(i)) -
                 (m_y(i) - m_y(i - 1)) / (m_x(i) - m_x(i - 1));
      }
      // boundary conditions
      if (m_left == bd_type::second_deriv) {
        // 2*b[0] = f''
        A(0, 0) = 2.0;
        A(0, 1) = 0.0;
        rhs(0) = m_left_value;
      } else if (m_left == bd_type::first_deriv) {
        // c[0] = f', needs to be re-expressed in terms of b:
        // (2b[0]+b[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
        A(0, 0) = 2.0 * (m_x(1) - m_x(0));
        A(0, 1) = 1.0 * (m_x(1) - m_x(0));
        rhs(0) = 3.0 * ((m_y(1) - m_y(0)) / (m_x(1) - m_x(0)) - m_left_value);
      } else {
        assert(false);
      }
      if (m_right == bd_type::second_deriv) {
        // 2*b[n-1] = f''
        A(n - 1, n - 1) = 2.0;
        A(n - 1, n - 2) = 0.0;
        rhs(n - 1) = m_right_value;
      } else if (m_right == bd_type::first_deriv) {
        // c[n-1] = f', needs to be re-expressed in terms of b:
        // (b[n-2]+2b[n-1])(x[n-1]-x[n-2])
        // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
        A(n - 1, n - 1) = 2.0 * (m_x(n - 1) - m_x(n - 2));
        A(n - 1, n - 2) = 1.0 * (m_x(n - 1) - m_x(n - 2));
        rhs(n - 1) = 3.0 * (m_right_value - (m_y(n - 1) - m_y(n - 2)) /
                                                (m_x(n - 1) - m_x(n - 2)));
      } else {
        assert(false);
      }

      // solve the equation system to obtain the parameters b[]
      m_b = A.householderQr().solve(rhs);

      // calculate parameters a[] and c[] based on b[]
      m_a.resize(n);
      m_c.resize(n);
      for (std::size_t i = 0; i < n - 1; i++) {
        m_a(i) = 1.0 / 3.0 * (m_b(i + 1) - m_b(i)) / (m_x(i + 1) - m_x(i));
        m_c(i) =
            (m_y(i + 1) - m_y(i)) / (m_x(i + 1) - m_x(i)) -
            1.0 / 3.0 * (2.0 * m_b(i) + m_b(i + 1)) * (m_x(i + 1) - m_x(i));
      }

    } else {  // linear interpolation
      m_a.resize(n);
      m_b.resize(n);
      m_c.resize(n);
      for (std::size_t i = 0; i < n - 1; i++) {
        m_a(i) = 0.0;
        m_b(i) = 0.0;
        m_c(i) = (m_y(i + 1) - m_y(i)) / (m_x(i + 1) - m_x(i));
      }
    }

    // for left extrapolation coefficients
    m_b0 = (m_force_linear_extrapolation == false) ? m_b(0) : 0.0;
    m_c0 = m_c(0);

    // for the right extrapolation coefficients
    // f_{n-1}(x) = b*(x-x_{n-1})^2 + c*(x-x_{n-1}) + y_{n-1}
    double h = m_x(n - 1) - m_x(n - 2);
    // m_b[n-1] is determined by the boundary condition
    m_a(n - 1) = 0.0;
    m_c(n - 1) = 3.0 * m_a(n - 2) * h * h + 2.0 * m_b(n - 2) * h +
                 m_c(n - 2);  // = f'_{n-2}(x_{n-1})

    if (m_force_linear_extrapolation == true) m_b(n - 1) = 0.0;
  }  // set_points

  double operator()(double x) const {
    std::size_t idx = find_closestindex(x);
    double h = x - m_x(idx);
    double interpol = 0.0;
    if (x < m_x(0)) {
      // extrapolation to the left
      interpol = (m_b0 * h + m_c0) * h + m_y(0);
    } else if (x > m_x(n - 1)) {
      // extrapolation to the right
      interpol = (m_b(n - 1) * h + m_c(n - 1)) * h + m_y(n - 1);
    } else {
      // interpolation
      interpol = ((m_a(idx) * h + m_b(idx)) * h + m_c(idx)) * h + m_y(idx);
    }
    return interpol;
  }  // operator()

  double deriv(int order, double x) const {
    assert(order > 0);

    std::size_t idx = find_closestindex(x);
    double h = x - m_x(idx);
    double interpol = 0.0;
    if (x < m_x(0)) {
      // extrapolation to the left
      switch (order) {
        case 1:
          interpol = 2.0 * m_b0 * h + m_c0;
          break;
        case 2:
          interpol = 2.0 * m_b0;
          break;
        default:
          interpol = 0.0;
          break;
      }
    } else if (x > m_x(n - 1)) {
      // extrapolation to the right
      switch (order) {
        case 1:
          interpol = 2.0 * m_b(n - 1) * h + m_c(n - 1);
          break;
        case 2:
          interpol = 2.0 * m_b(n - 1);
          break;
        default:
          interpol = 0.0;
          break;
      }
    } else {
      // interpolation
      switch (order) {
        case 1:
          // calculate the first derivative
          interpol = (3.0 * m_a(idx) * h + 2.0 * m_b(idx)) * h + m_c(idx);
          break;
        case 2:
          // calculate the second derivative
          interpol = 6.0 * m_a(idx) * h + 2.0 * m_b(idx);
          break;
        case 3:
          // calculate the third derivative
          interpol = 6.0 * m_a(idx);
          break;
        default:
          interpol = 0.0;
          break;
      }
    }
    return interpol;
  }  // deriv()
};   // end class spline

// two-dimensional spline with analytic form
class Spline2D {
 public:
  Spline2D(const Eigen::VectorXd& _x, const Eigen::VectorXd& _y)
      : n(0), X_2d(_x), Y_2d(_y) {
    setupspline2d();
  }

  virtual ~Spline2D() = default;
  // re-calculate the interpolation
  void reinterpolation(const Eigen::VectorXd& _x, const Eigen::VectorXd& _y) {
    X_2d = _x;
    Y_2d = _y;
    setupspline2d();
  }  // reinterpolation
  // calculate the x,y based on the arclength
  Eigen::Vector2d compute_position(double _arclength) const {
    return (Eigen::Vector2d() << SX_(_arclength), SY_(_arclength)).finished();
  }  // compute_position

  // calculate the curvature based on the arclength
  double compute_curvature(double _arclength) const {
    double dx = SX_.deriv(1, _arclength);
    double ddx = SX_.deriv(2, _arclength);
    double dy = SY_.deriv(1, _arclength);
    double ddy = SY_.deriv(2, _arclength);
    // kappa = (ddy * dx - ddx * dy) /
    //         std::pow(std::pow(dx, 2) + std::pow(dy, 2), 1.5);
    // avoid using sqrt to speed up
    double kappa = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
    return kappa;
  }  // compute_curvature

  // calculate the derivative of curvature to arclength,
  // WARNING: As cubic spline is twice differentiable, causing the
  // disconinuty in dk/ds
  double compute_dcurvature(double _arclength) const {
    double dx = SX_.deriv(1, _arclength);
    double ddx = SX_.deriv(2, _arclength);
    double dddx = SX_.deriv(3, _arclength);
    double dy = SY_.deriv(1, _arclength);
    double ddy = SY_.deriv(2, _arclength);
    double dddy = SY_.deriv(3, _arclength);

    double squareterm = dx * dx + dy * dy;
    // dkappa = ((dddy * dx - dddx * dy) * squareterm -
    //           3 * (ddy * dx - ddx * dy) * (dx * ddx + dy * ddy)) /
    //          std::pow(squareterm, 2.5);
    // avoid using sqrt to speed up
    double dkappa = ((dddy * dx - dddx * dy) * squareterm -
                     3 * (ddy * dx - ddx * dy) * (dx * ddx + dy * ddy)) /
                    (squareterm * squareterm);

    return dkappa;
  }  // compute_dcurvature

  // calculate the orientation based on the arclength
  double compute_yaw(double _arclength) const {
    double dx = SX_.deriv(1, _arclength);
    double dy = SY_.deriv(1, _arclength);
    return std::atan2(dy, dx);
  }  // compute_yaw
  Eigen::VectorXd arclength() const { return arclength_; }

 private:
  std::size_t n;
  Eigen::VectorXd X_2d, Y_2d;
  Eigen::VectorXd arclength_;

  spline SX_;
  spline SY_;

  void setupspline2d() {
    compute_arclength();
    SX_.set_points(arclength_, X_2d);
    SY_.set_points(arclength_, Y_2d);
  }  // setupspline2d

  void compute_arclength() {
    assert(X_2d.size() == Y_2d.size());
    assert(X_2d.size() > 2);
    n = X_2d.size();

    arclength_.resize(n);
    arclength_(0) = 0.0;

    for (std::size_t i = 0; i != (n - 1); i++) {
      double dx = X_2d(i + 1) - X_2d(i);
      double dy = Y_2d(i + 1) - Y_2d(i);
      arclength_(i + 1) =
          arclength_(i) + std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    }
  }  // compute_arclength
};   // class spline2d

// g(x)= a5 x^5 + a4 x^4 + a3 x^3 + a2 x^2 + a1 x + a0
template <std::size_t order = 5>
class polynomialvalue {
  using polyvector = Eigen::Matrix<double, order + 1, 1>;

 public:
  polynomialvalue() : a(polyvector::Zero()) {}
  polynomialvalue(const polyvector& _a) : a(_a) {}
  virtual ~polynomialvalue() = default;

  // calculate the derivtive of polynomial
  template <std::size_t _order = 0>
  double compute_order_derivative(double _x) const {
    double results = 0.0;
    if constexpr (_order <= order) {
      for (std::size_t i = 0; i != (order + 1 - _order); i++) {
        std::size_t t_order = 1;
        for (std::size_t j = 0; j != _order; j++) t_order *= (order - i - j);
        results = results * _x + t_order * a(i);
      }
    }
    return results;
  }  // compute_order_derivative

  void setcofficient(const polyvector& _a) { a = _a; }
  polyvector getcofficient() const { return a; }

 protected:
  polyvector a;  // coefficent; a={a5, a4, a3, a2, a1, a0}
};               // class polynomialvalue

}  // namespace ASV::common::math

#endif /* TK_SPLINE_H */
