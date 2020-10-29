/*
***********************************************************************
* Reeds_Shepp.h:
* Generate the Reeds Shepp curves from starting state to ending state.
* The comments, variable names, etc. use the nomenclature
* from the Reeds & Shepp paper.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _REEDS_SHEPP_H_
#define _REEDS_SHEPP_H_

#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include <iostream>

namespace ASV::common::math {

enum ReedsSheppPathSegmentType {
  RS_NOP = 0,
  RS_LEFT = 1,
  RS_STRAIGHT = 2,
  RS_RIGHT = 3
};

const ReedsSheppPathSegmentType reedsSheppPathType[18][5] = {
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
};

struct ReedsSheppPath {
  ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                 double t = std::numeric_limits<double>::max(), double u = 0.,
                 double v = 0., double w = 0., double x = 0.)
      : type_(type) {
    length_[0] = t;
    length_[1] = u;
    length_[2] = v;
    length_[3] = w;
    length_[4] = x;
    totalLength_ = std::fabs(t) + std::fabs(u) + std::fabs(v) + std::fabs(w) +
                   std::fabs(x);
  }

  double length() const noexcept { return totalLength_; }
  const ReedsSheppPathSegmentType *type_;
  double length_[5];
  double totalLength_;
};  // end struct ReedsSheppPath

class ReedsSheppStateSpace {
 public:
  ReedsSheppStateSpace(double turningRadius = 1.0)
      : rho_(turningRadius),
        ZERO_(10 * std::numeric_limits<double>::epsilon()),
        RS_EPS_(1e-6),
        twopi_(2. * M_PI) {}
  virtual ~ReedsSheppStateSpace() = default;

  double rs_distance(const std::array<double, 3> &q0,
                     const std::array<double, 3> &q1) const {
    return rho_ * reedsShepp(q0, q1).length();
  }  // rs_distance

  std::array<int, 5> rs_type(const std::array<double, 3> &q0,
                             const std::array<double, 3> &q1) {
    ReedsSheppPath path = reedsShepp(q0, q1);
    std::array<int, 5> types;
    for (std::size_t i = 0; i < 5; ++i) {
      types[i] = static_cast<int>(path.type_[i]);
    }
    return types;
  }  // rs_type

  // interpolate the rs curve, and separate the curve when reverse/forward
  // switch occurs
  std::vector<std::tuple<double, double, double, bool>> rs_trajectory(
      const std::array<double, 3> &q0, const std::array<double, 3> &q1,
      double step_size) const {
    ReedsSheppPath path = reedsShepp(q0, q1);

    // separate each segment of rs curve
    std::vector<std::array<double, 2>> segment_arclengths;
    std::vector<bool> segment_IsForward;

    double current_seg = 0.0;
    segment_arclengths.push_back({current_seg, path.length()});
    segment_IsForward.push_back((path.length_[0] > 0));
    for (std::size_t i = 0; i != 4; i++) {
      current_seg += std::abs(path.length_[i]);
      // if there exists a switch point
      if (path.length_[i] * path.length_[i + 1] < 0) {
        segment_arclengths.back().at(1) = current_seg;
        segment_arclengths.push_back({current_seg, path.length()});
        segment_IsForward.push_back((path.length_[i + 1] > 0));
      }
    }

    // interploate
    std::size_t num_segment = segment_arclengths.size();
    std::vector<std::tuple<double, double, double, bool>> results;
    // results.resize(num_segment);

    for (std::size_t index = 0; index != num_segment; ++index) {
      double current_arclength = segment_arclengths[index].at(0);
      while (current_arclength < segment_arclengths[index].at(1)) {
        auto interpolate_results = interpolate(q0, path, current_arclength);
        results.push_back({interpolate_results[0], interpolate_results[1],
                           interpolate_results[2], segment_IsForward[index]});
        current_arclength += (step_size / rho_);
      }
      // Don't forget the end point
      auto interpolate_results =
          interpolate(q0, path, segment_arclengths[index].at(1));
      results.push_back({interpolate_results[0], interpolate_results[1],
                         interpolate_results[2], segment_IsForward[index]});
    }

    return results;
  }  // rs_trajectory

  std::vector<std::array<double, 3>> rs_state(const std::array<double, 3> &q0,
                                              const std::array<double, 3> &q1,
                                              double step_size) const {
    ReedsSheppPath path = reedsShepp(q0, q1);
    double dist = rho_ * path.length();
    std::vector<std::array<double, 3>> result;

    for (double arclength = 0.0; arclength < dist; arclength += step_size) {
      result.emplace_back(interpolate(q0, path, arclength / rho_));
    }
    result.emplace_back(q1);

    return result;
  }  // rs_state

  ReedsSheppPath reedsShepp(const std::array<double, 3> &q0,
                            const std::array<double, 3> &q1) const {
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double dth = q1[2] - q0[2];
    double c = std::cos(q0[2]);
    double s = std::sin(q0[2]);
    double x = c * dx + s * dy, y = -s * dx + c * dy;

    return reedsShepp(x / rho_, y / rho_, dth);
  }  // reedsShepp

  std::array<double, 3> interpolate(const std::array<double, 3> &q0,
                                    const ReedsSheppPath &path,
                                    double seg) const {
    if (seg < 0.0) seg = 0.0;
    if (seg > path.length()) seg = path.length();

    double phi = 0.0;
    double v = 0.0;

    std::array<double, 3> s = {0.0, 0.0, 0.0};

    s[0] = s[1] = 0.0;
    s[2] = q0[2];

    for (std::size_t i = 0; i < 5 && seg > 0; ++i) {
      if (path.length_[i] < 0) {
        v = std::max(-seg, path.length_[i]);
        seg += v;
      } else {
        v = std::min(seg, path.length_[i]);
        seg -= v;
      }
      phi = s[2];
      switch (path.type_[i]) {
        case RS_LEFT:
          s[0] += (std::sin(phi + v) - std::sin(phi));
          s[1] += (-std::cos(phi + v) + std::cos(phi));
          s[2] = phi + v;
          break;
        case RS_RIGHT:
          s[0] += (-std::sin(phi - v) + std::sin(phi));
          s[1] += (std::cos(phi - v) - std::cos(phi));
          s[2] = phi - v;
          break;
        case RS_STRAIGHT:
          s[0] += (v * std::cos(phi));
          s[1] += (v * std::sin(phi));
          break;
        case RS_NOP:
          break;
      }
    }

    s[0] = s[0] * rho_ + q0[0];
    s[1] = s[1] * rho_ + q0[1];

    return s;
  }  // interpolate

 protected:
  double rho_;  // TURNNING RADIUS

 private:
  const double ZERO_;
  const double RS_EPS_;
  const double twopi_;

  ReedsSheppPath reedsShepp(double x, double y, double phi) const {
    ReedsSheppPath path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    return path;
  }  // reedsShepp

  inline double mod2pi(double x) const {
    double v = std::fmod(x, twopi_);
    if (v < -M_PI)
      v += twopi_;
    else if (v > M_PI)
      v -= twopi_;
    return v;
  }  // mod2pi

  inline void polar(double x, double y, double &r, double &theta) const {
    r = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
  }  // polar

  inline void tauOmega(double u, double v, double xi, double eta, double phi,
                       double &tau, double &omega) const {
    double delta = mod2pi(u - v), A = std::sin(u) - std::sin(delta),
           B = std::cos(u) - std::cos(delta) - 1.;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B),
           t2 = 2. * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3;
    tau = (t2 < 0) ? mod2pi(t1 + M_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
  }  // tauOmega

  // formula 8.1 in Reeds-Shepp paper
  inline bool LpSpLp(double x, double y, double phi, double &t, double &u,
                     double &v) const {
    polar(x - std::sin(phi), y - 1. + std::cos(phi), u, t);
    if (t >= -ZERO_) {
      v = mod2pi(phi - t);
      if (v >= -ZERO_) {
        assert(std::fabs(u * std::cos(t) + std::sin(phi) - x) < RS_EPS_);
        assert(std::fabs(u * std::sin(t) - std::cos(phi) + 1 - y) < RS_EPS_);
        assert(std::fabs(mod2pi(t + v - phi)) < RS_EPS_);
        return true;
      }
    }
    return false;
  }  // LpSpLp

  // formula 8.2
  inline bool LpSpRp(double x, double y, double phi, double &t, double &u,
                     double &v) const {
    double t1, u1;
    polar(x + std::sin(phi), y - 1. - std::cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.) {
      double theta;
      u = std::sqrt(u1 - 4.);
      theta = std::atan2(2., u);
      t = mod2pi(t1 + theta);
      v = mod2pi(t - phi);
      assert(std::fabs(2 * std::sin(t) + u * std::cos(t) - std::sin(phi) - x) <
             RS_EPS_);
      assert(std::fabs(-2 * std::cos(t) + u * std::sin(t) + std::cos(phi) + 1 -
                       y) < RS_EPS_);
      assert(std::fabs(mod2pi(t - v - phi)) < RS_EPS_);
      return t >= -ZERO_ && v >= -ZERO_;
    }
    return false;
  }  // LpSpRp

  void CSC(double x, double y, double phi, ReedsSheppPath &path) const {
    double t, u, v, Lmin = path.length(), L;
    if (LpSpLp(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[14], t, u, v);
      Lmin = L;
    }
    if (LpSpLp(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[14], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpLp(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[15], t, u, v);
      Lmin = L;
    }
    if (LpSpLp(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[15], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpRp(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[12], t, u, v);
      Lmin = L;
    }
    if (LpSpRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[12], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[13], t, u, v);
      Lmin = L;
    }
    if (LpSpRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[13], -t, -u, -v);
  }  // CSC

  // formula 8.3 / 8.4  *** TYPO IN PAPER ***
  inline bool LpRmL(double x, double y, double phi, double &t, double &u,
                    double &v) const {
    double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi), u1, theta;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.) {
      u = -2. * std::asin(0.25 * u1);
      t = mod2pi(theta + 0.5 * u + M_PI);
      v = mod2pi(phi - t + u);
      assert(std::fabs(2 * (std::sin(t) - std::sin(t - u)) + std::sin(phi) -
                       x) < RS_EPS_);
      assert(std::fabs(2 * (-std::cos(t) + std::cos(t - u)) - std::cos(phi) +
                       1 - y) < RS_EPS_);
      assert(std::fabs(mod2pi(t - u + v - phi)) < RS_EPS_);
      return t >= -ZERO_ && u <= ZERO_;
    }
    return false;
  }  // LpRmL

  void CCC(double x, double y, double phi, ReedsSheppPath &path) const {
    double t, u, v, Lmin = path.length(), L;
    if (LpRmL(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[0], t, u, v);
      Lmin = L;
    }
    if (LpRmL(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[0], -t, -u, -v);
      Lmin = L;
    }
    if (LpRmL(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], t, u, v);
      Lmin = L;
    }
    if (LpRmL(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], -t, -u, -v);
      Lmin = L;
    }

    // backwards
    double xb = x * std::cos(phi) + y * std::sin(phi),
           yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[0], v, u, t);
      Lmin = L;
    }
    if (LpRmL(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[0], -v, -u, -t);
      Lmin = L;
    }
    if (LpRmL(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], v, u, t);
      Lmin = L;
    }
    if (LpRmL(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[1], -v, -u, -t);
  }  // CCC

  // formula 8.7
  inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u,
                         double &v) const {
    double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi),
           rho = 0.25 * (2. + std::sqrt(xi * xi + eta * eta));
    if (rho <= 1.) {
      u = std::acos(rho);
      tauOmega(u, -u, xi, eta, phi, t, v);
      assert(
          std::fabs(2 * (std::sin(t) - std::sin(t - u) + std::sin(t - 2 * u)) -
                    std::sin(phi) - x) < RS_EPS_);
      assert(
          std::fabs(2 * (-std::cos(t) + std::cos(t - u) - std::cos(t - 2 * u)) +
                    std::cos(phi) + 1 - y) < RS_EPS_);
      assert(std::fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS_);
      return t >= -ZERO_ && v <= ZERO_;
    }
    return false;
  }  // LpRupLumRm

  // formula 8.8
  inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u,
                         double &v) const {
    double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi),
           rho = (20. - xi * xi - eta * eta) / 16.;
    if (rho >= 0 && rho <= 1) {
      u = -std::acos(rho);
      if (u >= -0.5 * M_PI) {
        tauOmega(u, u, xi, eta, phi, t, v);
        assert(std::fabs(4 * std::sin(t) - 2 * std::sin(t - u) - std::sin(phi) -
                         x) < RS_EPS_);
        assert(std::fabs(-4 * std::cos(t) + 2 * std::cos(t - u) +
                         std::cos(phi) + 1 - y) < RS_EPS_);
        assert(std::fabs(mod2pi(t - v - phi)) < RS_EPS_);
        return t >= -ZERO_ && v >= -ZERO_;
      }
    }
    return false;
  }  // LpRumLumRp

  void CCCC(double x, double y, double phi, ReedsSheppPath &path) const {
    double t, u, v, Lmin = path.length(), L;
    if (LpRupLumRm(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[2], t, u, -u, v);
      Lmin = L;
    }
    if (LpRupLumRm(-x, y, -phi, t, u, v) &&
        Lmin >
            (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, u, -v);
      Lmin = L;
    }
    if (LpRupLumRm(x, -y, -phi, t, u, v) &&
        Lmin >
            (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], t, u, -u, v);
      Lmin = L;
    }
    if (LpRupLumRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + 2. * std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, u, -v);
      Lmin = L;
    }

    if (LpRumLumRp(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[2], t, u, u, v);
      Lmin = L;
    }
    if (LpRumLumRp(-x, y, -phi, t, u, v) &&
        Lmin >
            (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, -u, -v);
      Lmin = L;
    }
    if (LpRumLumRp(x, -y, -phi, t, u, v) &&
        Lmin >
            (L = std::fabs(t) + 2. * std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], t, u, u, v);
      Lmin = L;
    }
    if (LpRumLumRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + 2. * std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, -u, -v);
  }  // CCCC

  // formula 8.9
  inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u,
                       double &v) const {
    double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
      double r = std::sqrt(rho * rho - 4.);
      u = 2. - r;
      t = mod2pi(theta + std::atan2(r, -2.));
      v = mod2pi(phi - 0.5 * M_PI - t);
      assert(std::fabs(2 * (std::sin(t) - std::cos(t)) - u * std::sin(t) +
                       std::sin(phi) - x) < RS_EPS_);
      assert(std::fabs(-2 * (std::sin(t) + std::cos(t)) + u * std::cos(t) -
                       std::cos(phi) + 1 - y) < RS_EPS_);
      assert(std::fabs(mod2pi(t + 0.5 * M_PI + v - phi)) < RS_EPS_);
      return t >= -ZERO_ && u <= ZERO_ && v <= ZERO_;
    }
    return false;
  }  // LpRmSmLm

  // formula 8.10
  inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u,
                       double &v) const {
    double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho, theta;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.) {
      t = theta;
      u = 2. - rho;
      v = mod2pi(t + 0.5 * M_PI - phi);
      assert(std::fabs(2 * std::sin(t) - std::cos(t - v) - u * std::sin(t) -
                       x) < RS_EPS_);
      assert(std::fabs(-2 * std::cos(t) - std::sin(t - v) + u * std::cos(t) +
                       1 - y) < RS_EPS_);
      assert(std::fabs(mod2pi(t + 0.5 * M_PI - v - phi)) < RS_EPS_);
      return t >= -ZERO_ && u <= ZERO_ && v <= ZERO_;
    }
    return false;
  }  // LpRmSmRm

  void CCSC(double x, double y, double phi, ReedsSheppPath &path) const {
    double t, u, v, Lmin = path.length() - .5 * M_PI, L;
    if (LpRmSmLm(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[4], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmLm(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[4], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }
    if (LpRmSmLm(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[5], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmLm(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[5], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }

    if (LpRmSmRm(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[8], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmRm(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[8], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }
    if (LpRmSmRm(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[9], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[9], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }

    // backwards
    double xb = x * std::cos(phi) + y * std::sin(phi),
           yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[6], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmLm(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[6], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }
    if (LpRmSmLm(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[7], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmLm(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[7], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }

    if (LpRmSmRm(xb, yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[10], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmRm(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[10], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }
    if (LpRmSmRm(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[11], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmRm(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[11], -v, -u, .5 * M_PI, -t);
  }  // CCSC

  // formula 8.11 *** TYPO IN PAPER ***
  inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u,
                        double &v) const {
    double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
      u = 4. - std::sqrt(rho * rho - 4.);
      if (u <= ZERO_) {
        t = mod2pi(std::atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
        v = mod2pi(t - phi);
        assert(std::fabs(4 * std::sin(t) - 2 * std::cos(t) - u * std::sin(t) -
                         std::sin(phi) - x) < RS_EPS_);
        assert(std::fabs(-4 * std::cos(t) - 2 * std::sin(t) + u * std::cos(t) +
                         std::cos(phi) + 1 - y) < RS_EPS_);
        assert(std::fabs(mod2pi(t - v - phi)) < RS_EPS_);
        return t >= -ZERO_ && v >= -ZERO_;
      }
    }
    return false;
  }  // LpRmSLmRp

  void CCSCC(double x, double y, double phi, ReedsSheppPath &path) const {
    double t, u, v, Lmin = path.length() - M_PI, L;
    if (LpRmSLmRp(x, y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[16], t, -0.5 * M_PI, u,
                            -0.5 * M_PI, v);
      Lmin = L;
    }
    if (LpRmSLmRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[16], -t, 0.5 * M_PI, -u,
                            0.5 * M_PI, -v);
      Lmin = L;
    }
    if (LpRmSLmRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[17], t, -0.5 * M_PI, u,
                            -0.5 * M_PI, v);
      Lmin = L;
    }
    if (LpRmSLmRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = std::fabs(t) + std::fabs(u) +
                    std::fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[17], -t, 0.5 * M_PI, -u,
                            0.5 * M_PI, -v);
  }  // CCSCC
};   //  end class ReedsSheppStateSpace

}  // namespace ASV::common::math

#endif /* _REEDS_SHEPP_H_ */