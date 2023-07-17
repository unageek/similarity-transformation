#pragma once

#include <manif/SE3.h>

#include <Eigen/Core>
#include <vector>

class SimilarityTransformation {
  static constexpr int DimN = 7;
  static constexpr int DimM = Eigen::Dynamic;
  using Vec3 = Eigen::Vector3d;
  using VecN = Eigen::Vector<double, DimN>;
  using VecM = Eigen::Vector<double, DimM>;
  using Mat3 = Eigen::Matrix3d;
  using MatMN = Eigen::Matrix<double, DimM, DimN>;
  using SE3 = manif::SE3d;
  using SE3Tangent = manif::SE3Tangentd;

 public:
  Mat3 rotation() const { return state_.m.rotation(); }

  Vec3 translation() const { return state_.m.translation(); }

  double scale() const { return state_.s; }

  void estimate(const std::vector<Vec3>& source, const std::vector<Vec3>& target) {
    auto x = state_;
    auto n = static_cast<int>(source.size());
    VecM innovation{VecM::Zero(3 * n)};
    MatMN j;
    for (auto iter = 0; iter < 20; ++iter) {
      for (auto i = 0; i < n; ++i) {
        const auto& p = source.at(i);
        const auto& q = target.at(i);
        innovation.segment<3>(3 * i) = q - x.m.act(x.s * p);
      }
      jacobian(x, source, j);
      VecN delta_x{(j.transpose() * j).inverse() * j.transpose() * innovation};
      x = x + delta_x;
    }
    state_ = x;
  }

 private:
  struct State {
    SE3 m{SE3::Identity()};
    double s{1.0};

    State operator+(const VecN& rhs) const {
      State x;
      x.m = m + SE3Tangent{rhs.head<6>()};
      x.s = s + rhs(6);
      return x;
    }
  };

  static void jacobian(const State& x, const std::vector<Vec3>& source, MatMN& j) {
    auto n = static_cast<int>(source.size());
    auto r = x.m.rotation();
    j.resize(3 * n, DimN);
    for (auto i = 0; i < n; ++i) {
      const auto& p = source.at(i);
      auto sp_cross = manif::skew(x.s * p);
      j.block<3, 3>(3 * i, 0) = r;
      j.block<3, 3>(3 * i, 3) = -r * sp_cross;
      j.block<3, 1>(3 * i, 6) = r * p;
    }
  }

  State state_;
};
