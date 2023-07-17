#include <gtest/gtest.h>
#include <similarity_transformation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

TEST(SimilarityTransformation, Test) {
  std::vector<Eigen::Vector3d> source;
  std::vector<Eigen::Vector3d> target;

  double scale_true{1.23456789};
  Eigen::Matrix3d rot_true{Eigen::Quaterniond::UnitRandom().toRotationMatrix()};
  Eigen::Vector3d trans_true{Eigen::Vector3d::Random()};

  for (auto i = 0; i < 10; ++i) {
    Eigen::Vector3d p{Eigen::Vector3d::Random()};
    Eigen::Vector3d q{rot_true * scale_true * p + trans_true};

    source.push_back(p);
    target.push_back(q);
  }

  SimilarityTransformation st;
  st.estimate(source, target);

  ASSERT_LT((st.translation() - trans_true).norm(), 1e-12);
  ASSERT_LT((st.rotation() - rot_true).norm(), 1e-12);
  ASSERT_LT(std::abs(st.scale() - scale_true), 1e-12);
}
