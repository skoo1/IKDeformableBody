//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include <memory>
#include "RaisimGymEnv.hpp"
#include "kdtree.h"
#include "smpl.h"


namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {
  }

  void init() final { }

  void reset() final {
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {

    Eigen::VectorXd trans(3), pose(72);
    for(int i = 0; i < 3; i++) {
      trans(i) = static_cast<double>(action[i]);
    }
    for(int i = 0; i < 72; i++) {
      pose(i) = static_cast<double>(action[3 + i]);
    }

    auto [smpl_r, _, joints] = smpl_->SMPL_Calc(trans, pose, betas_);
    // smpl_r는 (N,3) double matrix

    // 첫 번째 비용(c1) 계산
    std::vector<KDTree::Point3f> smplPoints(smpl_r.rows());
    for(int i = 0; i < smpl_r.rows(); i++) {
      smplPoints[i][0] = static_cast<float>(smpl_r(i, 0));
      smplPoints[i][1] = static_cast<float>(smpl_r(i, 1));
      smplPoints[i][2] = static_cast<float>(smpl_r(i, 2));
    }    
    auto [dists, indices] = tree_->query(smplPoints);
    float c1 = 0.f;
    for (size_t i = 0; i < smplPoints.size(); i++) {
      int idx = indices[i];  // KDTree가 반환한 최근접 점 인덱스

      float vx = vertices_(idx, 0);
      float vy = vertices_(idx, 1);
      float vz = vertices_(idx, 2);

      // 해당 인덱스의 노멀
      float nx = normals_(idx, 0);
      float ny = normals_(idx, 1);
      float nz = normals_(idx, 2);

      // displacement = smpl_r(i,:) - vertices_[idx,:]
      float dx = smplPoints[i][0] - vertices_(idx, 0);
      float dy = smplPoints[i][1] - vertices_(idx, 1);
      float dz = smplPoints[i][2] - vertices_(idx, 2);

      // dot = displacement dot normal
      float dot = dx * nx + dy * ny + dz * nz;

      c1 += dot * dot; // (dot^2)
    }

    // smpl_r로 새 KD트리 생성
    KDTree smplTree(smplPoints);

    // vertices_ 전체를 질의 대상
    std::vector<KDTree::Point3f> vertPoints(vertices_.rows());
    for(int i = 0; i < vertices_.rows(); i++) {
      vertPoints[i][0] = vertices_(i, 0);
      vertPoints[i][1] = vertices_(i, 1);
      vertPoints[i][2] = vertices_(i, 2);
    }

    auto [dists2, inds2] = smplTree.query(vertPoints);

    float c2 = 0.f;
    // c2 = sum of (dists2^2)
    for (auto distVal : dists2) {
      c2 += distVal * distVal;
    }

    // 최종 cost = (c1 + c2) * 100
    float cost = (c1 + c2) * 100.f;

    return cost;
  }

  void build(const Eigen::Ref<EigenRowMajorMat>& vertices, const Eigen::Ref<EigenRowMajorMat>& normals, const Eigen::Ref<EigenVec>& betas) final {
    vertices_.resize(vertices.rows(), vertices.cols());
    vertices_ = vertices;
    normals_.resize(normals.rows(), normals.cols());
    normals_ = normals;
    betas_ = betas.cast<double>();

    // Build SMPL Model
    {
      std::string model_path = "basicmodel_m_lbs_10_207_0_v1.1.0.json";
      smpl_ = std::make_unique<SMPLModel>(model_path);
    }

    // Build KDTree for input vertices
    {
      std::vector<KDTree::Point3f> points(vertices.rows());
      for (int i = 0; i < vertices.rows(); ++i) {
        points[i] = {
            vertices(i, 0),
            vertices(i, 1),
            vertices(i, 2)
        };
      }

      tree_ = std::make_unique<KDTree>(points);
    }
  }

  void updateObservation() {
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  Eigen::Matrix<float, -1, -1> vertices_;
  Eigen::Matrix<float, -1, -1> normals_;
  std::unique_ptr<SMPLModel> smpl_;
  std::unique_ptr<KDTree> tree_;
  Eigen::VectorXd betas_;
};
}
