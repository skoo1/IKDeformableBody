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
#include "skel.h"
#include "vnormal.h"


namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) { }

  void init() final { }

  void reset() final {
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {

    Eigen::VectorXd trans(3);
    for(int i = 0; i < 3; i++) {
      trans(i) = static_cast<double>(action[i]);
    }

    Eigen::VectorXd pose(46);
    for(int i = 0; i < 46; i++) {
      pose(i) = static_cast<double>(action[3 + i]);
    }

    auto skel_r = skel_->SKEL_Mesh_Calc(pose, betas_, trans);
    // skel_r는 (N,3) double matrix

    // 첫 번째 비용(c1) 계산
    Eigen::MatrixXd skel_positions(indices_.size(), 3);
    for (size_t i = 0; i < indices_.size(); i++) {
      skel_positions.row(i) = skel_r.row(indices_[i]); // 전체 행 복사
    }
    
    Eigen::MatrixXf diff = skel_positions.cast<float>() - trc_/1000.0;

    IK_weights_ = Eigen::VectorXf(39);
    IK_weights_.setOnes();
    IK_weights_ << 1.0f, 1.0f, 1.0f, 1.0f,
                   1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                   1.0f, 0.20000000000000001f, 1.0f, 0.20000000000000001f, 1.0f, 1.0f, 1.0f,
                   1.0f, 0.20000000000000001f, 1.0f, 0.20000000000000001f, 1.0f, 1.0f, 1.0f,
                   10.0f, 10.0f, 10.0f, 10.f,
                   0.20000000000000001f, 1.0f, 0.20000000000000001f, 1.0f, 1.0f, 1.0f,
                   0.20000000000000001f, 1.0f, 0.20000000000000001f, 1.0f, 1.0f, 1.0f;

    float cost = 0.f;
    for (int i = 0; i < diff.rows(); ++i) {
        float d = diff.row(i).norm();
        cost += IK_weights_[i] * d * d;
    }
    cost = std::sqrt(cost / diff.rows()) * 1000.f;
    
    return cost;
  }

  void build(const Eigen::Ref<EigenRowMajorMat>& trc, const std::vector<std::string>& markers, const Eigen::Ref<Eigen::VectorXi>& indices, const Eigen::Ref<EigenVec>& betas, const Eigen::MatrixXi& f) final {
    trc_.resize(trc.rows(), trc.cols());
    trc_ = trc;
    markers_ = markers;
    indices_ = indices;
    betas_ = betas.cast<double>();
    f_ = f;

    // Build SKEL Model
    {
      std::string model_path = "skel_male.json";
      skel_ = std::make_unique<SKELModel>(model_path);
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

  Eigen::Matrix<float, -1, -1> trc_;
  std::vector<std::string> markers_;
  Eigen::VectorXi indices_;
  std::unique_ptr<SKELModel> skel_;
  Eigen::VectorXd betas_;
  Eigen::MatrixXi f_;
  Eigen::VectorXf IK_weights_;
};
}
