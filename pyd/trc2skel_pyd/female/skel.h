#pragma once

#include "skel.h"
#include "osim_joints.h"
#include "joints_def.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

// SMPLModel 클래스
class SKELModel
{
public:
    // 생성자: JSON 경로로부터 모델 로드
    SKELModel(const std::string& model_path);

    // 소멸자
    ~SKELModel() = default;

    // pose, betas를 주면 최종 버텍스와 관절 위치 등을 계산
    // 반환: (verts, v_posed, joints)
    //  - verts: 스키닝 적용 후 최종 버텍스 좌표 (N,3)
    //  - v_posed: pose에 의한 posed 버텍스 (N,3)
    //  - joints: 글로벌 관절 위치 (24,3) 가정
    Eigen::MatrixXd SKEL_Mesh_Calc(const Eigen::VectorXd& pose, const Eigen::VectorXd& betas, const Eigen::Vector3d& trans);

private:
    // 모델 파싱 및 멤버 변수 세팅
    void loadModel(const std::string& model_path);
    void setup();

    // 로드된 JSON 데이터를 파싱해서 Eigen 형식 멤버에 대입
    void parseJson(const std::string& json_file);

    std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>> pose_params_to_rot(
        const Eigen::VectorXd& osim_poses,
        const std::vector<OsimJoint*>& joints_dict
    ) const;

    Eigen::Matrix3d rotation_matrix_from_vectors(
        const Eigen::Vector3d& vec1,
        const Eigen::Vector3d& vec2);

    std::vector<Eigen::Matrix3d> compute_bone_orientation(
        const Eigen::MatrixXd& J,             // (nJ, 3) global joint 위치
        const Eigen::MatrixXd& J_local       // (nJ, 3) local joint 위치
    );

    Eigen::Matrix3d matmul_chain(const std::vector<Matrix3d>& rot_list);

private:

    // constructor에서 생성하는 변수
    std::vector<OsimJoint*> joints_dict_;

    // SKEL 관련 멤버들
    Eigen::MatrixXd skin_template_v_;    // (N,3)
    Eigen::MatrixXd shapedirs_;     // (N*3, n_betas)
    Eigen::MatrixXd posedirs_;      // (N*3, n_posemap)
    Eigen::MatrixXd J_regressor_;   // (#joints, N)
    Eigen::MatrixXd J_regressor_osim_;   // (#joints, N)
    Eigen::MatrixXd skin_weights_;       // (N, #joints)
    Eigen::MatrixXi osim_kintree_table_; // (2, #joints=24)
    std::vector<Eigen::Matrix4d> apose_rel_transfo_;
    std::vector<Eigen::Matrix3d> per_joint_rot_;


    // pose, betas, trans의 기본값 (모델 로드 시점에 설정될 수도 있으므로 보관)
    //  - 그러나 실제 계산은 SKEL_Calc()에 전달된 pose, betas 우선
    Eigen::VectorXd pose_;   // (46,)
    Eigen::VectorXd betas_;  // (nbetas,)
    Eigen::Vector3d trans_;  // (3,)

    size_t n_joints_ = 24;      // 일반적으로 SKEL은 24관절
    size_t n_betas_ = 10;       // 일반적으로 betas 10개 (모델에 따라 다를 수 있음).
    std::vector<int> parent_;
    std::vector<int> child_;

    bool shape_dirty_ = true;         // betas가 변경되었는지 추적하는 플래그
    Eigen::VectorXd cached_betas_;    // 최근에 사용한 betas 값을 저장
    Eigen::MatrixXd cached_v_shaped_; // betas로부터 계산된 v_shaped (N,3)를 캐싱
    Eigen::MatrixXd cached_J_;        // betas로부터 계산된 관절 위치 J (24,3)를 캐싱
    Eigen::MatrixXd J_local_;
    std::vector<Eigen::Vector3d> t_;
};

