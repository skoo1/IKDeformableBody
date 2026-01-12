#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

// SMPLModel 클래스
class SMPLModel
{
public:
    // 생성자: JSON 경로로부터 모델 로드
    SMPLModel(const std::string& model_path);

    // 소멸자
    ~SMPLModel() = default;

    // pose, betas를 주면 최종 버텍스와 관절 위치 등을 계산
    // 반환: (verts, v_posed, joints)
    //  - verts: 스키닝 적용 후 최종 버텍스 좌표 (N,3)
    //  - v_posed: pose에 의한 posed 버텍스 (N,3)
    //  - joints: 글로벌 관절 위치 (24,3) 가정
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    SMPL_Calc(const Eigen::Vector3d& trans, const Eigen::VectorXd& pose, const Eigen::VectorXd& betas);

private:
    // 모델 파싱 및 멤버 변수 세팅
    void loadModel(const std::string& model_path);
    void setup();

    // 로드된 JSON 데이터를 파싱해서 Eigen 형식 멤버에 대입
    void parseJson(const std::string& json_file);

    // rodrigues: axis-angle 벡터(3,) -> 회전행렬(3,3)
    Eigen::Matrix3d rodrigues(const Eigen::Vector3d& rvec) const;

    // posemap: pose 벡터(24*3=72 차원)를 받아,
    //         각 관절 회전행렬에서 (R - I)를 벡터화한 것들을 이어붙임
    Eigen::VectorXd posemap(const Eigen::VectorXd& p) const;

    // 관절별 글로벌 4x4 변환행렬 계산
    // 반환: (결과적으로 스키닝에 사용하는 24개의 4x4 행렬(A), 그리고 24개의 원본 4x4 행렬)
    //       A[i]는 (Ai - pack(Ai*joint_i)) 형태(= "rest pose" 해제된 행렬).
    std::pair<std::vector<Eigen::Matrix4d>, std::vector<Eigen::Matrix4d>>
    global_rigid_transformation(const Eigen::MatrixXd& pose,
                                const Eigen::MatrixXd& J,
                                const Eigen::MatrixXi& kintree_table) const;

    // 스키닝 적용: v_posed, J, weights -> 최종 verts, 글로벌 관절
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
    verts_core(const Eigen::VectorXd& pose,
               const Eigen::MatrixXd& v_posed,
               const Eigen::MatrixXd& J,
               const Eigen::MatrixXd& weights,
               const Eigen::MatrixXi& kintree_table) const;

private:
    // SMPL 관련 멤버들
    Eigen::MatrixXd v_template_;    // (N,3)
    Eigen::MatrixXd shapedirs_;     // (N*3, n_betas)
    Eigen::MatrixXd posedirs_;      // (N*3, n_posemap)
    Eigen::MatrixXd J_regressor_;   // (#joints, N)
    Eigen::MatrixXd weights_;       // (N, #joints)
    Eigen::MatrixXi kintree_table_; // (2, #joints=24)

    // pose, betas, trans의 기본값 (모델 로드 시점에 설정될 수도 있으므로 보관)
    //  - 그러나 실제 계산은 SMPL_Calc()에 전달된 pose, betas 우선
    Eigen::VectorXd pose_;   // (72,)
    Eigen::VectorXd betas_;  // (nbetas,)
    Eigen::Vector3d trans_;  // (3,)

    int n_joints_ = 24;      // 일반적으로 SMPL은 24관절
    int n_betas_ = 10;       // 일반적으로 betas 10개 (모델에 따라 다를 수 있음).

    bool shape_dirty_ = true;         // betas가 변경되었는지 추적하는 플래그
    Eigen::VectorXd cached_betas_;    // 최근에 사용한 betas 값을 저장
    Eigen::MatrixXd cached_v_shaped_; // betas로부터 계산된 v_shaped (N,3)를 캐싱
    Eigen::MatrixXd cached_J_;        // betas로부터 계산된 관절 위치 J (24,3)를 캐싱
};

