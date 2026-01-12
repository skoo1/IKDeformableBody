#include "SMPL.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

// 편의상 네임스페이스별로 using
using json = nlohmann::json;
using namespace Eigen;

////////////////////////////////////////////////////////////////
// SMPLModel 구현
////////////////////////////////////////////////////////////////

SMPLModel::SMPLModel(const std::string& model_path)
{
    loadModel(model_path);
    setup();

    shape_dirty_ = true;
    cached_betas_.resize(0); // 아직 비어 있음
}

void SMPLModel::loadModel(const std::string& model_path)
{
    // 여기서는 .pkl 대신 .json만 처리한다고 가정
    // 확장자 확인
    auto dot_pos = model_path.rfind('.');
    if (dot_pos == std::string::npos) {
        throw std::runtime_error("파일 확장자를 찾을 수 없습니다.");
    }
    std::string ext = model_path.substr(dot_pos);
    for (auto& c : ext) c = ::tolower(c);

    if (ext == ".json") {
        parseJson(model_path);
    }
    else {
        throw std::runtime_error("지원하지 않는 확장자(현재는 .json만 지원): " + ext);
    }
}

void SMPLModel::parseJson(const std::string& json_file)
{
    std::ifstream ifs(json_file);
    if (!ifs.is_open()) {
        throw std::runtime_error("JSON 파일을 열 수 없습니다: " + json_file);
    }
    json j;
    ifs >> j;
    ifs.close();

    // kintree_table
    if (!j.contains("kintree_table")) {
        throw std::runtime_error("JSON에 kintree_table이 없습니다.");
    }
    {
        auto kt = j["kintree_table"]; // 일반적으로 (2,24)
        int row = kt.size();
        if (row <= 0) {
            throw std::runtime_error("kintree_table의 row<=0");
        }
        int col = kt.at(0).size();
        kintree_table_.resize(row, col);
        for (int r = 0; r < row; ++r) {
            for (int c = 0; c < col; ++c) {
                kintree_table_(r, c) = kt[r][c].get<int>();
            }
        }
        n_joints_ = col; // 보통 24
    }

    // shapedirs: (N, 3, n_betas) -> (N*3, n_betas) 형태로 저장
    if (!j.contains("shapedirs")) {
        throw std::runtime_error("JSON에 shapedirs가 없습니다.");
    }
    {
        // shapedirs는 3차원 배열
        // shapedirs[i][j][k]: i-th vertex, j in [0..2], k-th beta
        auto& shapedirs_json = j["shapedirs"];
        int N = shapedirs_json.size();
        if (N == 0) {
            throw std::runtime_error("shapedirs: N=0");
        }
        int num_dim2 = shapedirs_json.at(0).size(); // 보통 3
        int num_betas = shapedirs_json.at(0).at(0).size(); // 보통 10
        // Flatten
        shapedirs_.resize(N * num_dim2, num_betas);
        for (int i = 0; i < N; i++) {
            for (int d = 0; d < num_dim2; d++) {
                for (int b = 0; b < num_betas; b++) {
                    double val = shapedirs_json[i][d][b].get<double>();
                    shapedirs_(i * num_dim2 + d, b) = val;
                }
            }
        }
        n_betas_ = num_betas;
    }

    // posedirs: (N, 3, pcount) -> (N*3, pcount)
    if (!j.contains("posedirs")) {
        throw std::runtime_error("JSON에 posedirs가 없습니다.");
    }
    {
        auto& posedirs_json = j["posedirs"];
        int N = posedirs_json.size();
        if (N == 0) {
            throw std::runtime_error("posedirs: N=0");
        }
        int num_dim2 = posedirs_json.at(0).size(); // 보통 3
        int pcount = posedirs_json.at(0).at(0).size(); // 보통 207
        posedirs_.resize(N * num_dim2, pcount);
        for (int i = 0; i < N; i++) {
            for (int d = 0; d < num_dim2; d++) {
                for (int p = 0; p < pcount; p++) {
                    double val = posedirs_json[i][d][p].get<double>();
                    posedirs_(i * num_dim2 + d, p) = val;
                }
            }
        }
    }

    // v_template: (N,3)
    if (!j.contains("v_template")) {
        throw std::runtime_error("JSON에 v_template가 없습니다.");
    }
    {
        auto& vt_json = j["v_template"];
        int N = vt_json.size();
        if (N == 0) {
            throw std::runtime_error("v_template: N=0");
        }
        int dim = vt_json.at(0).size(); // 보통 3
        v_template_.resize(N, dim);
        for (int i = 0; i < N; i++) {
            for (int d = 0; d < dim; d++) {
                double val = vt_json[i][d].get<double>();
                v_template_(i, d) = val;
            }
        }
    }

    // weights: (N, #joints=24)
    if (!j.contains("weights")) {
        throw std::runtime_error("JSON에 weights가 없습니다.");
    }
    {
        auto& w_json = j["weights"];
        int N = w_json.size();
        if (N == 0) {
            throw std::runtime_error("weights: N=0");
        }
        int col = w_json.at(0).size();
        weights_.resize(N, col);
        for (int i = 0; i < N; i++) {
            for (int c = 0; c < col; c++) {
                double val = w_json[i][c].get<double>();
                weights_(i, c) = val;
            }
        }
    }

    // J_regressor: (#joints, N)
    if (!j.contains("J_regressor")) {
        throw std::runtime_error("JSON에 J_regressor가 없습니다.");
    }
    {
        auto& jr_json = j["J_regressor"];
        int nJ = jr_json.size();      // 보통 24
        if (nJ == 0) {
            throw std::runtime_error("J_regressor: nJ=0");
        }
        int N = jr_json.at(0).size(); // 보통 6890
        J_regressor_.resize(nJ, N);
        for (int r = 0; r < nJ; r++) {
            for (int c = 0; c < N; c++) {
                double val = jr_json[r][c].get<double>();
                J_regressor_(r, c) = val;
            }
        }
    }

    // pose (기본값)
    if (j.contains("pose")) {
        auto& p_json = j["pose"];
        int length = p_json.size();
        pose_.resize(length);
        for (int i = 0; i < length; i++) {
            pose_(i) = p_json[i].get<double>();
        }
    }
    else {
        // 없으면 (24*3=72) zeros
        pose_.setZero(kintree_table_.cols() * 3);
    }

    // betas (기본값)
    if (j.contains("betas")) {
        auto& b_json = j["betas"];
        int length = b_json.size();
        betas_.resize(length);
        for (int i = 0; i < length; i++) {
            betas_(i) = b_json[i].get<double>();
        }
    }
    else {
        // shapedirs의 마지막 차원 크기에 맞게 0으로
        betas_.setZero(n_betas_);
    }

    // trans (기본값)
    if (j.contains("trans")) {
        auto& t_json = j["trans"];
        if (t_json.size() != 3) {
            throw std::runtime_error("trans의 크기는 3이어야 합니다.");
        }
        trans_ << t_json[0].get<double>(),
                  t_json[1].get<double>(),
                  t_json[2].get<double>();
    }
    else {
        trans_.setZero();
    }
}

void SMPLModel::setup()
{
    // JSON 로드 완료 후, 필요한 후처리 등의 작업이 있다면 여기서.
    // 현재는 별도 로직 없이 넘어감.
}

std::tuple<MatrixXd, MatrixXd, MatrixXd>
SMPLModel::SMPL_Calc(const Vector3d& trans, const VectorXd& pose, const VectorXd& betas)
{
    if (betas.size() != cached_betas_.size()) {
        shape_dirty_ = true;
    } else {
        // 크기가 같다면, 실제 값도 비교 (정밀도는 적당히 1e-8 ~ 1e-12 등)
        double diff = (betas - cached_betas_).norm();
        if (diff > 1e-12) {
            shape_dirty_ = true;
        }
    }


    if (shape_dirty_) {
        // 1) v_shaped = v_template + shapedirs * betas
        //    shapedirs: (N*3, n_betas)
        //    betas: (n_betas,)
        //    결과: (N*3, 1) -> reshape (N,3)
        //    v_template_: (N,3)
        const int N = static_cast<int>(v_template_.rows());
        // shapedirs * betas
        int zero_betas = n_betas_ - betas.size();
        VectorXd t_betas(n_betas_);
        t_betas.setZero();
        t_betas.head(betas.size()) = betas;

        VectorXd dV = shapedirs_ * t_betas; // (N*3, 1)
        MatrixXd v_shaped(N, 3);
        cached_v_shaped_.resize(N, 3);
        for (int i = 0; i < N; i++) {
            cached_v_shaped_(i, 0) = v_template_(i, 0) + dV(i * 3 + 0);
            cached_v_shaped_(i, 1) = v_template_(i, 1) + dV(i * 3 + 1);
            cached_v_shaped_(i, 2) = v_template_(i, 2) + dV(i * 3 + 2);
        }

        // 2) J = J_regressor * v_shaped => (#joints, N) * (N,3) = (#joints,3)
        const int nJ = kintree_table_.cols(); // 보통 24
        cached_J_.resize(nJ, 3);
        // 행렬곱 각각 (x, y, z)에 대해
        // (nJ, N) x (N,) -> (nJ,)
        // v_shaped.col(0) 등은 (N,) => 변환 필요
        {
            VectorXd vx = cached_v_shaped_.col(0);
            VectorXd vy = cached_v_shaped_.col(1);
            VectorXd vz = cached_v_shaped_.col(2);

            VectorXd Jx = J_regressor_ * vx; // (nJ,)
            VectorXd Jy = J_regressor_ * vy; // (nJ,)
            VectorXd Jz = J_regressor_ * vz; // (nJ,)

            for (int i = 0; i < nJ; i++) {
                cached_J_(i, 0) = Jx(i);
                cached_J_(i, 1) = Jy(i);
                cached_J_(i, 2) = Jz(i);
            }
        }
        cached_betas_ = betas;
        shape_dirty_ = false;
    }

    // 3) posemap
    // posedirs_: (N*3, pcount)
    // pmap: (pcount,)
    VectorXd pmap = posemap(pose);
    const int N = static_cast<int>(cached_v_shaped_.rows());
    // v_posed = v_shaped + posedirs_ * pmap -> (N*3,) -> reshape(N,3)
    VectorXd dV_pose = posedirs_ * pmap; // (N*3,)
    MatrixXd v_posed(N, 3);
    for (int i = 0; i < N; i++) {
        v_posed(i, 0) = cached_v_shaped_(i, 0) + dV_pose(i * 3 + 0);
        v_posed(i, 1) = cached_v_shaped_(i, 1) + dV_pose(i * 3 + 1);
        v_posed(i, 2) = cached_v_shaped_(i, 2) + dV_pose(i * 3 + 2);
    }

    // 4) 스키닝 -> verts, joints
    auto [verts, joints] = verts_core(pose, v_posed, cached_J_, weights_, kintree_table_);

    // 5) trans 적용
    for (int i = 0; i < verts.rows(); i++) {
        verts(i, 0) += trans(0);
        verts(i, 1) += trans(1);
        verts(i, 2) += trans(2);
    }
    for (int i = 0; i < joints.rows(); i++) {
        joints(i, 0) += trans(0);
        joints(i, 1) += trans(1);
        joints(i, 2) += trans(2);
    }

    return {verts, v_posed, joints};
}

Matrix3d SMPLModel::rodrigues(const Vector3d& rvec) const
{
    // OpenCV의 Rodrigues 와 유사한 방식의 axis-angle -> 회전행렬 변환
    double theta = rvec.norm();
    Matrix3d R = Matrix3d::Identity();
    if (theta < 1.0e-12) {
        // 각이 매우 작으면 근사적으로 Identity 반환
        return R;
    }
    Vector3d k = rvec / theta;
    // [kx, ky, kz]
    // skew-symmetric K
    Matrix3d K;
    K <<    0,   -k.z(),  k.y(),
         k.z(),     0,   -k.x(),
        -k.y(),   k.x(),     0;

    R = Matrix3d::Identity() + std::sin(theta)*K + (1.0 - std::cos(theta)) * (K * K);
    return R;
}

VectorXd SMPLModel::posemap(const VectorXd& p) const
{
    // 앞 3개는 global rotation용(혹은 global trans용)으로 빼고,
    // 나머지 관절들에 대해서 (R - I)를 벡터화하여 이어붙임
    // pose = (24*3) = (72,)
    // => 첫번째 (0~2)는 root rotation으로 보지만, 여기서는 그대로 Python 코드를 따라
    //    실제 posemap은 [3:]부터 사용
    if (p.size() < 3) {
        // 방어 코드
        return VectorXd::Zero(0);
    }
    VectorXd p_body = p.segment(3, p.size() - 3);
    int n_poses = static_cast<int>(p_body.size() / 3);

    std::vector<double> results;
    results.reserve(n_poses * 9); // (R - I) 펼친 9개씩

    for (int i = 0; i < n_poses; i++) {
        Vector3d rvec = p_body.segment(i * 3, 3);
        Matrix3d R = rodrigues(rvec);
        Matrix3d R_minus_I = R - Matrix3d::Identity();
        // 벡터화
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                results.push_back(R_minus_I(r, c));
            }
        }
    }

    // Eigen::VectorXd로 변환
    VectorXd out(results.size());
    for (size_t i = 0; i < results.size(); i++) {
        out((Eigen::Index)i) = results[i];
    }
    return out;
}

// 글로벌 관절별 4x4 변환행렬 A, 그리고 원본 A_global 반환
std::pair<std::vector<Matrix4d>, std::vector<Matrix4d>>
SMPLModel::global_rigid_transformation(const MatrixXd& pose,
                                       const MatrixXd& J,
                                       const MatrixXi& kintree_table) const
{
    // pose: (24,3), J: (24,3)
    // kintree_table: (2,24)
    // parent[i] = kintree_table(0, i)를 ID로 갖는 관절의 열 index
    // (python 코드에서 id_to_col, parent 구하는 것과 동일)
    // 여기서는 기본적으로 i번째 관절의 부모는 parent[i]
    //  - 단, i=0(root)는 parent 없음
    std::vector<int> parent_vec(kintree_table.cols(), -1);
    // id_to_col map
    std::vector<int> id_to_col(200, -1); // 단순히 200개 정도 미리 할당( SMPL은 24 관절 )
    // 두번째 행이 id, 첫번째 행이 parent id
    // kintree_table(1, i) => i번째 관절의 ID
    // kintree_table(0, i) => 해당 관절의 부모 ID
    for (int i = 0; i < kintree_table.cols(); i++) {
        int id = kintree_table(1, i);
        id_to_col[id] = i;
    }
    for (int i = 1; i < kintree_table.cols(); i++) {
        int this_id = kintree_table(1, i);
        int par_id = kintree_table(0, i);
        parent_vec[i] = id_to_col[par_id];
    }

    auto with_zeros = [&](const Matrix<double, 3, 4>& m) {
        // 아래쪽에 [0,0,0,1]을 추가 (4x4)
        Matrix4d out;
        out.setZero();
        out.block<3,4>(0,0) = m;
        out(3,3) = 1.0;
        return out;
    };

    std::vector<Matrix4d> results(kintree_table.cols());
    // root 처리
    Matrix3d R0 = rodrigues(pose.row(0).transpose());
    // J(0) => (3,)
    Vector3d j0 = J.row(0).transpose();
    Matrix<double,3,4> root_mat;
    root_mat.block<3,3>(0,0) = R0;
    root_mat.col(3) = j0;
    results[0] = with_zeros(root_mat);

    // 나머지 관절
    for (int i = 1; i < kintree_table.cols(); i++) {
        Matrix3d R = rodrigues(pose.row(i).transpose());
        Vector3d t = J.row(i).transpose() - J.row(parent_vec[i]).transpose();
        Matrix<double,3,4> tmp;
        tmp.block<3,3>(0,0) = R;
        tmp.col(3) = t;
        results[i] = results[parent_vec[i]] * with_zeros(tmp);
    }

    // 이제 A_global = results (원본)
    //  관절 위치를 0 기준으로 이동시킨 행렬(A) = Ai - pack(Ai * [Ji,0])
    auto pack = [&](const Vector4d& x) {
        // np.hstack([0행렬(4x3), x(4x1)]) 에 해당
        // => 4x4 행렬
        Matrix4d P;
        P.setZero();
        P.col(3) = x;
        return P;
    };

    std::vector<Matrix4d> results2(kintree_table.cols());
    for (int i = 0; i < kintree_table.cols(); i++) {
        // joint_homo = [J[i], 0]
        Vector4d joint_homo;
        joint_homo << J(i,0), J(i,1), J(i,2), 0.0;
        Matrix4d Ai = results[i];
        results2[i] = Ai - pack(Ai * joint_homo);
    }

    return {results2, results}; // (스키닝용, 원본)
}

std::pair<MatrixXd, MatrixXd>
SMPLModel::verts_core(const VectorXd& pose,
                      const MatrixXd& v_posed,
                      const MatrixXd& J,
                      const MatrixXd& weights,
                      const MatrixXi& kintree_table) const
{
    // A: (4x4) x 24개, A_global: (4x4) x 24개
    // A = (Ai - pack(Ai*Ji)), i=0..23
    MatrixXd pose_mat(24,3);
    // pose는 (24*3,) 일 것이므로 reshape
    for (int i = 0; i < 24; i++) {
        pose_mat.row(i) = pose.segment(i*3, 3).transpose();
    }

    auto [A, A_global] = global_rigid_transformation(pose_mat, J, kintree_table);

    // 이제 각 버텍스별로
    // verts[i] = sum_{k=0..23} weights(i,k)*A[k] * [v_posed[i], 1]
    const int N = static_cast<int>(v_posed.rows());
    MatrixXd verts(N, 3);
    for (int i = 0; i < N; i++) {
        Vector4d v_homo(v_posed(i,0), v_posed(i,1), v_posed(i,2), 1.0);
        Matrix4d M = Matrix4d::Zero();
        for (int k = 0; k < 24; k++) {
            double w = weights(i, k);
            if (std::abs(w) > 1e-12) {
                M += w * A[k];
            }
        }
        Vector4d v_out = M * v_homo;
        verts(i,0) = v_out(0);
        verts(i,1) = v_out(1);
        verts(i,2) = v_out(2);
    }

    // 글로벌 관절 위치: A_global[i](0..2,3)
    MatrixXd joints(24,3);
    for (int i = 0; i < 24; i++) {
        joints(i,0) = A_global[i](0,3);
        joints(i,1) = A_global[i](1,3);
        joints(i,2) = A_global[i](2,3);
    }

    return {verts, joints};
}
