#include "skel.h"
#include "osim_joints.h"
#include "joints_def.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

// 편의상 네임스페이스별로 using
using json = nlohmann::json;
using namespace Eigen;

////////////////////////////////////////////////////////////////
// SKELModel 구현
////////////////////////////////////////////////////////////////

SKELModel::SKELModel(const std::string& model_path)
{
    loadModel(model_path);
    setup();

    shape_dirty_ = true;
    cached_betas_.resize(0); // 아직 비어 있음

    joints_dict_ = {
        new CustomJoint({{0,0,1}, {1,0,0}, {0,1,0}}, {1, 1, 1}),                // 0 pelvis
        new CustomJoint({{0,0,1}, {1,0,0}, {0,1,0}}, {1, 1, 1}),                // 1 femur_r
        new WalkerKnee(),                                                       // 2 tibia_r
        new PinJoint(Vector3d(0.175895, -0.105208, 0.0186622)),                 // 3 talus_r
        new PinJoint(Vector3d(-1.76818999, 0.906223, 1.8196000)),               // 4 calcn_r
        new PinJoint(Vector3d(-3.141589999, 0.6199010, 0)),                     // 5 toes_r
        new CustomJoint({{0,0,1}, {1,0,0}, {0,1,0}}, {1, -1, -1}),              // 6 femur_l
        new WalkerKnee(),                                                       // 7 tibia_l
        new PinJoint(Vector3d(0.175895, -0.105208, 0.0186622)),                 // 8 talus_l
        new PinJoint(Vector3d(1.768189999, -0.906223, 1.8196000)),              // 9 calcn_l
        new PinJoint(Vector3d(-3.141589999, -0.6199010, 0)),                    // 10 toes_l
        new ConstantCurvatureJoint({{1,0,0}, {0,0,1}, {0,1,0}}, {1, 1, 1}),     // 11 lumbar
        new ConstantCurvatureJoint({{1,0,0}, {0,0,1}, {0,1,0}}, {1, 1, 1}),     // 12 thorax
        new ConstantCurvatureJoint({{1,0,0}, {0,0,1}, {0,1,0}}, {1, 1, 1}),     // 13 head
        new EllipsoidJoint({{0,1,0}, {0,0,1}, {1,0,0}}, {1, -1, -1}),           // 14 scapula_r
        new CustomJoint({{1,0,0}, {0,1,0}, {0,0,1}}, {1, 1, 1}),                // 15 humerus_r
        new CustomJoint({{0.0494, 0.0366, 0.99810825}}, {1}),                   // 16 ulna_r
        new CustomJoint({{-0.01716099, 0.99266564, -0.11966796}}, {1}),         // 17 radius_r
        new CustomJoint({{1,0,0}, {0,0,-1}}, {1, 1}),                           // 18 hand_r
        new EllipsoidJoint({{0,1,0}, {0,0,1}, {1,0,0}}, {1, 1, 1}),             // 19 scapula_l
        new CustomJoint({{1,0,0}, {0,1,0}, {0,0,1}}, {1, 1, 1}),                // 20 humerus_l
        new CustomJoint({{-0.0494, -0.0366, 0.99810825}}, {1}),                 // 21 ulna_l
        new CustomJoint({{0.01716099, -0.99266564, -0.11966796}}, {1}),         // 22 radius_l
        new CustomJoint({{-1,0,0}, {0,0,-1}}, {1, 1})                           // 23 hand_l
    };
}

void SKELModel::loadModel(const std::string& model_path)
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

void SKELModel::parseJson(const std::string& json_file)
{
    std::ifstream ifs(json_file);
    if (!ifs.is_open()) {
        throw std::runtime_error("JSON 파일을 열 수 없습니다: " + json_file);
    }
    json j;
    ifs >> j;
    ifs.close();

    // osim_kintree_table
    if (!j.contains("osim_kintree_table")) {
        throw std::runtime_error("JSON에 osim_kintree_table이 없습니다.");
    }
    {
        auto kt = j["osim_kintree_table"]; // 일반적으로 (2,24)
        size_t row = kt.size();
        if (row <= 0) {
            throw std::runtime_error("osim_kintree_table의 row<=0");
        }
        size_t col = kt.at(0).size();
        osim_kintree_table_.resize(row, col);
        for (size_t r = 0; r < row; ++r) {
            for (size_t c = 0; c < col; ++c) {
                osim_kintree_table_(r, c) = kt[r][c].get<int>();
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
        size_t N = shapedirs_json.size();
        if (N == 0) {
            throw std::runtime_error("shapedirs: N=0");
        }
        size_t num_dim2 = shapedirs_json.at(0).size(); // 보통 3
        size_t num_betas = shapedirs_json.at(0).at(0).size(); // 보통 10
        // Flatten
        shapedirs_.resize(N * num_dim2, num_betas);
        for (size_t i = 0; i < N; i++) {
            for (size_t d = 0; d < num_dim2; d++) {
                for (size_t b = 0; b < num_betas; b++) {
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
        size_t N = posedirs_json.size();
        if (N == 0) {
            throw std::runtime_error("posedirs: N=0");
        }
        size_t num_dim2 = posedirs_json.at(0).size(); // 보통 3
        size_t pcount = posedirs_json.at(0).at(0).size(); // 보통 207
        posedirs_.resize(N * num_dim2, pcount);
        for (size_t i = 0; i < N; i++) {
            for (size_t d = 0; d < num_dim2; d++) {
                for (size_t p = 0; p < pcount; p++) {
                    double val = posedirs_json[i][d][p].get<double>();
                    posedirs_(i * num_dim2 + d, p) = val;
                }
            }
        }
    }

    // skin_template_v: (N,3)
    if (!j.contains("skin_template_v")) {
        throw std::runtime_error("JSON에 skin_template_v 없습니다.");
    }
    {
        auto& vt_json = j["skin_template_v"];
        size_t N = vt_json.size();
        if (N == 0) {
            throw std::runtime_error("skin_template_v: N=0");
        }
        size_t dim = vt_json.at(0).size(); // 보통 3
        skin_template_v_.resize(N, dim);
        for (size_t i = 0; i < N; i++) {
            for (size_t d = 0; d < dim; d++) {
                double val = vt_json[i][d].get<double>();
                skin_template_v_(i, d) = val;
            }
        }
    }

    // skin_weights: (N, #joints=24)
    if (!j.contains("skin_weights")) {
        throw std::runtime_error("JSON에 skin_weights가 없습니다.");
    }
    {
        auto& w_json = j["skin_weights"];
        size_t N = w_json.size();
        if (N == 0) {
            throw std::runtime_error("skin_weights: N=0");
        }
        size_t col = w_json.at(0).size();
        skin_weights_.resize(N, col);
        for (size_t i = 0; i < N; i++) {
            for (size_t c = 0; c < col; c++) {
                double val = w_json[i][c].get<double>();
                skin_weights_(i, c) = val;
            }
        }
    }

    // J_regressor: (#joints, N)
    if (!j.contains("J_regressor")) {
        throw std::runtime_error("JSON에 J_regressor가 없습니다.");
    }
    {
        auto& jr_json = j["J_regressor"];
        size_t nJ = jr_json.size();      // 보통 24
        if (nJ == 0) {
            throw std::runtime_error("J_regressor: nJ=0");
        }
        size_t N = jr_json.at(0).size(); // 보통 6890
        J_regressor_.resize(nJ, N);
        for (size_t r = 0; r < nJ; r++) {
            for (size_t c = 0; c < N; c++) {
                double val = jr_json[r][c].get<double>();
                J_regressor_(r, c) = val;
            }
        }
    }

    // J_regressor-osim: (#joints, N)
    if (!j.contains("J_regressor_osim")) {
        throw std::runtime_error("JSON에 J_regressor_osim이 없습니다.");
    }
    {
        auto& jr_json = j["J_regressor_osim"];
        size_t nJ = jr_json.size();      // 보통 24
        if (nJ == 0) {
            throw std::runtime_error("J_regressor_osim: nJ=0");
        }
        size_t N = jr_json.at(0).size(); // 보통 6890
        J_regressor_osim_.resize(nJ, N);
        for (size_t r = 0; r < nJ; r++) {
            for (size_t c = 0; c < N; c++) {
                double val = jr_json[r][c].get<double>();
                J_regressor_osim_(r, c) = val;
            }
        }
    }

    // apose_rel_transfo
    if (!j.contains("apose_rel_transfo")) {
        throw std::runtime_error("JSON에 apose_rel_transfo가 없습니다.");
    }
    {
        auto& apose_rt = j["apose_rel_transfo"];
        size_t nJ = apose_rt.size();
        if (apose_rt.size() != 24) {
            throw std::runtime_error("apose_rel_transfo 크기가 24가 아닙니다.");
        }
        apose_rel_transfo_.resize(nJ);
        for (int i = 0; i < 24; ++i) {
            if (apose_rt[i].size() != 4 || apose_rt[i][0].size() != 4) {
                throw std::runtime_error("apose_rel_transfo의 각 원소가 4x4 행렬이 아닙니다.");
            }
    
            Matrix4d mat;
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    mat(r, c) = apose_rt[i][r][c].get<double>();
                }
            }
            apose_rel_transfo_[i] = mat;
        }
    }

    // per_joint_rot
    if (!j.contains("per_joint_rot")) {
        throw std::runtime_error("JSON에 per_joint_rot이 없습니다.");
    }
    {
        auto& pjr_rt = j["per_joint_rot"];
        size_t nJ = pjr_rt.size();
        if (pjr_rt.size() != 24) {
            throw std::runtime_error("per_joint_rot 크기가 24가 아닙니다.");
        }
        per_joint_rot_.resize(nJ);
        for (int i = 0; i < 24; ++i) {
            if (pjr_rt[i].size() != 3 || pjr_rt[i][0].size() != 3) {
                throw std::runtime_error("per_joint_rot의 각 원소가 3x3 행렬이 아닙니다.");
            }
    
            Matrix3d mat;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    mat(r, c) = pjr_rt[i][r][c].get<double>();
                }
            }
            per_joint_rot_[i] = mat;
        }
    }

    // pose (기본값)
    if (j.contains("pose")) {
        auto& p_json = j["pose"];
        size_t length = p_json.size();
        pose_.resize(length);
        for (size_t i = 0; i < length; i++) {
            pose_(i) = p_json[i].get<double>();
        }
    }
    else {
        // 없으면 (46 = # of pose parameters) zeros
        pose_.setZero(46);
    }

    // betas (기본값)
    if (j.contains("betas")) {
        auto& b_json = j["betas"];
        size_t length = b_json.size();
        betas_.resize(length);
        for (size_t i = 0; i < length; i++) {
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

void SKELModel::setup()
{
    // JSON 로드 완료 후, 필요한 후처리 등의 작업이 있다면 여기서.
    // 현재는 별도 로직 없이 넘어감.
}

MatrixXd SKELModel::SKEL_Mesh_Calc(const VectorXd& pose, const VectorXd& betas, const Vector3d& trans)
{
    size_t num_joints = J_regressor_osim_.rows();
    size_t num_joints_smpl = J_regressor_.rows();
    size_t Ns = skin_template_v_.rows();
    size_t Nj = num_joints;
    
    parent_.resize(Nj - 1);
    for (size_t it = 1; it < Nj; ++it) {
        parent_[it - 1] = osim_kintree_table_(0, it);
    }

    child_.resize(Nj);
    for (size_t i = 0; i < Nj; ++i) {
        bool found = false;
        // osim_kintree_table_의 첫 번째 행에서 현재 joint(i)를 찾아서, 
        // 대응하는 두 번째 행 값을 child_로 설정
        for (int j = 0; j < osim_kintree_table_.cols(); ++j) {
            if (osim_kintree_table_(0, j) == i) {
                child_[i] = osim_kintree_table_(1, j);
                found = true;
                break;  // 찾으면 바로 종료
            }
        }
        // 자식이 없으면 기본값(0)이 유지됨
        if (!found) {
            child_[i] = 0;
        }
    }
    
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
        // 1) Shape
        // v_shaped = skin_template_v + shapedirs * betas
        //    shapedirs: (N*3, n_betas)
        //    betas: (n_betas,)
        //    결과: (N*3, 1) -> reshape (N,3)
        //    skin_template_v_: (N,3)
        const int N = static_cast<int>(skin_template_v_.rows());
        // shapedirs * betas
        VectorXd t_betas(n_betas_);
        t_betas.setZero();
        t_betas.head(betas.size()) = betas;

        VectorXd dV = shapedirs_ * t_betas; // (N*3, 1)
        MatrixXd v_shaped(N, 3);
        cached_v_shaped_.resize(N, 3);
        for (int i = 0; i < N; i++) {
            cached_v_shaped_(i, 0) = skin_template_v_(i, 0) + dV(i * 3 + 0);
            cached_v_shaped_(i, 1) = skin_template_v_(i, 1) + dV(i * 3 + 1);
            cached_v_shaped_(i, 2) = skin_template_v_(i, 2) + dV(i * 3 + 2);
        }

        // 2) Joints 
        // J = J_regressor * v_shaped => (#joints, N) * (N,3) = (#joints,3)
        const auto nJ = osim_kintree_table_.cols(); // 보통 24
        cached_J_.resize(nJ, 3);
        // 행렬곱 각각 (x, y, z)에 대해
        // (nJ, N) x (N,) -> (nJ,)
        // v_shaped.col(0) 등은 (N,) => 변환 필요
        {
            VectorXd vx = cached_v_shaped_.col(0);
            VectorXd vy = cached_v_shaped_.col(1);
            VectorXd vz = cached_v_shaped_.col(2);

            VectorXd Jx = J_regressor_osim_ * vx; // (nJ,)
            VectorXd Jy = J_regressor_osim_ * vy; // (nJ,)
            VectorXd Jz = J_regressor_osim_ * vz; // (nJ,)

            for (int i = 0; i < nJ; i++) {
                cached_J_(i, 0) = Jx(i);
                cached_J_(i, 1) = Jy(i);
                cached_J_(i, 2) = Jz(i);
            }
        }
        cached_betas_ = betas;
        shape_dirty_ = false;
        // Local translation
        J_local_.resize(nJ, 3);
        J_local_.row(0) = cached_J_.row(0);
        for (int i = 1; i < nJ; ++i) {
            int parent_idx = osim_kintree_table_(0, i);  // 🔁 i-1 → i 로 변경
            J_local_.row(i) = cached_J_.row(i) - cached_J_.row(parent_idx);
        }
        t_.resize(nJ);
        for (int i = 0; i < nJ; ++i) {
            t_[i] = J_local_.row(i).transpose();
        }
    }

    // 3) Joint 위치 수정
    // Rp : (3, 3)
    // 여기서 tp는 사용 안 함
    // Bone initial transform to go from unposed to SMPL T pose
    auto Rk01 = compute_bone_orientation(cached_J_, J_local_);
    
    // BSM default pose rotations
    std::vector<Matrix3d> Ra(24);
    for (size_t j = 0; j < 24; ++j) {
        Ra[j] = apose_rel_transfo_[j].block<3,3>(0,0);
    }


    // Local bone rotation given by the pose param
    auto [Rp, tp] = pose_params_to_rot(pose, joints_dict_);
    std::vector<Matrix3d> R(Nj);
    for (size_t j = 0; j < Nj; ++j) {
        R[j] = Rk01[j] * Ra[j].transpose() * Rp[j] * Ra[j] * Rk01[j].transpose();
    }

    // Compute translation for non pure rotation joints  ::: J -> cached_J_
    auto t_posed = t_;

    // scapula
    double thorax_width = (cached_J_.row(19) - cached_J_.row(14)).norm();
    double thorax_height = (cached_J_.row(12) - cached_J_.row(11)).norm();

    double angle_abduction = pose(26);
    double angle_elevation = pose(27);
    double angle_zero = 0.0;

    Vector3d delta_scapula_r = right_scapula(angle_abduction, angle_elevation, thorax_width, thorax_height)
                   - right_scapula(angle_zero, angle_zero, thorax_width, thorax_height);
    t_posed[14] += delta_scapula_r;

    angle_abduction = pose(36);
    angle_elevation = pose(37);
    angle_zero = 0.0;

    Vector3d delta_scapula_l = left_scapula(angle_abduction, angle_elevation, thorax_width, thorax_height)
                       - left_scapula(angle_zero, angle_zero, thorax_width, thorax_height);
    t_posed[19] += delta_scapula_l;

    // spine
    double lumbar_bending = pose(17);
    double lumbar_extension = pose(18);
    angle_zero = 0.0;
    double interp_t = 1.0;
    double len_lumbar = std::abs(cached_J_(11, 1) - cached_J_(0, 1)); // y축 spine 길이

    Vector3d delta_lumbar = curve_3d(lumbar_bending, lumbar_extension, interp_t, len_lumbar)
                        - curve_3d(angle_zero, angle_zero, interp_t, len_lumbar);
    t_posed[11] += delta_lumbar;

    double thorax_bending = pose(20);
    double thorax_extension = pose(21);
    angle_zero = 0.0;
    interp_t = 1.0;
    double len_thorax = std::abs(cached_J_(12, 1) - cached_J_(11, 1));

    Vector3d delta_thorax = curve_3d(thorax_bending, thorax_extension, interp_t, len_thorax)
                        - curve_3d(angle_zero, angle_zero, interp_t, len_thorax);
    t_posed[12] += delta_thorax;

    double head_bending = pose(23);
    double head_extension = pose(24);
    angle_zero = 0.0;
    interp_t = 1.0;
    double len_head = std::abs(cached_J_(13, 1) - cached_J_(12, 1));

    Vector3d delta_head = curve_3d(head_bending, head_extension, interp_t, len_head)
                        - curve_3d(angle_zero, angle_zero, interp_t, len_head);
    t_posed[13] += delta_head;

    // Body surface transformation matrix
    // G_: local transformation matrices (4x4)
    std::vector<Matrix4d> G_(Nj);
    for (size_t j = 0; j < Nj; ++j) {
        Matrix4d mat = Matrix4d::Identity();
        mat.block<3,3>(0,0) = R[j];
        mat.block<3,1>(0,3) = t_posed[j]; // ✅ std::vector<Eigen::Vector3d>는 이렇게 접근
        G_[j] = mat;
    }
    
    // Global transformations (G)
    std::vector<Matrix4d> G(Nj);
    G[0] = G_[0];
    for (size_t j = 1; j < Nj; ++j) {
        int parent_idx = parent_[j - 1];
        G[j] = G[parent_idx] * G_[j];
    }


    // 4) Pose dependent blend shapes
    // posedirs_: (N*3, pcount)

    // Rskin: Extracted local rotations from G_
    std::vector<Matrix3d> Rskin(Nj);
    for (size_t j = 0; j < Nj; ++j) {
        Rskin[j] = G_[j].block<3,3>(0,0);
    }
    // smpl_joint_cooresp form kin_skel.py
    const std::vector<int> smpl_joint_corresp = {
        0, 2, 5, 8, 8, 11, 1, 4, 7, 7, 10, 3, 6, 15, 14, 17, 19, 0, 21, 13, 16, 18, 0, 20
    };

    
    // Initialize Rsmpl with identity matrices
    std::vector<Matrix3d> Rsmpl(num_joints_smpl, Matrix3d::Identity());

    // Map Rskin to Rsmpl using smpl_joint_corresp
    for (size_t idx = 0; idx < smpl_joint_corresp.size(); ++idx) {
        Rsmpl[smpl_joint_corresp[idx]] = Rskin[idx];
    }

    // Compute pose_feature and pose_offsets
    MatrixXd pose_feature(1, 207);
    for (int j = 1; j < 24; ++j) {
        Matrix3d diff = Rsmpl[j] - Matrix3d::Identity();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> diff_row = diff;
        Map<VectorXd> flat(diff_row.data(), 9);
        pose_feature.block(0, (j - 1) * 9, 1, 9) = flat.transpose();
    }
       
    MatrixXd pose_offsets_flat = pose_feature * posedirs_.transpose();  // (1 x Ns*3)

    // ✅ PyTorch-style reshape: row-major view
    Eigen::Map<Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> pose_offsets(pose_offsets_flat.data(), Ns, 3);
    MatrixXd v_shaped_pd = cached_v_shaped_ + pose_offsets;


    // 5) 스키닝 -> Gskins
    // Compute Gskin by applying global transformations and removing rest translations
    std::vector<Matrix4d> Gskin(Nj);
    for (size_t j = 0; j < Nj; ++j) {
        Gskin[j] = G[j];
    
        // Translation 보정: G @ rest.translation
        Vector3d rest_t = cached_J_.row(j).transpose();             // python에서 rest  (skel_model.py line 466, 468)
        Vector3d rest = G[j].block<3,3>(0,0) * rest_t;              // rest = G - rest (skel_model.py line 469)
    
        Gskin[j].block<3,1>(0,3) -= rest;                           // Gskin = G - rest (skel_model.py line 470)
    }

    // Compute vertex transformations T using skin weights
    std::vector<Matrix4d> T(Ns, Matrix4d::Zero());
    for (size_t v = 0; v < Ns; ++v) {
        for (size_t j = 0; j < Nj; ++j) {
            T[v] += skin_weights_(v, j) * Gskin[j];
        }
    }

    // Apply transformations to v_shaped_pd
    MatrixXd v_posed(Ns, 3);
    for (size_t v = 0; v < Ns; ++v) {
        Vector4d rest_shape_h;
        rest_shape_h << v_shaped_pd.row(v).transpose(), 1.0;  // 단일 프레임이므로 b 삭제
        Vector4d transformed = T[v] * rest_shape_h;
        v_posed.row(v) = transformed.head<3>().transpose();
    }

    // 6) trans 적용
    // Apply global translation
    MatrixXd verts(Ns, 3);
    for (size_t v = 0; v < Ns; ++v) {
        verts.row(v) = v_posed.row(v) + trans.transpose(); // trans는 (3,) 이므로 transpose하여 (1,3) 형태로 맞춰줌
    }

    return verts;
}

std::pair<std::vector<Matrix3d>, std::vector<Vector3d>>
SKELModel::pose_params_to_rot(
    const VectorXd& osim_poses,
    const std::vector<OsimJoint*>& joints_dict
) const
{
    const auto nJ = cached_J_.rows();
    std::vector<Matrix3d> Rp;
    std::vector<Vector3d> tp;

    Rp.resize(nJ);
    tp.resize(nJ, Vector3d::Zero());

    size_t start_index = 0;
    for (int i = 0; i < nJ; ++i) {
        OsimJoint* joint_object = joints_dict[i];
        size_t nb_dof = joint_object->get_dof();

        VectorXd q = osim_poses.segment(start_index, nb_dof);
        Rp[i] = joint_object->q_to_rot(q);
        
        start_index += nb_dof;
    }

    return {Rp, tp};
}

// Function to compute rotation matrix aligning vec1 to vec2
Matrix3d SKELModel::rotation_matrix_from_vectors(
    const Vector3d& vec1,
    const Vector3d& vec2)
{
    Vector3d a = vec1.normalized();
    Vector3d b = vec2.normalized();

    Vector3d v = a.cross(b);
    double c = a.dot(b);
    double s = v.norm() + std::numeric_limits<double>::epsilon();

    Matrix3d kmat;
    kmat <<      0,   -v(2),  v(1),
              v(2),       0, -v(0),
             -v(1),    v(0),     0;

    Matrix3d rotation_matrix = Matrix3d::Identity()
                                    + kmat
                                    + (kmat * kmat) * ((1 - c) / (s * s));

    return rotation_matrix;
}

// Function to compute bone orientation using J
std::vector<Matrix3d>
SKELModel::compute_bone_orientation(
    const MatrixXd& J,             // (nJ, 3) global joint 위치
    const MatrixXd& J_local       // (nJ, 3) local joint 위치
) {
    const auto nJ = J.rows();
    std::vector<Matrix3d> Gk(nJ, Matrix3d::Identity());

    // bone_vect 계산
    std::vector<Vector3d> bone_vect(nJ, Vector3d::Zero());
    for(int i = 0; i < nJ; ++i){
        bone_vect[i] = J_local.row(child_[i]);
    }

    // 특정 관절의 벡터 합 조정 (ulna, thorax 등)
    bone_vect[16] += bone_vect[17]; // 오른쪽 ulna
    bone_vect[21] += bone_vect[22]; // 왼쪽 ulna
    bone_vect[12] = bone_vect[11];  // thorax

    // osim_vect 계산 (osim 기준 벡터에서 자식 인덱스로 초기화)
    std::vector<Vector3d> osim_vect(nJ);
    for(int i = 0; i < nJ; ++i){
        osim_vect[i] = apose_rel_transfo_[child_[i]].block<3,1>(0,3);
    }

    osim_vect[16] += osim_vect[17];
    osim_vect[21] += osim_vect[22];

    // "learn_adjust" 방법 적용
    std::vector<Vector3d> osim_vect_corr(nJ);
    for(int i = 0; i < nJ; ++i){
        osim_vect_corr[i] = per_joint_rot_[i] * osim_vect[i];
    }

    for(int i = 0; i < nJ; ++i){
        Gk[i] = rotation_matrix_from_vectors(osim_vect_corr[i], bone_vect[i]);
        
        // NaN 체크 후 identity로 복구
        if(!Gk[i].allFinite()){
            Gk[i] = Matrix3d::Identity();
        }
    }

    // beta에 따라 고정되는 관절 orientation 강제 설정
    std::vector<int> joint_idx_fixed_beta = {0, 5, 10, 13, 18, 23};
    for(int idx : joint_idx_fixed_beta){
        Gk[idx] = Matrix3d::Identity();
    }

    // 최종 Gk에 학습된 rotation을 곱함
    for(int i = 0; i < nJ; ++i){
        Gk[i] = Gk[i] * per_joint_rot_[i];
    }

    return Gk;
}

Matrix3d SKELModel::matmul_chain(const std::vector<Matrix3d>& rot_list) {
    Matrix3d R_tot = rot_list.back();
    for (int i = static_cast<int>(rot_list.size()) - 2; i >= 0; --i) {
        R_tot = rot_list[i] * R_tot;
    }
    return R_tot;
}
