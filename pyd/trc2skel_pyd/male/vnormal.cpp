#include "vnormal.h"
#include <cmath>  // for std::sqrt

//-------------------------------------------------
// (추가) 두 벡터 사이 각도(라디안) 계산 보조 함수
//-------------------------------------------------
static inline float angleBetween(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
    float denom = a.norm() * b.norm();
    if (denom < 1e-12f) {
        return 0.0f;
    }
    float c = a.dot(b) / denom;
    // -1 <= c <= 1 범위로 clamp
    if (c > 1.0f)  c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c);
}

void MeshNormalCalculator::setVertex(const Eigen::MatrixXf& vertices)
{
    m_vertices = vertices;
}

void MeshNormalCalculator::setFace(const Eigen::MatrixXi& faces)
{
    m_faces = faces;
}

// -------------------------------------------------------------
// 1) getVertexNormal() - "각도 가중치"로 수정
// -------------------------------------------------------------
Eigen::MatrixXf MeshNormalCalculator::getVertexNormal() const
{
    // 예외 처리
    if (m_vertices.rows() == 0 || m_faces.rows() == 0) {
        throw std::runtime_error("Vertices or faces not set!");
    }

    const int n = static_cast<int>(m_vertices.rows());

    // 정점 노멀 누적용
    Eigen::MatrixXf normals = Eigen::MatrixXf::Zero(n, 3);

    // 각 face를 순회
    for (int i = 0; i < m_faces.rows(); ++i) {
        int idx0 = m_faces(i, 0);
        int idx1 = m_faces(i, 1);
        int idx2 = m_faces(i, 2);

        // 버텍스 좌표
        Eigen::Vector3f v0 = m_vertices.row(idx0);
        Eigen::Vector3f v1 = m_vertices.row(idx1);
        Eigen::Vector3f v2 = m_vertices.row(idx2);

        // face normal (기존처럼 교차곱 계산)
        Eigen::Vector3f edge1 = v1 - v0;
        Eigen::Vector3f edge2 = v2 - v0;
        Eigen::Vector3f faceNormal = edge1.cross(edge2);

        // 면적이 너무 작으면(=degenerate face) 건너뜀
        float length = faceNormal.norm();
        if (length < 1e-12f) {
            continue;
        }

        // (기존) faceNormal 정규화
        faceNormal /= length;

        // (추가) 각 정점에서의 내각 계산
        float alpha0 = angleBetween(v1 - v0, v2 - v0);
        float alpha1 = angleBetween(v0 - v1, v2 - v1);
        float alpha2 = angleBetween(v0 - v2, v1 - v2);

        // (수정) 각도만큼 곱해서 누적
        normals.row(idx0) += alpha0 * faceNormal;
        normals.row(idx1) += alpha1 * faceNormal;
        normals.row(idx2) += alpha2 * faceNormal;
    }

    // 마지막에 각 정점 노멀 정규화
    for (int i = 0; i < n; ++i) {
        float length = normals.row(i).norm();
        if (length > 1e-12f) {
            normals.row(i) /= length;
        }
    }

    return normals;
}

// -------------------------------------------------------------
// (도움 함수) row마다 3D 벡터 외적 (기존 그대로)
// -------------------------------------------------------------
static Eigen::MatrixXf crossRowwise(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B)
{
    int M = (int)A.rows();
    Eigen::MatrixXf C(M, 3);
    for (int i = 0; i < M; ++i) {
        float ax = A(i,0), ay = A(i,1), az = A(i,2);
        float bx = B(i,0), by = B(i,1), bz = B(i,2);
        C(i,0) = ay * bz - az * by; // x
        C(i,1) = az * bx - ax * bz; // y
        C(i,2) = ax * by - ay * bx; // z
    }
    return C;
}

// -------------------------------------------------------------
// (도움 함수) 각 row를 정규화 (기존 그대로)
// -------------------------------------------------------------
static void normalizeRowwise(Eigen::MatrixXf &mat, float eps=1e-12f)
{
    int M = (int)mat.rows();
    for (int i = 0; i < M; ++i) {
        float length = mat.row(i).norm();
        if (length > eps) {
            mat.row(i) /= length;
        }
    }
}

// -------------------------------------------------------------
// 2) getVertexNormal2() - Sparse Matrix 방식에서
//    1.0 대신 "각도"로 치환
// -------------------------------------------------------------
Eigen::MatrixXf MeshNormalCalculator::getVertexNormal2() const
{
    if (m_vertices.rows() == 0 || m_faces.rows() == 0) {
        throw std::runtime_error("Vertices or faces not set!");
    }

    int N = (int)m_vertices.rows();  // 정점 수
    int M = (int)m_faces.rows();     // face 수

    // --------------------------------------
    // 1) 각 face (i0, i1, i2)의 v0, v1, v2를 (M x 3)에 담기 (기존 그대로)
    // --------------------------------------
    Eigen::MatrixXf v0(M,3), v1(M,3), v2(M,3);
    for(int j = 0; j < M; ++j) {
        int i0 = m_faces(j, 0);
        int i1 = m_faces(j, 1);
        int i2 = m_faces(j, 2);

        v0.row(j) = m_vertices.row(i0);
        v1.row(j) = m_vertices.row(i1);
        v2.row(j) = m_vertices.row(i2);
    }

    // face normal 계산 후 정규화 (기존)
    Eigen::MatrixXf edge1 = v1 - v0;
    Eigen::MatrixXf edge2 = v2 - v0;
    Eigen::MatrixXf faceNormals = crossRowwise(edge1, edge2);
    normalizeRowwise(faceNormals);

    // --------------------------------------
    // (추가) 각도(내각) 계산
    // --------------------------------------
    // -> M개 face 각각에 대해 alpha0, alpha1, alpha2
    // -> 희소행렬에 1.0 대신 alpha를 넣을 예정
    std::vector<float> alpha0(M), alpha1(M), alpha2(M);
    for(int j = 0; j < M; ++j) {
        Eigen::Vector3f vv0 = v0.row(j);
        Eigen::Vector3f vv1 = v1.row(j);
        Eigen::Vector3f vv2 = v2.row(j);

        alpha0[j] = angleBetween(vv1 - vv0, vv2 - vv0);
        alpha1[j] = angleBetween(vv0 - vv1, vv2 - vv1);
        alpha2[j] = angleBetween(vv0 - vv2, vv1 - vv2);
    }

    // --------------------------------------
    // 2) 희소행렬 A (N x M)에 "내각(alpha)"를 넣음
    // --------------------------------------
    typedef Eigen::Triplet<float> T;
    std::vector<T> tripletList;
    tripletList.reserve(M * 3);

    for(int j = 0; j < M; ++j) {
        int i0 = m_faces(j, 0);
        int i1 = m_faces(j, 1);
        int i2 = m_faces(j, 2);

        tripletList.push_back(T(i0, j, alpha0[j]));
        tripletList.push_back(T(i1, j, alpha1[j]));
        tripletList.push_back(T(i2, j, alpha2[j]));
    }

    Eigen::SparseMatrix<float> A(N, M);
    A.setFromTriplets(tripletList.begin(), tripletList.end());
    A.makeCompressed();

    // --------------------------------------
    // 3) vertexNormals = A * faceNormals  ->  (N x 3)
    // --------------------------------------
    Eigen::MatrixXf vertexNormals = A * faceNormals;

    // 마지막 rowwise 정규화
    normalizeRowwise(vertexNormals);

    return vertexNormals;
}
