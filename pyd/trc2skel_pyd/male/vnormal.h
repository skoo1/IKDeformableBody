#ifndef VNORMAL_H
#define VNORMAL_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

class MeshNormalCalculator
{
public:
    MeshNormalCalculator() = default;

    // vertices, faces를 통째로 받고 내부에 저장
    void setVertex(const Eigen::MatrixXf& vertices);
    void setFace(const Eigen::MatrixXi& faces);

    // 버텍스 노멀을 계산해서 Eigen::MatrixXf로 반환
    Eigen::MatrixXf getVertexNormal() const;
    Eigen::MatrixXf getVertexNormal2() const;

private:
    Eigen::MatrixXf m_vertices; // (n x 3)
    Eigen::MatrixXi m_faces;    // (m x 3)
};

#endif // VNORMAL_H
