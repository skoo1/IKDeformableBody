#include "osim_joints.h"
#include <iostream>
#include <cmath>

using namespace Eigen;

// ---- CustomJoint ----
CustomJoint::CustomJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_)
    : axis(axis_), axis_flip(axis_flip_), nb_dof(axis_.size()) {}

size_t CustomJoint::get_dof() const {
    return nb_dof;
}

Matrix3d CustomJoint::axis_angle_to_matrix(const Vector3d& angle_axis) const {
    double angle = angle_axis.norm();
    if (angle < 1e-8) return Matrix3d::Identity();
    Vector3d axis_norm = angle_axis.normalized();
    return AngleAxisd(angle, axis_norm).toRotationMatrix();
}

Matrix3d CustomJoint::q_to_rot(const VectorXd& q) const {
    Matrix3d Rp = Matrix3d::Identity();
    for (size_t i = 0; i < nb_dof; ++i) {
        Vector3d angle_axis = q(i) * axis_flip[i] * axis[i];
        Matrix3d Rp_i = axis_angle_to_matrix(angle_axis);
        Rp = Rp_i * Rp;
    }
    return Rp;
}

// ---- CustomJoint1D ----
CustomJoint1D::CustomJoint1D(const Vector3d& axis_, double axis_flip_)
    : axis(axis_.normalized()), axis_flip(axis_flip_) {}

size_t CustomJoint1D::get_dof() const {
    return 1;
}

Matrix3d CustomJoint1D::q_to_rot(const VectorXd& q) const {
    Vector3d angle_axis = q(0) * axis_flip * axis;
    double angle = angle_axis.norm();
    if (angle < 1e-8) return Matrix3d::Identity();
    return AngleAxisd(angle, angle_axis.normalized()).toRotationMatrix();
}

// ---- WalkerKnee ----
size_t WalkerKnee::get_dof() const {
    return 1;
}

Matrix3d WalkerKnee::q_to_rot(const VectorXd& q) const {
    Vector3d angle_axis(0, 0, -q(0));
    double angle = angle_axis.norm();
    if (angle < 1e-8) return Matrix3d::Identity();
    return AngleAxisd(angle, angle_axis.normalized()).toRotationMatrix();
}

// ---- PinJoint ----
PinJoint::PinJoint(const Vector3d& parent_frame_ori_)
    : parent_frame_ori(parent_frame_ori_) {}

size_t PinJoint::get_dof() const {
    return 1;
}

Matrix3d PinJoint::axis_angle_to_matrix(const Vector3d& angle_axis) const {
    double angle = angle_axis.norm();
    if (angle < 1e-8) return Matrix3d::Identity();
    Vector3d axis_norm = angle_axis.normalized();
    return AngleAxisd(angle, axis_norm).toRotationMatrix();
}

Matrix3d PinJoint::q_to_rot(const VectorXd& q) const {
    AngleAxisd Rx(parent_frame_ori(0), Vector3d::UnitX());
    AngleAxisd Ry(parent_frame_ori(1), Vector3d::UnitY());
    AngleAxisd Rz(parent_frame_ori(2), Vector3d::UnitZ());

    Matrix3d Ra_i = (Rx * Ry * Rz).toRotationMatrix();
    Vector3d z_axis = Vector3d(0, 0, 1);
    Vector3d axis = Ra_i * z_axis;

    Vector3d angle_axis = q(0) * axis;
    double angle = angle_axis.norm();
    if (angle < 1e-8) return Matrix3d::Identity();
    Matrix3d R_pi = axis_angle_to_matrix(angle_axis);
    
    return R_pi;
}

// ---- ConstantCurvatureJoint ----
ConstantCurvatureJoint::ConstantCurvatureJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_)
    : CustomJoint(axis_, axis_flip_) {}

// ---- EllipsoidJoint ----
EllipsoidJoint::EllipsoidJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_)
    : CustomJoint(axis_, axis_flip_) {}