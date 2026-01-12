#ifndef OSIM_JOINTS_H
#define OSIM_JOINTS_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class OsimJoint {
public:
    virtual Matrix3d q_to_rot(const VectorXd& q) const = 0;
    virtual size_t get_dof() const = 0;
};

class CustomJoint : public OsimJoint {
    protected:
        std::vector<Vector3d> axis;
        std::vector<double> axis_flip;
        size_t nb_dof;

        Matrix3d axis_angle_to_matrix(const Vector3d& angle_axis) const;

    public:
        CustomJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_);
        size_t get_dof() const override;
        Matrix3d q_to_rot(const VectorXd& q) const override;
};

class CustomJoint1D : public OsimJoint {
    Vector3d axis;
    double axis_flip;

public:
    CustomJoint1D(const Vector3d& axis_, double axis_flip_);
    size_t get_dof() const override;
    Matrix3d q_to_rot(const VectorXd& q) const override;
};

class WalkerKnee : public OsimJoint {
public:
    size_t get_dof() const override;
    Matrix3d q_to_rot(const VectorXd& q) const override;
};

class PinJoint : public OsimJoint {
    Vector3d parent_frame_ori;

public:
    PinJoint(const Vector3d& parent_frame_ori_);
    size_t get_dof() const override;
    Matrix3d axis_angle_to_matrix(const Vector3d& angle_axis) const;
    Matrix3d q_to_rot(const VectorXd& q) const override;
};

class ConstantCurvatureJoint : public CustomJoint {
public:
    ConstantCurvatureJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_);
};

class EllipsoidJoint : public CustomJoint {
public:
    EllipsoidJoint(const std::vector<Vector3d>& axis_, const std::vector<double>& axis_flip_);
};

#endif // OSIM_JOINTS_H
