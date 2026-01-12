#ifndef JOINTS_DEF_H
#define JOINTS_DEF_H

#include <Eigen/Dense>

Eigen::Vector3d right_scapula(double angle_abduction, double angle_elevation, double thorax_width, double thorax_height);
Eigen::Vector3d left_scapula(double angle_abduction, double angle_elevation, double thorax_width, double thorax_height);
void curve_1d(const double angle, const double t, const double l, double &x, double &y);
Eigen::Vector3d curve_3d(const double angle_x, const double angle_y, const double t, const double l);

#endif // JOINTS_DEF_H
