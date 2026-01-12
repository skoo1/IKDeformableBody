#include "joints_def.h"
#include <cmath>

double M_PI = 3.14159265358979323846;

Eigen::Vector3d right_scapula(double angle_abduction, double angle_elevation, double thorax_width, double thorax_height) {
    double radius_x = thorax_width / 4.0f * std::cos(angle_elevation - M_PI / 4.0f);
    double radius_y = thorax_width / 4.0f;
    double radius_z = thorax_height / 2.0f;

    Eigen::Vector3d t;
    t << -radius_x * std::cos(angle_abduction),
         -radius_z * std::sin(angle_elevation - M_PI / 4.0f),
          radius_y * std::sin(angle_abduction);

    return t;
}

Eigen::Vector3d left_scapula(double angle_abduction, double angle_elevation, double thorax_width, double thorax_height) {
    angle_abduction = -angle_abduction;
    angle_elevation = -angle_elevation;

    double radius_x = thorax_width / 4.0f * std::cos(angle_elevation - M_PI / 4.0f);
    double radius_y = thorax_width / 4.0f;
    double radius_z = thorax_height / 2.0f;

    Eigen::Vector3d t;
    t << radius_x * std::cos(angle_abduction),
        -radius_z * std::sin(angle_elevation - M_PI / 4.0f),
         radius_y * std::sin(angle_abduction);

    return t;
}

void curve_1d(const double angle, const double t, const double l, double &x, double &y) {
    const double epsilon = 1e-5f;
    if (std::abs(angle) < epsilon) {
        x = l * t * t * angle / 2.0f;
        y = l * t * (1.0f - t * t * t * angle * angle / 6.0f);
    } else {
        double r = l / angle;
        x = r * (1.0f - std::cos(t * angle));
        y = r * std::sin(t * angle);
    }
}

Eigen::Vector3d curve_3d(const double angle_x, const double angle_y, const double t, const double l) {
    double x1, y1, x2, y2;
    
    curve_1d(angle_x, t, l, x1, y1);
    Eigen::Vector3d tx(-x1, y1, 0.0f);

    curve_1d(angle_y, t, l, x2, y2);
    Eigen::Vector3d ty(0.0f, y2, -x2);

    return tx + ty;
}
