#include <iostream>
#include <cmath>
#include <array>
#include <optional>

#include "angle_calculation.h"

// Compute angle between vectors u and v in radians.
// Returns std::nullopt if either vector has (near) zero length.
std::optional<double> angle_between_vectors_rad(const std::array<double,3>& u, const std::array<double,3>& v, double eps) {
    double dot = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    double nu = std::sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    double nv = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    if (nu <= eps || nv <= eps) return std::nullopt;

    double cos_theta = dot / (nu * nv);
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    return std::optional<double>(std::acos(cos_theta));
}

// Helper: form vector from segment (p->q)
std::array<double,3> vec_from_points(const std::array<double,3>& p, const std::array<double,3>& q) {
    return { q[0]-p[0], q[1]-p[1], q[2]-p[2] };
}