#pragma once 




std::optional<double> angle_between_vectors_rad(const std::array<double,3>& u, const std::array<double,3>& v, double eps = 1e-12);

std::array<double,3> vec_from_points(const std::array<double,3>& p, const std::array<double,3>& q);

