#include <string>
#include <iostream>
#include <cmath>
#include <array>
#include <optional>

#include "angle_calculation.h"
#include "skeleton_receiver.h"

constexpr double PI = 3.14159265358979323846;


int main() {
    // Example: segment A from A1 to A2, segment B from B1 to B2
    std::array<double,3> A1 = {0.0, 0.0, 0.0};
    std::array<double,3> A2 = {1.0, 1.0, 0.0};
    std::array<double,3> B1 = {0.0, 0.0, 0.0};
    std::array<double,3> B2 = {1.0, 0.0, 1.0};

    auto u = vec_from_points(A1, A2);
    auto v = vec_from_points(B1, B2);

    auto angle_opt = angle_between_vectors_rad(u, v);
    if (!angle_opt) {
        std::cerr << "One of the segments has near-zero length; angle undefined.\n";
        return 1;
    }

    double angle_rad = *angle_opt;
    double angle_deg = angle_rad * 180.0 / PI;

    std::cout << "Angle = " << angle_rad << " radians (" << angle_deg << " degrees)\n";
    



    // SkeletonZmqSubscriber skelSub("ipc:///tmp/skeleton.ipc");
    SkeletonZmqSubscriber skelSub(10, "MERGED", 7000);
    
    skelSub.start();

    // SkeletonCapsuleBuffer skel;
    // skelSub.readLatest(skel); 
    // std.cout << skel << std.endl;
    std::string tmp;
    std::cin >> tmp;

    skelSub.stop();

    return 0;
}