#include <string>
#include <iostream>
#include <cmath>
#include <array>
#include <optional>

#include "angle_calculation.h"
#include "skeleton_receiver.h"

constexpr double PI = 3.14159265358979323846;


// Parse a single [x, y, z] triplet from a string starting at pos
std::array<double, 3> parseTriplet(const std::string& s, size_t& pos) {
    // Find opening '['
    pos = s.find('[', pos);
    if (pos == std::string::npos) throw std::runtime_error("Expected '['");
    pos++; // skip '['

    std::array<double, 3> triplet;
    for (int i = 0; i < 3; i++) {
        // Skip whitespace and commas
        while (pos < s.size() && (s[pos] == ' ' || s[pos] == ',')) pos++;

        // Extract token until ',' or ']'
        size_t end = pos;
        while (end < s.size() && s[end] != ',' && s[end] != ']') end++;

        std::string token = s.substr(pos, end - pos);
        // Trim whitespace
        token.erase(0, token.find_first_not_of(' '));
        token.erase(token.find_last_not_of(' ') + 1);

        if (token == "NaN" || token == "nan") {
            triplet[i] = std::numeric_limits<double>::quiet_NaN();
        } else {
            triplet[i] = std::stod(token);
        }
        pos = end;
    }

    // Find closing ']'
    pos = s.find(']', pos);
    if (pos == std::string::npos) throw std::runtime_error("Expected ']'");
    pos++; // skip ']'

    return triplet;
}

// Parse the full keypoint list [[x,y,z], [x,y,z], ...]
std::vector<std::array<double, 3>> parseKeypointList(const std::string& s, size_t& pos) {
    // Find opening '[' of the outer list
    pos = s.find('[', pos);
    if (pos == std::string::npos) throw std::runtime_error("Expected outer '['");
    pos++; // skip outer '['

    std::vector<std::array<double, 3>> keypoints;

    while (pos < s.size()) {
        // Skip whitespace and commas
        while (pos < s.size() && (s[pos] == ' ' || s[pos] == ',' || s[pos] == '\n')) pos++;

        if (s[pos] == ']') { pos++; break; } // end of outer list
        if (s[pos] == '[') {
            keypoints.push_back(parseTriplet(s, pos));
        } else {
            pos++;
        }
    }

    return keypoints;
}

// Top-level parser: extracts label and first list from the merged string
std::vector<std::array<double, 3>> parseMergedString(const std::string& input, std::string& label) {
    size_t pos = 0;

    // Extract label (everything before the first ';')
    size_t semicolon = input.find(';');
    if (semicolon == std::string::npos) throw std::runtime_error("Expected ';' separator");
    label = input.substr(0, semicolon);
    label.erase(label.find_last_not_of(' ') + 1); // trim trailing space

    pos = semicolon + 1;

    return parseKeypointList(input, pos);
}





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
    while (true) {
        std::string msg = skelSub.get_skeleton_data();
        std::string label;
        if (msg != "") {
            auto keypoints = parseMergedString(msg, label);
        }
        
        // std::cout << keypoints[0][0] << std::endl;
    }
    skelSub.stop();

    return 0;
}