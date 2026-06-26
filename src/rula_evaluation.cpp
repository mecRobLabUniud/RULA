#include <string>
#include <iostream>
#include <cmath>
#include <array>
#include <optional>

// #include "angle_calculation.h"
#include "skeleton_receiver.h"
#include "rula_score_computation.h"

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












int rula_evaluation(Skeleton skeleton) {

    AdjustmentFlags flags;
    flags.isRepeated    = true;   // task is repetitive
    flags.forceScoreA   = 1;      // 2-10 kg load, intermittent
    flags.forceScoreB   = 1;

    

    auto start = std::chrono::steady_clock::now();

    RULAResult result = computeRULA(skeleton, flags, 'R', false);

    auto end = std::chrono::steady_clock::now();

    // Cast to whatever unit you need
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    result.print();

    std::cout << "Elapsed time: " << elapsed_ms << "ms" << std::endl;

    return result.grandScore;
}






int main() {
    SkeletonZmqSubscriber skelSub(10, "MERGED", 7000);
    
    skelSub.start();
    while (true) {
        std::string msg = skelSub.get_skeleton_data();
        std::string label;
        if (msg != "") {
            std::cout << msg << std::endl;
            auto skeleton = parseMergedString(msg, label);

            int result = rula_evaluation(skeleton);
        }
    }
    skelSub.stop();

    return 0;
}



// // ---------------------------------------------------------------------------
// // Example usage
// // ---------------------------------------------------------------------------
// int main()
// {
//     // Example skeleton: worker reaching forward with right arm,
//     // slightly bent trunk, head looking down.
//     // Coordinates in meters, Y-up.
//     Skeleton kp = {
//         /* 0  HEAD         */ { 0.00,  1.70,  0.05},
//         /* 1  L_SHOULDER   */ {-0.18,  1.45,  0.00},
//         /* 2  R_SHOULDER   */ { 0.18,  1.45,  0.00},
//         /* 3  L_ELBOW      */ {-0.25,  1.20,  0.00},
//         /* 4  R_ELBOW      */ { 0.25,  1.10,  0.20},
//         /* 5  L_WRIST      */ {-0.28,  1.00,  0.00},
//         /* 6  R_WRIST      */ { 0.30,  0.95,  0.40},
//         /* 7  UPPER_TORSO  */ { 0.00,  1.35, -0.02},
//         /* 8  LOWER_TORSO  */ { 0.00,  1.00, -0.05},
//         /* 9  L_HIP        */ {-0.10,  0.90,  0.00},
//         /* 10 R_HIP        */ { 0.10,  0.90,  0.00},
//         /* 11 L_KNEE       */ {-0.10,  0.50,  0.00},
//         /* 12 R_KNEE       */ { 0.10,  0.50,  0.00},
//         /* 13 L_ANKLE      */ {-0.10,  0.05,  0.00},
//         /* 14 R_ANKLE      */ { 0.10,  0.05,  0.00},
//     };
// 
//     
//     AdjustmentFlags flags;
//     flags.isRepeated    = true;   // task is repetitive
//     flags.forceScoreA   = 1;      // 2-10 kg load, intermittent
//     flags.forceScoreB   = 1;
// 
//     
// 
//     auto start = std::chrono::steady_clock::now();
// 
//     RULAResult result = computeRULA(kp, flags, 'R', false);
// 
//     auto end = std::chrono::steady_clock::now();
// 
//     // Cast to whatever unit you need
//     double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
//     
//     result.print();
// 
//     std::cout << "Elapsed time: " << elapsed_ms << "ms" << std::endl;
// 
//     return 0;
// }