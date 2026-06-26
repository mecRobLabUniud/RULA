#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

struct Vec3;

// Angle in degrees between two vectors
static double angleDeg(const Vec3& a, const Vec3& b);

using Skeleton = std::vector<std::array<double, 3>>;

// Convert a raw keypoint array to Vec3
static Vec3 toVec3(const std::array<double, 3>& p);

// ---------------------------------------------------------------------------
// Optional adjustment flags (set per-observation if known)
// ---------------------------------------------------------------------------
struct AdjustmentFlags {
    // Group A – Upper Arm
    bool shoulderRaised    = false;   // +1
    bool upperArmAbducted  = false;   // +1
    bool armSupported      = false;   // -1

    // Group A – Lower Arm
    bool crossingMidlineOrOut = false; // +1

    // Group A – Wrist
    bool wristDeviated     = false;   // +1 (radial/ulnar)

    // Group B – Neck
    bool neckTwisted       = false;   // +1
    bool neckSideBent      = false;   // +1

    // Group B – Trunk
    bool trunkTwisted      = false;   // +1
    bool trunkSideBent     = false;   // +1

    // Muscle use & force (applied identically to both groups A and B)
    bool isStaticPosture   = false;   // held >1 min  → +1
    bool isRepeated        = false;   // >4 times/min → +1

    // Force/load score (0–3) for group A and B independently
    int  forceScoreA       = 0;       // 0=<2kg intermittent … 3=shock/rapid
    int  forceScoreB       = 0;
};

// ---------------------------------------------------------------------------
// Group A scoring
// ---------------------------------------------------------------------------

// Step 1 – Upper Arm  (returns raw score 1-4, then adjustments added outside)
static int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                          const Vec3& hip,
                          const AdjustmentFlags& f);

// Step 2 – Lower Arm
static int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                          const Vec3& wrist,
                          const AdjustmentFlags& f);

// Step 3 – Wrist
// We estimate flexion/extension from the wrist-elbow-shoulder plane.
// For a more accurate result the hand direction should be used; here we
// proxy it with the forearm vector projected against the trunk frontal plane.
static int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                       const Vec3& shoulder,
                       const AdjustmentFlags& f);

// Step 4 – Wrist Twist
// Without a hand keypoint this must be supplied externally.
// Default: mid-range = 1, end of range = 2.
static int scoreWristTwist(bool atEndOfRange);


static int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist);

// ---------------------------------------------------------------------------
// Group B scoring
// ---------------------------------------------------------------------------

// Step 8 – Neck
static int scoreNeck(const Vec3& head, const Vec3& upperTorso,
                      const AdjustmentFlags& f);

// Step 9 – Trunk
static int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                       const AdjustmentFlags& f);

// Step 10 – Legs
// Heuristic: compare hip–knee and knee–ankle vectors; large deviation = unstable
static int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
                      const Vec3& lKnee, const Vec3& rKnee,
                      const Vec3& lAnkle,const Vec3& rAnkle);



static int lookupGroupB(int neck, int trunk, int legs);


static int lookupGrandScore(int scoreA, int scoreB);
// ---------------------------------------------------------------------------
// Action level interpretation
// ---------------------------------------------------------------------------
static std::string actionLevel(int grandScore);

// ---------------------------------------------------------------------------
// Main RULA assessment struct
// ---------------------------------------------------------------------------
struct RULAResult {
    // Group A intermediates
    int upperArmScore, lowerArmScore, wristScore, wristTwistScore;
    int postureScoreA, muscleUseScoreA, forceScoreA, finalScoreA;

    // Group B intermediates
    int neckScore, trunkScore, legScore;
    int postureScoreB, muscleUseScoreB, forceScoreB, finalScoreB;

    // Grand score
    int grandScore;
    std::string action;

    void print() const {
        std::cout << "=== RULA Assessment ===\n\n";

        std::cout << "-- Group A (Upper Limb) --\n";
        std::cout << "  Upper Arm score : " << upperArmScore   << "\n";
        std::cout << "  Lower Arm score : " << lowerArmScore   << "\n";
        std::cout << "  Wrist score     : " << wristScore      << "\n";
        std::cout << "  Wrist Twist     : " << wristTwistScore << "\n";
        std::cout << "  Posture Score A : " << postureScoreA   << "\n";
        std::cout << "  Muscle Use A    : +" << muscleUseScoreA << "\n";
        std::cout << "  Force Score A   : +" << forceScoreA     << "\n";
        std::cout << "  >>> Final Score A: " << finalScoreA     << "\n\n";

        std::cout << "-- Group B (Neck/Trunk/Legs) --\n";
        std::cout << "  Neck score      : " << neckScore       << "\n";
        std::cout << "  Trunk score     : " << trunkScore      << "\n";
        std::cout << "  Leg score       : " << legScore        << "\n";
        std::cout << "  Posture Score B : " << postureScoreB   << "\n";
        std::cout << "  Muscle Use B    : +" << muscleUseScoreB << "\n";
        std::cout << "  Force Score B   : +" << forceScoreB     << "\n";
        std::cout << "  >>> Final Score B: " << finalScoreB     << "\n\n";

        std::cout << "=== Grand Score: " << grandScore << " ===\n";
        std::cout << action << "\n";
    }
};

// ---------------------------------------------------------------------------
// Top-level function
// ---------------------------------------------------------------------------

/**
 * @param kp       15-element array of 3D keypoints (see enum KP for indices)
 * @param f        Adjustment flags (posture details not inferrable from kp)
 * @param side     Which side to assess: 'L' or 'R' (RULA is per-side)
 * @param wristAtEndOfRange  true if wrist is at end of rotation range
 */
RULAResult computeRULA(const Skeleton& kp,
                        const AdjustmentFlags& f,
                        char side = 'R',
                        bool wristAtEndOfRange = false);
