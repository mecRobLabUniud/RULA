/**
 * rula_assessment.cpp
 *
 * Rapid Upper Limb Assessment (RULA) calculator using 3D skeleton keypoints.
 *
 * Keypoint layout (index → body part):
 *   0: Head              1: Left Shoulder    2: Right Shoulder
 *   3: Left Elbow        4: Right Elbow      5: Left Wrist
 *   6: Right Wrist       7: Upper Torso      8: Lower Torso
 *   9: Left Hip         10: Right Hip       11: Left Knee
 *  12: Right Knee       13: Left Ankle      14: Right Ankle
 *
 * All positions are assumed to be in meters, in a right-handed coordinate
 * system where Y is up (adjust verticalAxis if your system differs).
 */

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

struct Vec3 {
    double x, y, z;

    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }

    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    double norm()              const { return std::sqrt(dot(*this));   }
    Vec3   normalized()        const { double n = norm(); return (n > 1e-9) ? (*this)*(1.0/n) : Vec3{0,0,0}; }
    Vec3   cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }
};

// Angle in degrees between two vectors
static double angleDeg(const Vec3& a, const Vec3& b) {
    double c = a.normalized().dot(b.normalized());
    c = std::max(-1.0, std::min(1.0, c));   // clamp for numerical safety
    return std::acos(c) * 180.0 / M_PI;
}

// Keypoint indices
enum KP {
    HEAD=0, L_SHOULDER=1, R_SHOULDER=2,
    L_ELBOW=3,  R_ELBOW=4,
    L_WRIST=5,  R_WRIST=6,
    UPPER_TORSO=7, LOWER_TORSO=8,
    L_HIP=9,   R_HIP=10,
    L_KNEE=11, R_KNEE=12,
    L_ANKLE=13, R_ANKLE=14
};

using Skeleton = std::vector<std::array<double, 3>>;

// Convert a raw keypoint array to Vec3
static Vec3 toVec3(const std::array<double, 3>& p) {
    return {p[0], p[1], p[2]};
}

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
// Vertical reference (world up). Change to {0,0,1} for Z-up systems.
// ---------------------------------------------------------------------------
static const Vec3 WORLD_UP = {0.0, 1.0, 0.0};

// ---------------------------------------------------------------------------
// Group A scoring
// ---------------------------------------------------------------------------

// Step 1 – Upper Arm  (returns raw score 1-4, then adjustments added outside)
static int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                          const Vec3& hip,
                          const AdjustmentFlags& f)
{
    // Vector from shoulder downward along trunk (reference for 0°)
    Vec3 trunkDown = (hip - shoulder).normalized();
    Vec3 upperArm  = (elbow - shoulder).normalized();

    // Angle between upper arm and the trunk-down direction
    double ang = angleDeg(upperArm, trunkDown);
    // ang ≈ 0  → arm hanging straight down (neutral)
    // ang ≈ 90 → arm horizontal
    // ang ≈ 180→ arm fully raised overhead

    // RULA defines flexion relative to the trunk:
    // neutral (arm down) = 0°, fully raised forward = 180°
    // We map the raw angle to RULA convention:
    //   <20° ≈ arm near vertical → score 1
    int score;
    if      (ang <= 20)  score = 1;
    else if (ang <= 45)  score = 2;
    else if (ang <= 90)  score = 3;
    else                 score = 4;

    if (f.shoulderRaised)   ++score;
    if (f.upperArmAbducted) ++score;
    if (f.armSupported)     --score;

    return std::max(1, score);
}

// Step 2 – Lower Arm
static int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                          const Vec3& wrist,
                          const AdjustmentFlags& f)
{
    Vec3 upper = (shoulder - elbow).normalized();
    Vec3 lower = (wrist    - elbow).normalized();
    double ang = angleDeg(upper, lower);
    // ang is the elbow flexion angle (0°=fully extended, 180°=fully flexed)
    // RULA uses flexion from 0°: score 1 for 60-100°, score 2 otherwise
    // Note: angleDeg gives supplementary angle relative to straight line.
    // elbow angle in RULA = 180° - ang (flexion from straight)
    double flexion = 180.0 - ang;

    int score = (flexion >= 60 && flexion <= 100) ? 1 : 2;

    if (f.crossingMidlineOrOut) ++score;

    return score;
}

// Step 3 – Wrist
// We estimate flexion/extension from the wrist-elbow-shoulder plane.
// For a more accurate result the hand direction should be used; here we
// proxy it with the forearm vector projected against the trunk frontal plane.
static int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                       const Vec3& shoulder,
                       const AdjustmentFlags& f)
{
    // Approximate wrist flexion: angle between forearm and world vertical
    Vec3 forearm = (wrist - elbow).normalized();
    double angFromVertical = angleDeg(forearm, WORLD_UP);
    // When forearm is horizontal (90° from up) and wrist is neutral,
    // deviation from 90° approximates flexion/extension.
    double approxFlexExt = std::abs(angFromVertical - 90.0);

    int score;
    if      (approxFlexExt <= 5)  score = 1;   // near neutral
    else if (approxFlexExt <= 15) score = 2;
    else                          score = 3;

    if (f.wristDeviated) ++score;

    return score;
}

// Step 4 – Wrist Twist
// Without a hand keypoint this must be supplied externally.
// Default: mid-range = 1, end of range = 2.
static int scoreWristTwist(bool atEndOfRange) {
    return atEndOfRange ? 2 : 1;
}

// Group A lookup table  [upperArm-1][lowerArm-1][wrist-1][wristTwist-1]
// Source: McAtamney & Corlett (1993), Table 2
static const int GROUP_A_TABLE[4][2][4][2] = {
    // Upper Arm 1
    { { {1,2},{2,2},{2,3},{3,3} },
      { {2,2},{2,2},{3,3},{3,3} } },
    // Upper Arm 2
    { { {2,3},{3,3},{3,3},{4,4} },
      { {3,3},{3,3},{3,4},{4,4} } },
    // Upper Arm 3
    { { {3,3},{4,4},{4,4},{5,5} },
      { {3,4},{4,4},{4,4},{5,5} } },
    // Upper Arm 4
    { { {4,4},{4,4},{4,5},{5,5} },
      { {4,4},{4,4},{5,5},{6,6} } }
};

static int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist)
{
    int ua = std::min(std::max(upperArm,  1), 4) - 1;
    int la = std::min(std::max(lowerArm,  1), 2) - 1;
    int w  = std::min(std::max(wrist,     1), 4) - 1;
    int wt = std::min(std::max(wristTwist,1), 2) - 1;
    return GROUP_A_TABLE[ua][la][w][wt];
}

// ---------------------------------------------------------------------------
// Group B scoring
// ---------------------------------------------------------------------------

// Step 8 – Neck
static int scoreNeck(const Vec3& head, const Vec3& upperTorso,
                      const AdjustmentFlags& f)
{
    Vec3 neck = (head - upperTorso).normalized();
    double ang = angleDeg(neck, WORLD_UP);
    // ang ≈ 0  → head straight up (neutral extension reference)
    // RULA flexion = angle forward from vertical
    int score;
    if      (ang <= 10) score = 1;
    else if (ang <= 20) score = 2;
    else if (ang <= 90) score = 3;
    else                score = 4;   // extension

    if (f.neckTwisted)   ++score;
    if (f.neckSideBent)  ++score;

    return score;
}

// Step 9 – Trunk
static int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                       const AdjustmentFlags& f)
{
    Vec3 trunk = (upperTorso - lowerTorso).normalized();
    double ang = angleDeg(trunk, WORLD_UP);

    int score;
    if      (ang <= 5)  score = 1;   // well-supported / upright
    else if (ang <= 20) score = 2;
    else if (ang <= 60) score = 3;
    else                score = 4;

    if (f.trunkTwisted)   ++score;
    if (f.trunkSideBent)  ++score;

    return score;
}

// Step 10 – Legs
// Heuristic: compare hip–knee and knee–ankle vectors; large deviation = unstable
static int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
                      const Vec3& lKnee, const Vec3& rKnee,
                      const Vec3& lAnkle,const Vec3& rAnkle)
{
    Vec3 lThigh  = (lKnee  - lHip).normalized();
    Vec3 lShank  = (lAnkle - lKnee).normalized();
    Vec3 rThigh  = (rKnee  - rHip).normalized();
    Vec3 rShank  = (rAnkle - rKnee).normalized();

    double lKneeAng = angleDeg(lThigh, lShank);
    double rKneeAng = angleDeg(rThigh, rShank);

    // If knees are roughly straight (legs extended / well-supported standing
    // or balanced sitting), score 1; otherwise score 2.
    bool balanced = (lKneeAng < 30 || lKneeAng > 150) &&
                    (rKneeAng < 30 || rKneeAng > 150);
    return balanced ? 1 : 2;
}

// Group B lookup table  [neck-1][trunk-1][legs-1]
// Source: McAtamney & Corlett (1993), Table 3
static const int GROUP_B_TABLE[4][5][2] = {
    // Neck 1
    { {1,3},{2,3},{3,4},{5,5},{7,7} },
    // Neck 2
    { {2,3},{2,3},{4,5},{5,6},{7,7} },
    // Neck 3
    { {3,3},{3,4},{5,6},{6,7},{7,8} },
    // Neck 4
    { {5,5},{5,6},{6,7},{7,8},{8,9} }
};

static int lookupGroupB(int neck, int trunk, int legs)
{
    int n = std::min(std::max(neck,  1), 4) - 1;
    int t = std::min(std::max(trunk, 1), 5) - 1;
    int l = std::min(std::max(legs,  1), 2) - 1;
    return GROUP_B_TABLE[n][t][l];
}

// ---------------------------------------------------------------------------
// Grand Score lookup table  [scoreA-1][scoreB-1]
// Source: McAtamney & Corlett (1993), Table 4
// ---------------------------------------------------------------------------
static const int GRAND_SCORE_TABLE[8][8] = {
    {1, 2, 3, 3, 4, 5, 5},   // Score A = 1
    {2, 2, 3, 4, 4, 5, 5},   // Score A = 2
    {3, 3, 3, 4, 4, 5, 6},   // Score A = 3
    {3, 3, 3, 4, 5, 6, 6},   // Score A = 4
    {4, 4, 4, 5, 6, 7, 7},   // Score A = 5
    {4, 4, 5, 6, 6, 7, 7},   // Score A = 6
    {5, 5, 6, 6, 7, 7, 7},   // Score A = 7
    {5, 5, 6, 7, 7, 7, 7}    // Score A = 8+
};

static int lookupGrandScore(int scoreA, int scoreB)
{
    int a = std::min(std::max(scoreA, 1), 8) - 1;
    int b = std::min(std::max(scoreB, 1), 7) - 1;
    return GRAND_SCORE_TABLE[a][b];
}

// ---------------------------------------------------------------------------
// Action level interpretation
// ---------------------------------------------------------------------------
static std::string actionLevel(int grandScore)
{
    if      (grandScore <= 2) return "Level 1 – Acceptable posture; no action required.";
    else if (grandScore <= 4) return "Level 2 – Further investigation; changes may be needed.";
    else if (grandScore <= 6) return "Level 3 – Investigation and changes needed soon.";
    else                      return "Level 4 – Immediate investigation and changes required!";
}

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
                        bool wristAtEndOfRange = false)
{
    RULAResult r{};

    // Select left or right keypoints
    int shoulder_idx = (side == 'L') ? L_SHOULDER : R_SHOULDER;
    int elbow_idx    = (side == 'L') ? L_ELBOW    : R_ELBOW;
    int wrist_idx    = (side == 'L') ? L_WRIST    : R_WRIST;
    int hip_idx      = (side == 'L') ? L_HIP      : R_HIP;

    const Vec3 shoulder    = toVec3(kp[shoulder_idx]);
    const Vec3 elbow       = toVec3(kp[elbow_idx]);
    const Vec3 wrist       = toVec3(kp[wrist_idx]);
    const Vec3 hip         = toVec3(kp[hip_idx]);
    const Vec3 upperTorso  = toVec3(kp[UPPER_TORSO]);
    const Vec3 lowerTorso  = toVec3(kp[LOWER_TORSO]);
    const Vec3 head        = toVec3(kp[HEAD]);

    // --- Group A ---
    r.upperArmScore   = scoreUpperArm(shoulder, elbow, hip, f);
    r.lowerArmScore   = scoreLowerArm(shoulder, elbow, wrist, f);
    r.wristScore      = scoreWrist(elbow, wrist, shoulder, f);
    r.wristTwistScore = scoreWristTwist(wristAtEndOfRange);

    r.postureScoreA   = lookupGroupA(r.upperArmScore, r.lowerArmScore,
                                     r.wristScore,    r.wristTwistScore);

    r.muscleUseScoreA = (f.isStaticPosture ? 1 : 0) + (f.isRepeated ? 1 : 0);
    r.forceScoreA     = f.forceScoreA;
    r.finalScoreA     = r.postureScoreA + r.muscleUseScoreA + r.forceScoreA;

    // --- Group B ---
    r.neckScore  = scoreNeck(head, upperTorso, f);
    r.trunkScore = scoreTrunk(upperTorso, lowerTorso, f);
    r.legScore   = scoreLegs(toVec3(kp[L_HIP]), toVec3(kp[R_HIP]),
                              toVec3(kp[L_KNEE]), toVec3(kp[R_KNEE]),
                              toVec3(kp[L_ANKLE]), toVec3(kp[R_ANKLE]));

    r.postureScoreB   = lookupGroupB(r.neckScore, r.trunkScore, r.legScore);
    r.muscleUseScoreB = (f.isStaticPosture ? 1 : 0) + (f.isRepeated ? 1 : 0);
    r.forceScoreB     = f.forceScoreB;
    r.finalScoreB     = r.postureScoreB + r.muscleUseScoreB + r.forceScoreB;

    // --- Grand Score ---
    r.grandScore = lookupGrandScore(r.finalScoreA, r.finalScoreB);
    r.action     = actionLevel(r.grandScore);

    return r;
}

// ---------------------------------------------------------------------------
// Example usage
// ---------------------------------------------------------------------------
int main()
{
    // Example skeleton: worker reaching forward with right arm,
    // slightly bent trunk, head looking down.
    // Coordinates in meters, Y-up.
    Skeleton kp = {
        /* 0  HEAD         */ { 0.00,  1.70,  0.05},
        /* 1  L_SHOULDER   */ {-0.18,  1.45,  0.00},
        /* 2  R_SHOULDER   */ { 0.18,  1.45,  0.00},
        /* 3  L_ELBOW      */ {-0.25,  1.20,  0.00},
        /* 4  R_ELBOW      */ { 0.25,  1.10,  0.20},
        /* 5  L_WRIST      */ {-0.28,  1.00,  0.00},
        /* 6  R_WRIST      */ { 0.30,  0.95,  0.40},
        /* 7  UPPER_TORSO  */ { 0.00,  1.35, -0.02},
        /* 8  LOWER_TORSO  */ { 0.00,  1.00, -0.05},
        /* 9  L_HIP        */ {-0.10,  0.90,  0.00},
        /* 10 R_HIP        */ { 0.10,  0.90,  0.00},
        /* 11 L_KNEE       */ {-0.10,  0.50,  0.00},
        /* 12 R_KNEE       */ { 0.10,  0.50,  0.00},
        /* 13 L_ANKLE      */ {-0.10,  0.05,  0.00},
        /* 14 R_ANKLE      */ { 0.10,  0.05,  0.00},
    };

    
    AdjustmentFlags flags;
    flags.isRepeated    = true;   // task is repetitive
    flags.forceScoreA   = 1;      // 2-10 kg load, intermittent
    flags.forceScoreB   = 1;

    

    auto start = std::chrono::steady_clock::now();

    RULAResult result = computeRULA(kp, flags, 'R', false);

    auto end = std::chrono::steady_clock::now();

    // Cast to whatever unit you need
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    result.print();

    std::cout << "Elapsed time: " << elapsed_ms << "ms" << std::endl;

    return 0;
}