#ifndef AR4_WRIST_SELECTION_CONTRACT_H
#define AR4_WRIST_SELECTION_CONTRACT_H

#include <cmath>

namespace ar4_protocol {

constexpr int kWristJointCount = 6;
constexpr int kWristFixedSeedCount = 7;
constexpr int kMaximumWristSolutions = kWristFixedSeedCount + 1;
constexpr float kWristCandidateToleranceDegrees = 0.001f;
constexpr float kWristSeedStepDegrees = 30.0f;
constexpr float kWristSignDeadbandDegrees = 0.5f;
constexpr float kWristSingularityDegrees = 2.0f;
constexpr float kWristExactPositionMillimetres = 0.001f;
constexpr float kWristExactRotationDegrees = 0.01f;
constexpr float kWristPositionValidationMillimetres = 0.1f;
constexpr float kWristRotationValidationDegrees = 0.1f;

inline bool wrist_config_valid(char wrist_config) {
    return wrist_config == 'A'
        || wrist_config == 'F'
        || wrist_config == 'N';
}

inline float wrist_seed_degrees(int seed_index) {
    return (
        seed_index - (kWristFixedSeedCount / 2)
    ) * kWristSeedStepDegrees;
}

inline float wrist_angular_difference(float left, float right) {
    const float wrapped_left = std::fmod(left, 360.0f);
    const float wrapped_right = std::fmod(right, 360.0f);
    float difference = std::fmod(
        wrapped_left - wrapped_right,
        360.0f
    );
    if (difference > 180.0f) difference -= 360.0f;
    if (difference < -180.0f) difference += 360.0f;
    return difference;
}

inline float wrist_angular_sum(float left, float right) {
    return wrist_angular_difference(
        std::fmod(left, 360.0f) + std::fmod(right, 360.0f),
        0.0f
    );
}

inline int wrist_sign(float value) {
    if (value > kWristSignDeadbandDegrees) return 1;
    if (value < -kWristSignDeadbandDegrees) return -1;
    return 0;
}

inline bool append_wrist_solution(
    float (&solutions)[kWristJointCount][kMaximumWristSolutions],
    int& solution_count,
    const float* candidate
) {
    if (
        candidate == nullptr
        || solution_count < 0
        || solution_count > kMaximumWristSolutions
    ) {
        return false;
    }
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        if (!std::isfinite(candidate[joint])) return false;
    }

    for (int solution = 0; solution < solution_count; ++solution) {
        bool duplicate = true;
        for (int joint = 0; joint < kWristJointCount; ++joint) {
            if (
                fabsf(solutions[joint][solution] - candidate[joint])
                > kWristCandidateToleranceDegrees
            ) {
                duplicate = false;
                break;
            }
        }
        if (duplicate) return false;
    }
    if (solution_count == kMaximumWristSolutions) return false;

    for (int joint = 0; joint < kWristJointCount; ++joint) {
        solutions[joint][solution_count] = candidate[joint];
    }
    ++solution_count;
    return true;
}

inline bool wrist_candidate_within_limits(
    const float* candidate,
    const float* upper_limits,
    const float* lower_limits
) {
    if (
        candidate == nullptr
        || upper_limits == nullptr
        || lower_limits == nullptr
    ) {
        return false;
    }
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        if (
            !std::isfinite(candidate[joint])
            || !std::isfinite(upper_limits[joint])
            || !std::isfinite(lower_limits[joint])
            || upper_limits[joint] < 0.0f
            || lower_limits[joint] < 0.0f
            || candidate[joint] > upper_limits[joint]
            || candidate[joint] < -lower_limits[joint]
        ) {
            return false;
        }
    }
    return true;
}

inline bool normalize_wrist_candidate(
    const float* candidate,
    const float* estimate,
    const float* upper_limits,
    const float* lower_limits,
    float* normalized
) {
    if (
        candidate == nullptr
        || estimate == nullptr
        || upper_limits == nullptr
        || lower_limits == nullptr
        || normalized == nullptr
    ) {
        return false;
    }

    float staged[kWristJointCount] = {};
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        if (
            !std::isfinite(candidate[joint])
            || !std::isfinite(estimate[joint])
            || !std::isfinite(upper_limits[joint])
            || !std::isfinite(lower_limits[joint])
            || upper_limits[joint] < 0.0f
            || lower_limits[joint] < 0.0f
        ) {
            return false;
        }

        double wrapped = std::fmod(
            static_cast<double>(candidate[joint]),
            360.0
        );
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;

        const double lower = -static_cast<double>(lower_limits[joint]);
        const double upper = static_cast<double>(upper_limits[joint]);
        const double tolerance = kWristCandidateToleranceDegrees;
        const double minimum_turn = std::ceil(
            (lower - tolerance - wrapped) / 360.0
        );
        const double maximum_turn = std::floor(
            (upper + tolerance - wrapped) / 360.0
        );
        if (
            !std::isfinite(minimum_turn)
            || !std::isfinite(maximum_turn)
            || minimum_turn > maximum_turn
        ) {
            return false;
        }

        const double estimated_turn = (
            static_cast<double>(estimate[joint]) - wrapped
        ) / 360.0;
        const double lower_turn = std::floor(estimated_turn);
        const double upper_turn = std::ceil(estimated_turn);
        const double lower_distance = std::fabs(estimated_turn - lower_turn);
        const double upper_distance = std::fabs(upper_turn - estimated_turn);
        double nearest_turn = lower_distance < upper_distance
            ? lower_turn
            : upper_turn;
        if (lower_distance == upper_distance) {
            nearest_turn = std::fabs(lower_turn) <= std::fabs(upper_turn)
                ? lower_turn
                : upper_turn;
        }
        if (nearest_turn < minimum_turn) nearest_turn = minimum_turn;
        if (nearest_turn > maximum_turn) nearest_turn = maximum_turn;

        double equivalent = wrapped + nearest_turn * 360.0;
        if (
            !std::isfinite(equivalent)
            || equivalent < lower - tolerance
            || equivalent > upper + tolerance
        ) {
            return false;
        }
        if (equivalent < lower) equivalent = lower;
        if (equivalent > upper) equivalent = upper;
        staged[joint] = static_cast<float>(equivalent);
        if (!std::isfinite(staged[joint])) return false;
    }
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        normalized[joint] = staged[joint];
    }
    return true;
}

inline bool wrist_pose_matches(
    const float* candidate_pose,
    const float* target_pose,
    float position_tolerance,
    float rotation_tolerance
) {
    if (
        candidate_pose == nullptr
        || target_pose == nullptr
        || !std::isfinite(position_tolerance)
        || !std::isfinite(rotation_tolerance)
        || position_tolerance < 0.0f
        || rotation_tolerance < 0.0f
    ) {
        return false;
    }
    for (int index = 0; index < 16; ++index) {
        if (
            !std::isfinite(candidate_pose[index])
            || !std::isfinite(target_pose[index])
        ) {
            return false;
        }
    }

    const double dx = static_cast<double>(candidate_pose[12])
        - static_cast<double>(target_pose[12]);
    const double dy = static_cast<double>(candidate_pose[13])
        - static_cast<double>(target_pose[13]);
    const double dz = static_cast<double>(candidate_pose[14])
        - static_cast<double>(target_pose[14]);
    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

    const int rotation_indices[9] = {0, 1, 2, 4, 5, 6, 8, 9, 10};
    double rotation_difference_squared = 0.0;
    for (int index : rotation_indices) {
        const double difference = static_cast<double>(candidate_pose[index])
            - static_cast<double>(target_pose[index]);
        rotation_difference_squared += difference * difference;
    }
    double half_rotation_sine =
        std::sqrt(rotation_difference_squared) / 2.8284271247461900976;
    if (half_rotation_sine > 1.0) half_rotation_sine = 1.0;
    const double rotation_error = 2.0
        * std::asin(half_rotation_sine)
        * 57.2957795130823208768;
    return position_error <= static_cast<double>(position_tolerance)
        && rotation_error <= static_cast<double>(rotation_tolerance);
}

template <typename CandidateValidator, typename Solver>
inline int generate_wrist_solutions(
    float (&solutions)[kWristJointCount][kMaximumWristSolutions],
    const float* target,
    const float* estimate,
    const float* upper_limits,
    const float* lower_limits,
    CandidateValidator candidate_reaches_target,
    Solver solve
) {
    if (
        target == nullptr
        || estimate == nullptr
        || upper_limits == nullptr
        || lower_limits == nullptr
    ) {
        return -1;
    }
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        if (!std::isfinite(target[joint]) || !std::isfinite(estimate[joint])) {
            return -1;
        }
    }

    int solution_count = 0;
    if (
        wrist_candidate_within_limits(
            estimate,
            upper_limits,
            lower_limits
        )
        && candidate_reaches_target(
            target,
            estimate,
            kWristExactPositionMillimetres,
            kWristExactRotationDegrees
        )
    ) {
        append_wrist_solution(solutions, solution_count, estimate);
    }

    for (
        int seed_index = 0;
        seed_index < kWristFixedSeedCount;
        ++seed_index
    ) {
        float seed[kWristJointCount] = {};
        float candidate[kWristJointCount] = {};
        float normalized[kWristJointCount] = {};
        for (int joint = 0; joint < kWristJointCount; ++joint) {
            seed[joint] = estimate[joint];
            candidate[joint] = estimate[joint];
        }
        seed[4] = wrist_seed_degrees(seed_index);
        candidate[4] = seed[4];
        if (!solve(target, candidate, seed)) continue;
        if (!normalize_wrist_candidate(
            candidate,
            estimate,
            upper_limits,
            lower_limits,
            normalized
        )) {
            continue;
        }
        if (!candidate_reaches_target(
            target,
            normalized,
            kWristPositionValidationMillimetres,
            kWristRotationValidationDegrees
        )) {
            continue;
        }
        append_wrist_solution(solutions, solution_count, normalized);
    }
    return solution_count;
}

inline int select_wrist_solution(
    const float (&solutions)[kWristJointCount][kMaximumWristSolutions],
    int solution_count,
    const float* estimate,
    char wrist_config
) {
    if (
        estimate == nullptr
        || solution_count <= 0
        || solution_count > kMaximumWristSolutions
        || !wrist_config_valid(wrist_config)
    ) {
        return -1;
    }
    for (int joint = 0; joint < kWristJointCount; ++joint) {
        if (!std::isfinite(estimate[joint])) return -1;
        for (int solution = 0; solution < solution_count; ++solution) {
            if (!std::isfinite(solutions[joint][solution])) return -1;
        }
    }

    int desired_sign = wrist_sign(estimate[4]);
    if (desired_sign == 0) desired_sign = 1;
    if (wrist_config == 'F') desired_sign = 1;
    if (wrist_config == 'N') desired_sign = -1;

    const float current_wrist_sum = wrist_angular_sum(
        estimate[3],
        estimate[5]
    );
    int best = -1;
    double best_cost = 1.0e300;

    for (int solution = 0; solution < solution_count; ++solution) {
        const float j5 = solutions[4][solution];
        const float absolute_j5 = fabsf(j5);
        const int solution_sign = wrist_sign(j5);
        double cost = 0.0;

        for (int joint = 0; joint < kWristJointCount; ++joint) {
            const double weight = joint == 4 ? 2.0 : 1.0;
            cost += weight * std::fabs(
                static_cast<double>(solutions[joint][solution])
                - static_cast<double>(estimate[joint])
            );
        }

        if (wrist_config == 'F' || wrist_config == 'N') {
            if (absolute_j5 > kWristSingularityDegrees) {
                if (solution_sign != desired_sign) continue;
            } else if (solution_sign != 0 && solution_sign != desired_sign) {
                cost += 200.0;
            }
        } else {
            if (
                absolute_j5 > kWristSingularityDegrees
                && solution_sign != 0
                && solution_sign != desired_sign
            ) {
                cost += 20.0;
            }
        }

        if (absolute_j5 <= kWristSingularityDegrees) {
            const float solution_wrist_sum = wrist_angular_sum(
                solutions[3][solution],
                solutions[5][solution]
            );
            cost += 5.0 * fabsf(
                wrist_angular_difference(
                    solution_wrist_sum,
                    current_wrist_sum
                )
            );
        }

        if (cost < best_cost) {
            best = solution;
            best_cost = cost;
        }
    }
    return best;
}

}  // namespace ar4_protocol

#endif
