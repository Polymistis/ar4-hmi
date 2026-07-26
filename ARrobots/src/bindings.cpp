#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#ifdef _MSC_VER
#pragma warning(push)
// The generated single-precision solver intentionally narrows math-library results.
#pragma warning(disable : 4244 4305)
#endif
#include "kinematics.cpp"  // Preserve the generated solver within the binding translation unit.
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace py = pybind11;

float CheckedFloat(double value, const char* label) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(label) + " must be finite");
    }
    if (std::fabs(value) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument(
            std::string(label) + " exceeds the native finite float range"
        );
    }
    const float converted = static_cast<float>(value);
    if (value != 0.0 && converted == 0.0f) {
        throw std::invalid_argument(
            std::string(label) + " underflows the native float range"
        );
    }
    return converted;
}

std::vector<float> CheckedMotionVector(
    const std::vector<double>& values,
    const char* size_error,
    const char* value_label
) {
    if (values.size() != ROBOT_nDOFs) {
        throw std::invalid_argument(size_error);
    }
    std::vector<float> converted;
    converted.reserve(ROBOT_nDOFs);
    for (double value : values) {
        converted.push_back(CheckedFloat(value, value_label));
    }
    return converted;
}

void ValidateDegreeToRadianValues(
    const std::vector<float>& values,
    std::size_t first,
    const char* label
) {
    for (std::size_t index = first; index < values.size(); ++index) {
        const float radians = values[index] * kRadiansPerDegree;
        if (values[index] != 0.0f && radians == 0.0f) {
            throw std::invalid_argument(
                std::string(label)
                + " cannot be represented in native radians"
            );
        }
    }
}

void SetDHParametersExplicit(
    double theta1, double theta2, double theta3,
    double theta4, double theta5, double theta6,
    double alpha1, double alpha2, double alpha3,
    double alpha4, double alpha5, double alpha6,
    double a1, double a2, double a3, double a4, double a5, double a6,
    double d1, double d2, double d3, double d4, double d5, double d6
) {
    const std::array<double, 24> input = {
        theta1, theta2, theta3, theta4, theta5, theta6,
        alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,
        a1, a2, a3, a4, a5, a6,
        d1, d2, d3, d4, d5, d6,
    };
    std::array<float, 24> parameters{};
    for (std::size_t index = 0; index < input.size(); ++index) {
        parameters[index] = CheckedFloat(input[index], "DH parameter");
    }

    for (std::size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
        float* link = Robot_Kin_DHM_Table + joint * Table_Size;
        link[DHM_Theta] = parameters[joint];
        link[DHM_Alpha] = parameters[ROBOT_nDOFs + joint];
        link[DHM_A] = parameters[2 * ROBOT_nDOFs + joint];
        link[DHM_D] = parameters[3 * ROBOT_nDOFs + joint];
    }
}

void SetJointLimits(
    const std::vector<double>& pos_limits,
    const std::vector<double>& neg_limits
) {
    if (pos_limits.size() != ROBOT_nDOFs || neg_limits.size() != ROBOT_nDOFs)
        throw std::invalid_argument(
            "Expected positive and negative limit vectors of size "
            + std::to_string(ROBOT_nDOFs)
        );

    std::array<float, ROBOT_nDOFs> positive{};
    std::array<float, ROBOT_nDOFs> negative{};
    for (std::size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
        positive[joint] = CheckedFloat(pos_limits[joint], "Positive joint limit");
        negative[joint] = CheckedFloat(neg_limits[joint], "Negative joint limit");
        if (positive[joint] < 0.0f || negative[joint] < 0.0f) {
            throw std::invalid_argument("Joint limits must be finite, non-negative magnitudes");
        }
    }

    for (std::size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
        Robot_JointLimits_Upper[joint] = positive[joint];
        Robot_JointLimits_Lower[joint] = negative[joint];
    }
}

void SetRobotToolFrame(
    double x,
    double y,
    double z,
    double rx_deg,
    double ry_deg,
    double rz_deg
) {
    const std::array<double, 6> input = {x, y, z, rx_deg, ry_deg, rz_deg};
    std::array<float, 6> values{};
    for (std::size_t index = 0; index < input.size(); ++index) {
        values[index] = CheckedFloat(input[index], "Tool-frame value");
    }
    const std::array<float, 6> native_tool_frame = BuildRobotToolFrame(
        values[0], values[1], values[2],
        values[3], values[4], values[5]
    );
    ApplyRobotToolFrame(native_tool_frame);
}

void SetRobotConfiguration(
    const std::vector<double>& dh_parameters,
    const std::vector<double>& pos_limits,
    const std::vector<double>& neg_limits,
    const std::vector<double>& tool_frame
) {
    if (dh_parameters.size() != 24) {
        throw std::invalid_argument("Expected 24 DH parameters");
    }
    if (pos_limits.size() != ROBOT_nDOFs || neg_limits.size() != ROBOT_nDOFs) {
        throw std::invalid_argument(
            "Expected positive and negative limit vectors of size "
            + std::to_string(ROBOT_nDOFs)
        );
    }
    if (tool_frame.size() != 6) {
        throw std::invalid_argument("Expected 6 tool-frame values");
    }
    std::array<float, 24> dh{};
    for (std::size_t index = 0; index < dh_parameters.size(); ++index) {
        dh[index] = CheckedFloat(dh_parameters[index], "DH parameter");
    }
    std::array<float, ROBOT_nDOFs> positive{};
    std::array<float, ROBOT_nDOFs> negative{};
    for (std::size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
        positive[joint] = CheckedFloat(pos_limits[joint], "Positive joint limit");
        negative[joint] = CheckedFloat(neg_limits[joint], "Negative joint limit");
        if (positive[joint] < 0.0f || negative[joint] < 0.0f) {
            throw std::invalid_argument(
                "Joint limits must be finite, non-negative magnitudes"
            );
        }
    }
    std::array<float, 6> tool{};
    for (std::size_t index = 0; index < tool_frame.size(); ++index) {
        tool[index] = CheckedFloat(tool_frame[index], "Tool-frame value");
    }
    const std::array<float, 6> native_tool_frame = BuildRobotToolFrame(
        tool[0], tool[1], tool[2], tool[3], tool[4], tool[5]
    );

    for (std::size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
        float* link = Robot_Kin_DHM_Table + joint * Table_Size;
        link[DHM_Theta] = dh[joint];
        link[DHM_Alpha] = dh[ROBOT_nDOFs + joint];
        link[DHM_A] = dh[2 * ROBOT_nDOFs + joint];
        link[DHM_D] = dh[3 * ROBOT_nDOFs + joint];
        Robot_JointLimits_Upper[joint] = positive[joint];
        Robot_JointLimits_Lower[joint] = negative[joint];
    }
    ApplyRobotToolFrame(native_tool_frame);
}

std::vector<float> SolveInverseKinematicsRadians(
    const std::vector<double>& target_xyzuvw,
    const std::vector<double>& estimate
) {
    const std::vector<float> target_radians = CheckedMotionVector(
        target_xyzuvw,
        "Expected 6-element xyzuvw input",
        "Kinematics target value"
    );
    const std::vector<float> native_estimate = CheckedMotionVector(
        estimate,
        "Expected 6-element joint estimate",
        "Kinematics estimate value"
    );
    ValidateDegreeToRadianValues(native_estimate, 0, "Joint estimate");

    std::vector<float> target_degrees = target_radians;
    for (std::size_t index = 3; index < ROBOT_nDOFs; ++index) {
        target_degrees[index] = CheckedFloat(
            target_xyzuvw[index] / static_cast<double>(kRadiansPerDegree),
            "Cartesian rotation degrees"
        );
    }
    return SolveInverseKinematicsConfigured(
        target_degrees,
        native_estimate,
        "A"
    );
}

std::vector<float> SolveInverseKinematicsDegrees(
    const std::vector<double>& target_xyzuvw,
    const std::vector<double>& estimate
) {
    const std::vector<float> native_target = CheckedMotionVector(
        target_xyzuvw,
        "Expected 6-element xyzuvw input",
        "Kinematics target value"
    );
    const std::vector<float> native_estimate = CheckedMotionVector(
        estimate,
        "Expected 6-element joint estimate",
        "Kinematics estimate value"
    );
    return SolveInverseKinematics(native_target, native_estimate);
}

std::vector<float> SolveInverseKinematicsConfiguredChecked(
    const std::vector<double>& target_xyzuvw,
    const std::vector<double>& estimate,
    const std::string& wrist_config
) {
    const std::vector<float> native_target = CheckedMotionVector(
        target_xyzuvw,
        "Expected 6-element xyzuvw input",
        "Kinematics target value"
    );
    const std::vector<float> native_estimate = CheckedMotionVector(
        estimate,
        "Expected 6-element joint estimate",
        "Kinematics estimate value"
    );
    return SolveInverseKinematicsConfigured(
        native_target,
        native_estimate,
        wrist_config
    );
}

PYBIND11_MODULE(robot_kinematics, m) {
    m.def("robot_set", &robot_set);
    m.def("robot_data_reset", &robot_data_reset);
    m.def("set_dh_parameters_explicit", &SetDHParametersExplicit);
    m.def("set_joint_limits", &SetJointLimits);
    m.def("set_robot_configuration", &SetRobotConfiguration);

    m.def("forward_kinematics", [](const std::vector<double>& joints) {
        const std::vector<float> native_joints = CheckedMotionVector(
            joints,
            "Expected 6-element joint input",
            "Joint input value"
        );
        ValidateDegreeToRadianValues(native_joints, 0, "Joint input");

        std::vector<float> result(6);
        forward_kinematics_robot_xyzuvw(
            native_joints.data(),
            result.data()
        );
        if (!all_finite(result))
            throw std::runtime_error("Forward kinematics returned non-finite values");
        return result;
    });

    m.def("inverse_kinematics", &SolveInverseKinematicsRadians);

    m.def("inverse_kinematics_no_estimate", [](const std::vector<double>& target_xyzuvw) {
        const std::array<double, ROBOT_nDOFs> estimate{};
        return SolveInverseKinematicsRadians(
            target_xyzuvw,
            std::vector<double>(estimate.begin(), estimate.end())
        );
    });

    m.def("SolveInverseKinematics", &SolveInverseKinematicsDegrees,
      py::arg("xyzuvw_In"), py::arg("JangleIn_in"));

    m.def(
        "SolveInverseKinematicsConfigured",
        &SolveInverseKinematicsConfiguredChecked,
        py::arg("xyzuvw_In"),
        py::arg("JangleIn_in"),
        py::arg("wrist_config")
    );
      

    m.def("get_dh_parameters", []() {
      std::vector<std::vector<float>> out(6, std::vector<float>(4));

        out[0][0] = Robot_Kin_DHM_L1[DHM_Theta];
        out[0][1] = Robot_Kin_DHM_L1[DHM_Alpha];
        out[0][2] = Robot_Kin_DHM_L1[DHM_A];
        out[0][3] = Robot_Kin_DHM_L1[DHM_D];

        out[1][0] = Robot_Kin_DHM_L2[DHM_Theta];
        out[1][1] = Robot_Kin_DHM_L2[DHM_Alpha];
        out[1][2] = Robot_Kin_DHM_L2[DHM_A];
        out[1][3] = Robot_Kin_DHM_L2[DHM_D];

        out[2][0] = Robot_Kin_DHM_L3[DHM_Theta];
        out[2][1] = Robot_Kin_DHM_L3[DHM_Alpha];
        out[2][2] = Robot_Kin_DHM_L3[DHM_A];
        out[2][3] = Robot_Kin_DHM_L3[DHM_D];

        out[3][0] = Robot_Kin_DHM_L4[DHM_Theta];
        out[3][1] = Robot_Kin_DHM_L4[DHM_Alpha];
        out[3][2] = Robot_Kin_DHM_L4[DHM_A];
        out[3][3] = Robot_Kin_DHM_L4[DHM_D];

        out[4][0] = Robot_Kin_DHM_L5[DHM_Theta];
        out[4][1] = Robot_Kin_DHM_L5[DHM_Alpha];
        out[4][2] = Robot_Kin_DHM_L5[DHM_A];
        out[4][3] = Robot_Kin_DHM_L5[DHM_D];

        out[5][0] = Robot_Kin_DHM_L6[DHM_Theta];
        out[5][1] = Robot_Kin_DHM_L6[DHM_Alpha];
        out[5][2] = Robot_Kin_DHM_L6[DHM_A];
        out[5][3] = Robot_Kin_DHM_L6[DHM_D];

        return out;
    });

    m.def("get_joint_limits", []() {
        return std::make_pair(
            std::vector<float>(
                Robot_JointLimits_Upper,
                Robot_JointLimits_Upper + ROBOT_nDOFs
            ),
            std::vector<float>(
                Robot_JointLimits_Lower,
                Robot_JointLimits_Lower + ROBOT_nDOFs
            )
        );
    });

    m.def("set_robot_tool_frame", &SetRobotToolFrame,
      "Set the robot tool frame (x, y, z, rx degrees, ry degrees, rz degrees)");

    m.def("get_robot_tool_frame", &get_robot_tool_frame,
        "Get the robot tool frame (x, y, z, rx degrees, ry degrees, rz degrees)");
}
