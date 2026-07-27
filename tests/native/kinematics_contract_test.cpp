#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../../ARrobots/src/kinematics.cpp"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/cartesian_pose_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/command_queue_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/controller_domain_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/debug_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/home_reference_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/identity_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/motion_command_parse_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/motion_mode_transaction.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/numeric_parse_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/persistence_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/serial_frame_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/spline_response_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/tool_jog_contract.h"
#include "../../ArduinoSketches/AR4_teensy41_sketch_v6.7.1/wrist_selection_contract.h"

namespace {

constexpr float kDegreesPerRadian = 57.295779513082320876f;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Result>
class DirectoryEntryNameStub {
public:
    DirectoryEntryNameStub(
        std::string value,
        bool succeeds = true,
        bool terminates = true
    )
        : value_(std::move(value)),
          succeeds_(succeeds),
          terminates_(terminates) {}

    Result getName(char* name, std::size_t capacity) {
        if (!succeeds_ || name == nullptr || capacity == 0) {
            return static_cast<Result>(0);
        }
        const std::size_t copy_capacity =
            terminates_ ? capacity - 1 : capacity;
        const std::size_t copied = std::min(value_.size(), copy_capacity);
        std::memcpy(name, value_.data(), copied);
        if (terminates_ && copied < capacity) name[copied] = '\0';
        return static_cast<Result>(value_.size());
    }

private:
    std::string value_;
    bool succeeds_;
    bool terminates_;
};

void test_directory_entry_name_contract() {
    char name[ar4_protocol::kControllerFilenameMaxLength + 1] = {};
    std::size_t name_length = 17;
    DirectoryEntryNameStub<bool> flag_result("program.txt");
    require(
        ar4_protocol::read_controller_directory_entry_name(
            flag_result,
            name,
            sizeof(name),
            name_length
        )
            && name_length == std::strlen("program.txt")
            && std::strcmp(name, "program.txt") == 0,
        "boolean directory-name result was treated as a filename length"
    );

    const std::string maximum_name(
        ar4_protocol::kControllerFilenameMaxLength,
        'a'
    );
    DirectoryEntryNameStub<std::size_t> length_result(maximum_name);
    require(
        ar4_protocol::read_controller_directory_entry_name(
            length_result,
            name,
            sizeof(name),
            name_length
        )
            && name_length == maximum_name.length()
            && std::string(name) == maximum_name,
        "length-returning directory-name result was rejected"
    );

    name_length = 17;
    DirectoryEntryNameStub<std::size_t> failed_result("program.txt", false);
    require(
        !ar4_protocol::read_controller_directory_entry_name(
            failed_result,
            name,
            sizeof(name),
            name_length
        )
            && name_length == 17
            && name[0] == '\0',
        "failed directory-name read mutated length or retained data"
    );

    name_length = 17;
    DirectoryEntryNameStub<bool> unterminated_result(
        std::string(sizeof(name), 'a'),
        true,
        false
    );
    require(
        !ar4_protocol::read_controller_directory_entry_name(
            unterminated_result,
            name,
            sizeof(name),
            name_length
        )
            && name_length == 17
            && name[0] == '\0',
        "unterminated directory name crossed the controller boundary"
    );
}

void test_rejected_motion_mode_transaction_atomicity() {
    std::string active_wrist = "N";
    int active_loop_modes[6] = {0, 0, 0, 0, 0, 0};
    const int requested_loop_modes[6] = {1, 1, 1, 1, 1, 1};

    {
        ar4_protocol::MotionModeTransaction<std::string, 6> transaction(
            active_wrist,
            active_loop_modes,
            std::string("F"),
            requested_loop_modes
        );
        require(
            active_wrist == "N"
                && std::all_of(
                    active_loop_modes,
                    active_loop_modes + 6,
                    [](int mode) { return mode == 0; }
                )
                && !transaction.committed(),
            "rejected motion preflight mutated active mode state"
        );
    }
    require(
        active_wrist == "N"
            && std::all_of(
                active_loop_modes,
                active_loop_modes + 6,
                [](int mode) { return mode == 0; }
            ),
        "rejected motion rollback changed active mode state"
    );

    {
        ar4_protocol::MotionModeTransaction<std::string, 6> transaction(
            active_wrist,
            active_loop_modes,
            std::string("F"),
            requested_loop_modes
        );
        transaction.commit();
        transaction.commit();
        require(transaction.committed(), "accepted motion mode commit was lost");
    }
    require(
        active_wrist == "F"
            && std::all_of(
                active_loop_modes,
                active_loop_modes + 6,
                [](int mode) { return mode == 1; }
            ),
        "accepted motion modes were not committed atomically"
    );
}

void test_bounded_serial_frame_accumulator() {
    using ar4_protocol::LiveControlFrameStatus;
    using ar4_protocol::LiveTerminalResponseKind;
    using ar4_protocol::SerialFrameReadStatus;
    using ar4_protocol::StoredRowReadStatus;

    bool discarding = false;
    std::string frame(
        ar4_protocol::kSerialCommandFrameMaximumLength - 1,
        'x'
    );
    require(
        ar4_protocol::append_serial_frame_byte(frame, discarding, '\n')
                == SerialFrameReadStatus::kComplete
            && frame.length()
                == ar4_protocol::kSerialCommandFrameMaximumLength
            && !discarding,
        "maximum-length serial frame was rejected"
    );

    frame.assign(ar4_protocol::kSerialCommandFrameMaximumLength, 'x');
    require(
        ar4_protocol::append_serial_frame_byte(frame, discarding, 'y')
                == SerialFrameReadStatus::kOverflow
            && frame.empty()
            && discarding,
        "oversized unterminated serial frame was retained"
    );
    require(
        ar4_protocol::append_serial_frame_byte(frame, discarding, 'z')
                == SerialFrameReadStatus::kPending
            && frame.empty()
            && discarding,
        "oversized serial frame continued accumulating"
    );
    require(
        ar4_protocol::append_serial_frame_byte(frame, discarding, '\n')
                == SerialFrameReadStatus::kPending
            && frame.empty()
            && !discarding,
        "serial overflow discard did not recover at the line boundary"
    );
    require(
        ar4_protocol::append_serial_frame_byte(frame, discarding, 'S')
                == SerialFrameReadStatus::kPending
            && ar4_protocol::append_serial_frame_byte(
                frame,
                discarding,
                '\n'
            ) == SerialFrameReadStatus::kComplete
            && frame == "S\n",
        "serial frame reader did not recover after overflow"
    );

    std::string stored_row(
        ar4_protocol::kStoredCommandRowMaximumLength,
        'x'
    );
    require(
        ar4_protocol::append_stored_row_byte(stored_row, '\n')
                == StoredRowReadStatus::kComplete
            && stored_row.length()
                == ar4_protocol::kStoredCommandRowMaximumLength,
        "maximum-length stored command row was rejected"
    );
    stored_row.assign(
        ar4_protocol::kStoredCommandRowMaximumLength,
        'x'
    );
    require(
        ar4_protocol::append_stored_row_byte(stored_row, 'y')
                == StoredRowReadStatus::kOverflow
            && stored_row.empty(),
        "oversized stored command row was retained"
    );
    stored_row = "X";
    require(
        ar4_protocol::finish_stored_row(stored_row)
                == StoredRowReadStatus::kComplete
            && ar4_protocol::append_stored_row_byte(stored_row, -1)
                == StoredRowReadStatus::kRejected
            && ar4_protocol::append_stored_row_byte(stored_row, 256)
                == StoredRowReadStatus::kRejected
            && ar4_protocol::finish_stored_row(std::string())
                == StoredRowReadStatus::kRejected,
        "stored command row accepted invalid file input"
    );

    require(
        ar4_protocol::classify_live_control_frame(
            SerialFrameReadStatus::kComplete,
            "S\n",
            2
        ) == LiveControlFrameStatus::kStop
            && ar4_protocol::classify_live_control_frame(
                SerialFrameReadStatus::kComplete,
                "S\r\n",
                3
            ) == LiveControlFrameStatus::kStop
            && ar4_protocol::classify_live_control_frame(
                SerialFrameReadStatus::kComplete,
                "RP\n",
                3
            ) == LiveControlFrameStatus::kRejected
            && ar4_protocol::classify_live_control_frame(
                SerialFrameReadStatus::kOverflow,
                nullptr,
                0
            ) == LiveControlFrameStatus::kRejected,
        "live control frame classification accepted a non-stop frame"
    );
    require(
        ar4_protocol::select_live_terminal_response(
            LiveControlFrameStatus::kStop,
            0,
            0
        ) == LiveTerminalResponseKind::kPosition
            && ar4_protocol::select_live_terminal_response(
                LiveControlFrameStatus::kRejected,
                0,
                0
            ) == LiveTerminalResponseKind::kError
            && ar4_protocol::select_live_terminal_response(
                LiveControlFrameStatus::kStop,
                0,
                1
            ) == LiveTerminalResponseKind::kAxisLimit,
        "live terminal response selection lost single-response ownership"
    );
}

class FirmwareCommandText {
public:
    explicit FirmwareCommandText(std::string text) : text_(std::move(text)) {}

    std::size_t length() const {
        return text_.length();
    }

    FirmwareCommandText substring(int begin, int end) const {
        return FirmwareCommandText(text_.substr(
            static_cast<std::size_t>(begin),
            static_cast<std::size_t>(end - begin)
        ));
    }

    int indexOf(char value) const {
        const std::size_t position = text_.find(value);
        return position == std::string::npos ? -1 : static_cast<int>(position);
    }

    int indexOf(const char* value) const {
        const std::size_t position = text_.find(value);
        return position == std::string::npos ? -1 : static_cast<int>(position);
    }

    int lastIndexOf(char value, int from) const {
        if (from < 0 || text_.empty()) return -1;
        const std::size_t last = std::min(
            static_cast<std::size_t>(from),
            text_.length() - 1
        );
        const std::size_t position = text_.rfind(value, last);
        return position == std::string::npos ? -1 : static_cast<int>(position);
    }

    char charAt(int index) const {
        if (index < 0 || static_cast<std::size_t>(index) >= text_.length()) {
            return '\0';
        }
        return text_[static_cast<std::size_t>(index)];
    }

    const char* c_str() const {
        return text_.c_str();
    }

private:
    std::string text_;
};

class FakeEeprom {
public:
    FakeEeprom() {
        bytes_.fill(0xFF);
    }

    template <typename Value>
    void put(int address, const Value& value) {
        check_range(address, sizeof(Value));
        if (address == failure_address_) {
            ++matching_write_count_;
            if (matching_write_count_ == failure_occurrence_) return;
        }
        const auto* source = reinterpret_cast<const std::uint8_t*>(&value);
        for (std::size_t offset = 0; offset < sizeof(Value); ++offset) {
            const int byte_address = address + static_cast<int>(offset);
            if (byte_address == byte_failure_address_) continue;
            bytes_[static_cast<std::size_t>(byte_address)] = source[offset];
        }
    }

    template <typename Value>
    void get(int address, Value& value) const {
        check_range(address, sizeof(Value));
        std::memcpy(&value, bytes_.data() + address, sizeof(Value));
    }

    void fail_address_write(int address, std::size_t occurrence = 1) {
        failure_address_ = address;
        failure_occurrence_ = occurrence;
        matching_write_count_ = 0;
    }

    void fail_byte_write(int address) {
        byte_failure_address_ = address;
    }

private:
    void check_range(int address, std::size_t size) const {
        if (
            address < 0
            || static_cast<std::size_t>(address) + size > bytes_.size()
        ) {
            throw std::out_of_range("fake EEPROM access exceeded storage");
        }
    }

    std::array<std::uint8_t, 256> bytes_;
    int failure_address_ = -1;
    std::size_t failure_occurrence_ = 0;
    std::size_t matching_write_count_ = 0;
    int byte_failure_address_ = -1;
};

float angular_difference(float left, float right) {
    float difference = std::fmod(left - right, 360.0f);
    if (difference > 180.0f) difference -= 360.0f;
    if (difference < -180.0f) difference += 360.0f;
    return difference;
}

void configure_tracked_defaults() {
    robot_data_reset();

    const float theta_degrees[] = {0.0f, -90.0f, 0.0f, 0.0f, 0.0f, 180.0f};
    const float alpha_degrees[] = {0.0f, -90.0f, 0.0f, -90.0f, 90.0f, -90.0f};
    const float a[] = {0.0f, 64.2f, 305.0f, 0.0f, 0.0f, 0.0f};
    const float d[] = {169.77f, 0.0f, 0.0f, 222.63f, 0.0f, 41.0f};
    const float upper[] = {170.0f, 90.0f, 52.0f, 180.0f, 105.0f, 180.0f};
    const float lower[] = {170.0f, 42.0f, 89.0f, 180.0f, 105.0f, 180.0f};

    for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        float* link = Robot_Kin_DHM_Table + joint * Table_Size;
        link[DHM_Theta] = theta_degrees[joint] / kDegreesPerRadian;
        link[DHM_Alpha] = alpha_degrees[joint] / kDegreesPerRadian;
        link[DHM_A] = a[joint];
        link[DHM_D] = d[joint];
        Robot_JointLimits_Upper[joint] = upper[joint];
        Robot_JointLimits_Lower[joint] = lower[joint];
    }
}

std::vector<float> target_for(const std::vector<float>& joints) {
    require(joints.size() == ROBOT_nDOFs, "test joint vector must contain six values");
    std::vector<float> target(ROBOT_nDOFs);
    forward_kinematics_robot_xyzuvw(joints.data(), target.data());
    for (int index = 3; index < ROBOT_nDOFs; ++index) {
        target[index] *= kDegreesPerRadian;
    }
    return target;
}

void require_pose_match(
    const std::vector<float>& target,
    const std::vector<float>& solution,
    const std::string& label
) {
    require(solution.size() == ROBOT_nDOFs, label + " returned no solution");
    std::array<float, ROBOT_nDOFs> target_radians = {
        target[0],
        target[1],
        target[2],
        target[3] / kDegreesPerRadian,
        target[4] / kDegreesPerRadian,
        target[5] / kDegreesPerRadian,
    };
    Matrix4x4 target_pose;
    Matrix4x4 solution_pose;
    xyzuvw_2_pose(target_radians.data(), target_pose);
    forward_kinematics_robot(solution.data(), solution_pose);

    const float dx = solution_pose[12] - target_pose[12];
    const float dy = solution_pose[13] - target_pose[13];
    const float dz = solution_pose[14] - target_pose[14];
    require(
        std::sqrt(dx * dx + dy * dy + dz * dz) <= 0.1f,
        label + " exceeded Cartesian position tolerance"
    );
    for (int index : {0, 1, 2, 4, 5, 6, 8, 9, 10}) {
        require(
            std::fabs(solution_pose[index] - target_pose[index]) <= 0.002f,
            label + " exceeded Cartesian rotation tolerance"
        );
    }
}

template <typename Callback>
void require_invalid_argument(Callback callback, const std::string& label) {
    try {
        callback();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(label + " did not reject invalid input");
}

void test_boundary_validation() {
    configure_tracked_defaults();
    const std::vector<float> valid(ROBOT_nDOFs, 0.0f);
    require_invalid_argument(
        [&]() { SolveInverseKinematicsConfigured({0.0f}, valid, "A"); },
        "short target"
    );
    require_invalid_argument(
        [&]() { SolveInverseKinematicsConfigured(valid, {0.0f}, "A"); },
        "short estimate"
    );
    std::vector<float> non_finite = valid;
    non_finite[2] = std::numeric_limits<float>::quiet_NaN();
    require_invalid_argument(
        [&]() { SolveInverseKinematicsConfigured(non_finite, valid, "A"); },
        "non-finite target"
    );
    require_invalid_argument(
        [&]() { SolveInverseKinematicsConfigured(valid, non_finite, "A"); },
        "non-finite estimate"
    );
    require_invalid_argument(
        [&]() { SolveInverseKinematicsConfigured(valid, valid, "X"); },
        "unknown wrist configuration"
    );
    std::vector<float> derived_underflow = valid;
    derived_underflow[3] = std::numeric_limits<float>::denorm_min();
    require_invalid_argument(
        [&]() {
            SolveInverseKinematicsConfigured(
                derived_underflow,
                valid,
                "A"
            );
        },
        "Cartesian rotation underflow"
    );
    derived_underflow = valid;
    derived_underflow[0] = std::numeric_limits<float>::denorm_min();
    require_invalid_argument(
        [&]() {
            SolveInverseKinematics(
                valid,
                derived_underflow
            );
        },
        "joint-estimate underflow"
    );

    const float maximum = std::numeric_limits<float>::max();
    const float large_wrist_difference = ar4_protocol::wrist_angular_difference(
        maximum,
        -maximum
    );
    const float large_wrist_sum = ar4_protocol::wrist_angular_sum(
        maximum,
        maximum
    );
    require(
        std::isfinite(large_wrist_difference)
        && std::fabs(large_wrist_difference) <= 180.0f
        && std::isfinite(large_wrist_sum)
        && std::fabs(large_wrist_sum) <= 180.0f,
        "native large finite wrist arithmetic did not normalize"
    );

    robot_data_reset();
    for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        require(
            Robot_JointLimits_Upper[joint] == 180.0f
            && Robot_JointLimits_Lower[joint] == 180.0f,
            "reset joint limits must remain positive magnitudes"
        );
    }
}

void require_matrix_match(
    const Matrix4x4 actual,
    const std::array<float, 16>& expected,
    const std::string& label
) {
    for (int index = 0; index < 16; ++index) {
        require(
            std::fabs(actual[index] - expected[index]) <= 0.00001f,
            label + " produced the wrong rotation matrix"
        );
    }
}

void configure_identity_kinematic_chain() {
    robot_data_reset();
    for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        float* link = Robot_Kin_DHM_Table + joint * Table_Size;
        link[DHM_Theta] = 0.0f;
        link[DHM_Alpha] = 0.0f;
        link[DHM_A] = 0.0f;
        link[DHM_D] = 0.0f;
    }
}

void test_cartesian_pose_order_contract() {
    const float external[ar4_protocol::kCartesianPoseSize] = {
        1.0f,
        2.0f,
        3.0f,
        4.0f,
        5.0f,
        6.0f,
    };
    float native[ar4_protocol::kCartesianPoseSize] = {};
    require(
        ar4_protocol::external_cartesian_pose_to_native(external, native),
        "valid external Cartesian pose was rejected"
    );
    const std::array<float, 6> expected_native = {1, 2, 3, 6, 5, 4};
    for (int index = 0; index < ar4_protocol::kCartesianPoseSize; ++index) {
        require(
            native[index] == expected_native[index],
            "external Cartesian rotation order entered the wrong native slot"
        );
    }

    float restored[ar4_protocol::kCartesianPoseSize] = {};
    require(
        ar4_protocol::native_cartesian_pose_to_external(native, restored),
        "valid native Cartesian pose was rejected"
    );
    for (int index = 0; index < ar4_protocol::kCartesianPoseSize; ++index) {
        require(
            restored[index] == external[index],
            "native Cartesian rotation order entered the wrong external slot"
        );
    }

    float invalid[ar4_protocol::kCartesianPoseSize] = {
        1.0f,
        2.0f,
        3.0f,
        4.0f,
        5.0f,
        std::numeric_limits<float>::quiet_NaN(),
    };
    float unchanged[ar4_protocol::kCartesianPoseSize] = {
        7.0f,
        7.0f,
        7.0f,
        7.0f,
        7.0f,
        7.0f,
    };
    require(
        !ar4_protocol::external_cartesian_pose_to_native(invalid, unchanged),
        "non-finite external Cartesian pose was accepted"
    );
    for (float value : unchanged) {
        require(value == 7.0f, "rejected Cartesian pose mutated the output");
    }

    float native_radians[ar4_protocol::kCartesianPoseSize] = {};
    require(
        ar4_protocol::external_cartesian_pose_to_native_radians(
            external,
            native_radians
        ),
        "valid external Cartesian radians conversion was rejected"
    );
    const std::array<float, 6> expected_native_radians = {
        1.0f,
        2.0f,
        3.0f,
        6.0f * kRadiansPerDegree,
        5.0f * kRadiansPerDegree,
        4.0f * kRadiansPerDegree,
    };
    for (int index = 0; index < ar4_protocol::kCartesianPoseSize; ++index) {
        require(
            std::fabs(native_radians[index] - expected_native_radians[index])
                <= 0.000001f,
            "external Cartesian radians conversion changed axis order"
        );
    }

    float underflow[ar4_protocol::kCartesianPoseSize] = {};
    underflow[3] = std::numeric_limits<float>::denorm_min();
    float preserved[ar4_protocol::kCartesianPoseSize] = {
        9.0f,
        9.0f,
        9.0f,
        9.0f,
        9.0f,
        9.0f,
    };
    require(
        !ar4_protocol::external_cartesian_pose_to_native_radians(
            underflow,
            preserved
        ),
        "Cartesian degree-to-radian underflow was accepted"
    );
    for (float value : preserved) {
        require(
            value == 9.0f,
            "rejected Cartesian radians conversion mutated the output"
        );
    }
}

void test_firmware_numeric_parse_contract() {
    float parsed_float = 17.0f;
    require(
        ar4_protocol::parse_finite_decimal_float("-1250.5", parsed_float)
            && parsed_float == -1250.5f,
        "valid firmware float text was rejected"
    );
    require(
        ar4_protocol::parse_finite_decimal_float(
            "0.0000000000000000000000000000000000000000000000",
            parsed_float
        )
            && parsed_float == 0.0f,
        "exact zero with a long fraction was rejected"
    );

    const std::array<const char*, 15> invalid_floats = {{
        "",
        " ",
        "value",
        "1x",
        "1 ",
        "nan",
        "inf",
        "0x1p2",
        "+",
        "+1",
        ".",
        "1e2",
        "0e-9999",
        "0.0000000000000000000000000000000000000000000001",
        "350000000000000000000000000000000000000",
    }};
    for (const char* text : invalid_floats) {
        parsed_float = 17.0f;
        require(
            !ar4_protocol::parse_finite_decimal_float(text, parsed_float)
                && parsed_float == 17.0f,
            std::string("invalid firmware float text mutated output: ") + text
        );
    }

    int parsed_int = 17;
    require(
        ar4_protocol::parse_decimal_int("-2147483648", parsed_int)
            && parsed_int == std::numeric_limits<int>::min(),
        "minimum firmware integer text was rejected"
    );
    require(
        ar4_protocol::parse_decimal_int("2147483647", parsed_int)
            && parsed_int == std::numeric_limits<int>::max(),
        "maximum firmware integer text was rejected"
    );
    const std::array<const char*, 8> invalid_ints = {{
        "",
        "1.0",
        "1x",
        " 1",
        "1 ",
        "+1",
        "2147483648",
        "-2147483649",
    }};
    for (const char* text : invalid_ints) {
        parsed_int = 17;
        require(
            !ar4_protocol::parse_decimal_int(text, parsed_int)
                && parsed_int == 17,
            std::string("invalid firmware integer text mutated output: ") + text
        );
    }

    const int binary_values[] = {0, 1, 1, 0};
    const int nonbinary_values[] = {0, 2, 1};
    require(
        ar4_protocol::values_are_binary(binary_values)
            && !ar4_protocol::values_are_binary(nonbinary_values)
            && !ar4_protocol::values_are_binary(nonbinary_values + 1, 2)
            && !ar4_protocol::values_are_binary(nullptr, 1),
        "binary firmware control validation accepted an invalid value"
    );

    ar4_protocol::ToolJogCommandFields tool_jog = {};
    require(
        ar4_protocol::parse_tool_jog_command(
            FirmwareCommandText(
                "X10.25Sp25.5G10.5H11.5I12.5WALm010101"
            ),
            tool_jog
        ),
        "canonical JT command with decimal timing was rejected"
    );
    require(
        tool_jog.axis == 'X'
            && tool_jog.direction == 1
            && tool_jog.distance == 0.25f
            && tool_jog.speed_mode == 'p'
            && tool_jog.speed == 25.5f
            && tool_jog.acceleration == 10.5f
            && tool_jog.deceleration == 11.5f
            && tool_jog.ramp == 12.5f
            && tool_jog.wrist_config == 'A'
            && tool_jog.loop_modes[0] == 0
            && tool_jog.loop_modes[5] == 1,
        "canonical JT command fields changed during firmware parsing"
    );

    const std::array<const char*, 11> invalid_tool_jogs = {{
        "X10.25Sp25.5GvalueH11.5I12.5WALm010101",
        "X20.25Sp25.5G10.5H11.5I12.5WALm010101",
        "X1-0.25Sp25.5G10.5H11.5I12.5WALm010101",
        "X10.25Sp0G10.5H11.5I12.5WALm010101",
        "X10.25Sp25.5G90H20I12.5WALm010101",
        "X10.25Sp25.5G10.5H11.5I0WALm010101",
        "junkX10.25Sp25.5G10.5H11.5I12.5WALm010101",
        "X10.25Sp25.5G10.5H11.5I12.5WQLm010101",
        "X10.25Sp25.5G10.5H11.5I12.5WALm010102",
        "X10.25Sp25.5G10.5H11.5I12.5WALm01010",
        "X10.25Sp25.5G10.5H11.5I12.5WALm010101junk",
    }};
    for (const char* text : invalid_tool_jogs) {
        tool_jog.speed = 17.0f;
        require(
            !ar4_protocol::parse_tool_jog_command(
                FirmwareCommandText(text),
                tool_jog
            )
                && tool_jog.speed == 17.0f,
            std::string("invalid JT command mutated staged fields: ") + text
        );
    }

    ar4_protocol::CartesianMoveCommandFields cartesian = {};
    const FirmwareCommandText canonical_mj(
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25.5Ac10.5Dc11.5Rm12.5"
        "WALm010101"
    );
    require(
        ar4_protocol::parse_cartesian_move_command(canonical_mj, cartesian),
        "canonical no-rounding MJ command was rejected"
    );
    require(
        cartesian.pose[0] == 1.0f
            && cartesian.pose[5] == 6.0f
            && cartesian.auxiliary[2] == 0.0f
            && cartesian.speed == 25.5f
            && cartesian.acceleration == 10.5f
            && cartesian.deceleration == 11.5f
            && cartesian.ramp == 12.5f
            && cartesian.rounding == 0.0f
            && cartesian.wrist_config == 'A'
            && cartesian.loop_modes[5] == 1,
        "canonical MJ command fields changed during firmware parsing"
    );
    cartesian.rounding = 17.0f;
    require(
        !ar4_protocol::parse_cartesian_move_command(
            FirmwareCommandText(
                "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10"
                "Rnd2.5WFLm111111"
            ),
            cartesian
        )
            && cartesian.rounding == 17.0f,
        "unsupported MJ rounding was accepted or mutated staged fields"
    );
    const std::array<const char*, 6> invalid_cartesian_moves = {{
        "junkX1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp0Ac10Dc10Rm10WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac90Dc20Rm10WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm0WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10Rnd-1WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sq25Ac10Dc10Rm10WALm000000",
    }};
    for (const char* text : invalid_cartesian_moves) {
        cartesian.speed = 17.0f;
        require(
            !ar4_protocol::parse_cartesian_move_command(
                FirmwareCommandText(text),
                cartesian
            )
                && cartesian.speed == 17.0f,
            std::string("invalid Cartesian command mutated staged fields: ")
                + text
        );
    }

    require(
        ar4_protocol::parse_linear_move_command(
            FirmwareCommandText(
                "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10"
                "Rnd2WALm010101Q0"
            ),
            cartesian
        )
            && cartesian.rounding == 2.0f,
        "canonical ML command was rejected"
    );
    for (const char wrist_config : std::array<char, 2>{{'N', 'F'}}) {
        const std::string rounded_command =
            "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10Rnd2W"
            + std::string(1, wrist_config)
            + "Lm010101Q0";
        require(
            ar4_protocol::parse_linear_move_command(
                FirmwareCommandText(rounded_command),
                cartesian
            )
                && cartesian.rounding == 2.0f
                && cartesian.wrist_config == wrist_config,
            std::string("rounded ML lost wrist configuration: ")
                + wrist_config
        );
    }
    require(
        !ar4_protocol::parse_linear_move_command(
            FirmwareCommandText(
                "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10"
                "Rnd2WALm010101Q1"
            ),
            cartesian
        ),
        "unsupported ML wrist-disable flag was accepted"
    );

    ar4_protocol::JointMoveCommandFields joint_move = {};
    require(
        ar4_protocol::parse_joint_move_command(
            FirmwareCommandText(
                "A1B2C3D4E5F6J70J80J90Sp25Ac10Dc10Rm10WALm010101"
            ),
            joint_move
        )
            && joint_move.positions[0] == 1.0f
            && joint_move.positions[8] == 0.0f
            && joint_move.speed == 25.0f,
        "canonical RJ command was rejected"
    );
    require(
        !ar4_protocol::parse_joint_move_command(
            FirmwareCommandText(
                "junkA1B2C3D4E5F6J70J80J90Sp25Ac10Dc10Rm10WALm010101"
            ),
            joint_move
        ),
        "prefixed RJ command was accepted"
    );

    ar4_protocol::LiveJogCommandFields live_jog = {};
    require(
        ar4_protocol::parse_live_jog_command(
            FirmwareCommandText("V91Sp25Ac10Dc10Rm10WALm010101"),
            ar4_protocol::LiveJogCommandKind::kJoint,
            live_jog
        )
            && live_jog.vector == 91
            && live_jog.speed_mode == 'p'
            && live_jog.speed == 25.0f
            && live_jog.acceleration == 10.0f
            && live_jog.deceleration == 10.0f
            && live_jog.ramp == 10.0f
            && ar4_protocol::valid_motion_profile(
                live_jog.speed_mode,
                live_jog.speed,
                live_jog.acceleration,
                live_jog.deceleration,
                live_jog.ramp
            ),
        "canonical live-jog command was rejected"
    );
    require(
        ar4_protocol::parse_live_jog_command(
            FirmwareCommandText("V11Sp25Ac10Dc10Rm10WNLm010101"),
            ar4_protocol::LiveJogCommandKind::kCartesian,
            live_jog
        )
            && ar4_protocol::parse_live_jog_command(
                FirmwareCommandText("V61Sp25Ac10Dc10Rm10WFLm010101"),
                ar4_protocol::LiveJogCommandKind::kTool,
                live_jog
            ),
        "canonical Cartesian or tool live-jog command was rejected"
    );
    const char* unsupported_live_modes[] = {
        "V11Ss25Ac10Dc10Rm10WALm010101",
        "V11Sm25Ac10Dc10Rm10WALm010101",
    };
    const ar4_protocol::LiveJogCommandKind live_kinds[] = {
        ar4_protocol::LiveJogCommandKind::kCartesian,
        ar4_protocol::LiveJogCommandKind::kJoint,
        ar4_protocol::LiveJogCommandKind::kTool,
    };
    for (const char* text : unsupported_live_modes) {
        for (const auto kind : live_kinds) {
            live_jog.speed = 17.0f;
            require(
                !ar4_protocol::parse_live_jog_command(
                    FirmwareCommandText(text),
                    kind,
                    live_jog
                )
                    && live_jog.speed == 17.0f,
                std::string("unsupported live-jog mode mutated staged fields: ")
                    + text
            );
        }
    }
    require(
        !ar4_protocol::parse_live_jog_command(
            FirmwareCommandText("V91Sp25Ac10Dc10Rm10WALm010101"),
            ar4_protocol::LiveJogCommandKind::kCartesian,
            live_jog
        )
            && !ar4_protocol::parse_live_jog_command(
                FirmwareCommandText("junkV11Sp25Ac10Dc10Rm10WALm010101"),
                ar4_protocol::LiveJogCommandKind::kCartesian,
                live_jog
            )
            && !ar4_protocol::parse_live_jog_command(
                FirmwareCommandText("V11Sp25Ac60Dc50Rm10WALm010101"),
                ar4_protocol::LiveJogCommandKind::kCartesian,
                live_jog
            )
            && !ar4_protocol::parse_live_jog_command(
                FirmwareCommandText("V11Sp25Ac10Dc10Rm10WNLm010101"),
                ar4_protocol::LiveJogCommandKind::kJoint,
                live_jog
            )
            && !ar4_protocol::parse_live_jog_command(
                FirmwareCommandText("V11Sp25Ac10Dc10Rm10WFLm010101"),
                ar4_protocol::LiveJogCommandKind::kJoint,
                live_jog
            ),
        "invalid live-jog command was accepted"
    );

    const FirmwareCommandText canonical_mv(
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25.5Ac10.5Dc11.5Rm12.5"
        "WNVr-12.5Lm001100"
    );
    require(
        ar4_protocol::parse_vision_move_command(canonical_mv, cartesian),
        "canonical no-rounding MV command was rejected"
    );
    require(
        cartesian.rounding == 0.0f
            && cartesian.vision_rotation_degrees == -12.5f
            && cartesian.wrist_config == 'N'
            && cartesian.loop_modes[2] == 1
            && cartesian.loop_modes[3] == 1,
        "canonical MV command fields changed during firmware parsing"
    );
    cartesian.rounding = 17.0f;
    require(
        !ar4_protocol::parse_vision_move_command(
            FirmwareCommandText(
                "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10"
                "Rnd3WAVr2.5Lm000000"
            ),
            cartesian
        )
            && cartesian.rounding == 17.0f,
        "unsupported MV rounding was accepted or mutated staged fields"
    );

    const std::array<const char*, 5> invalid_vision_moves = {{
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WALm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WAVrvalueLm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WQVr1Lm000000",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WAVr1Lm000002",
        "X1Y2Z3Rz4Ry5Rx6J70J80J90Sp25Ac10Dc10Rm10WAVr1Lm000000junk",
    }};
    for (const char* text : invalid_vision_moves) {
        cartesian.speed = 17.0f;
        require(
            !ar4_protocol::parse_vision_move_command(
                FirmwareCommandText(text),
                cartesian
            )
                && cartesian.speed == 17.0f,
            std::string("invalid MV command mutated staged fields: ") + text
        );
    }

    const FirmwareCommandText marker_command("A1BvalueC");
    const int marker_positions[] = {0, 2, 8};
    float marker_fields[] = {17.0f, 17.0f};
    require(
        !ar4_protocol::parse_float_marker_fields(
            marker_command,
            marker_positions,
            marker_fields
        )
            && marker_fields[0] == 17.0f
            && marker_fields[1] == 17.0f,
        "rejected marker fields partially mutated the staged output"
    );
}

void test_controller_domain_contract() {
    int release_step_limit = 17;
    require(
        ar4_protocol::calibration_release_step_limit(
            88.888f,
            10.0f,
            3000,
            release_step_limit
        )
            && release_step_limit == 889,
        "valid calibration switch-release travel was rejected"
    );
    release_step_limit = 17;
    require(
        ar4_protocol::calibration_release_step_limit(
            100.0f,
            10.0f,
            500,
            release_step_limit
        )
            && release_step_limit == 500,
        "calibration switch-release travel exceeded full axis travel"
    );
    release_step_limit = 17;
    require(
        !ar4_protocol::calibration_release_step_limit(
            0.0f,
            10.0f,
            3000,
            release_step_limit
        )
            && release_step_limit == 17
            && !ar4_protocol::calibration_release_step_limit(
                100.0f,
                0.0f,
                3000,
                release_step_limit
            )
            && release_step_limit == 17
            && !ar4_protocol::calibration_release_step_limit(
                100.0f,
                10.0f,
                0,
                release_step_limit
            )
            && release_step_limit == 17
            && !ar4_protocol::calibration_release_step_limit(
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                INT_MAX,
                release_step_limit
            )
            && release_step_limit == 17,
        "invalid calibration switch-release travel was accepted"
    );

    int step_limit = 17;
    int zero_step = 19;
    require(
        ar4_protocol::validate_axis_calibration(
            10.0f,
            20.0f,
            100.0f,
            step_limit,
            zero_step
        )
            && step_limit == 3000
            && zero_step == 1000,
        "valid controller-axis calibration was rejected"
    );
    step_limit = 17;
    zero_step = 19;
    require(
        !ar4_protocol::validate_axis_calibration(
            -1.0f,
            20.0f,
            100.0f,
            step_limit,
            zero_step
        )
            && step_limit == 17
            && zero_step == 19
            && !ar4_protocol::validate_axis_calibration(
                10.0f,
                20.0f,
                0.0f,
                step_limit,
                zero_step
            )
            && !ar4_protocol::validate_axis_calibration(
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                step_limit,
                zero_step
            ),
        "invalid controller-axis calibration was accepted or mutated output"
    );

    int future_step = 17;
    require(
        ar4_protocol::calibrated_position_to_step(
            20.0f,
            10.0f,
            20.0f,
            100.0f,
            3000,
            future_step
        )
            && future_step == 3000,
        "valid calibrated position was rejected"
    );
    future_step = 17;
    require(
        !ar4_protocol::calibrated_position_to_step(
            20.1f,
            10.0f,
            20.0f,
            100.0f,
            3000,
            future_step
        )
            && future_step == 17
            && !ar4_protocol::calibrated_position_to_step(
                0.0f,
                10.0f,
                20.0f,
                100.0f,
                2999,
                future_step
            ),
        "invalid calibrated position was accepted or mutated output"
    );

    ar4_protocol::ExternalAxisCalibration external = {};
    require(
        ar4_protocol::validate_external_axis_calibration(
            100.0f,
            360.0f,
            3600.0f,
            external
        )
            && external.positive_limit == 100.0f
            && external.steps_per_unit == 10.0f
            && external.step_limit == 1000
            && !ar4_protocol::validate_external_axis_calibration(
                100.0f,
                0.0f,
                3600.0f,
                external
            ),
        "external-axis calibration domain changed"
    );

    int master_step = 17;
    int center_step = 19;
    int joint_five_step = 23;
    require(
        ar4_protocol::calibration_reference_steps(
            1,
            0,
            20.0f,
            10.0f,
            100.0f,
            3000,
            0.0f,
            0.0f,
            false,
            master_step,
            center_step,
            joint_five_step
        )
            && master_step == 0
            && center_step == 1000
            && joint_five_step == 0,
        "valid calibration reference was rejected"
    );
    master_step = 17;
    center_step = 19;
    joint_five_step = 23;
    require(
        !ar4_protocol::calibration_reference_steps(
            1,
            0,
            20.0f,
            10.0f,
            100.0f,
            3000,
            std::numeric_limits<float>::max(),
            0.0f,
            false,
            master_step,
            center_step,
            joint_five_step
        )
            && master_step == 17
            && center_step == 19
            && joint_five_step == 23,
        "invalid calibration reference was accepted or mutated output"
    );
    require(
        !ar4_protocol::calibration_reference_steps(
            1,
            0,
            0.0f,
            0.0f,
            100.0f,
            0,
            0.0f,
            0.0f,
            false,
            master_step,
            center_step,
            joint_five_step
        ),
        "requested zero-travel calibration reference was accepted"
    );

    std::uint32_t milliseconds = 17;
    require(
        ar4_protocol::wait_seconds_to_milliseconds(2.5f, milliseconds)
            && milliseconds == 2500U
            && ar4_protocol::wait_seconds_to_milliseconds(
                ar4_protocol::kMainFirmwareWaitMaxSeconds,
                milliseconds
            )
            && milliseconds == 2147483000U,
        "valid controller wait was rejected"
    );
    milliseconds = 17;
    require(
        !ar4_protocol::wait_seconds_to_milliseconds(-0.1f, milliseconds)
            && milliseconds == 17U
            && !ar4_protocol::wait_seconds_to_milliseconds(
                ar4_protocol::kMainFirmwareWaitMaxSeconds + 1,
                milliseconds
            ),
        "invalid controller wait was accepted or mutated output"
    );

    using ar4_protocol::ModbusOperation;
    require(
        ar4_protocol::validate_modbus_request(
            ModbusOperation::kReadHoldingRegisters,
            1,
            0,
            64
        )
            && ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadHoldingRegisters,
                1,
                ar4_protocol::kModbusMaximumAddress,
                1
            )
            && ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadInputRegisters,
                1,
                ar4_protocol::kModbusMaximumAddress
                    - ar4_protocol::kModbusMaximumRegisterReadQuantity + 1,
                ar4_protocol::kModbusMaximumRegisterReadQuantity
            )
            && ar4_protocol::validate_modbus_request(
                ModbusOperation::kWriteRegister,
                247,
                65535,
                65535
            )
            && !ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadHoldingRegisters,
                0,
                0,
                1
            )
            && !ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadHoldingRegisters,
                1,
                0,
                65
            )
            && !ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadHoldingRegisters,
                1,
                ar4_protocol::kModbusMaximumAddress,
                2
            )
            && !ar4_protocol::validate_modbus_request(
                ModbusOperation::kReadInputRegisters,
                1,
                ar4_protocol::kModbusMaximumAddress
                    - ar4_protocol::kModbusMaximumRegisterReadQuantity + 2,
                ar4_protocol::kModbusMaximumRegisterReadQuantity
            )
            && !ar4_protocol::validate_modbus_request(
                ModbusOperation::kWriteCoil,
                1,
                0,
                2
            ),
        "controller I/O domain accepted an unsafe value"
    );
    milliseconds = 17;
    require(
        ar4_protocol::validate_modbus_wait(
            ModbusOperation::kReadCoil,
            1,
            0,
            1,
            2,
            milliseconds
        )
            && milliseconds == 2000U
            && !ar4_protocol::validate_modbus_wait(
                ModbusOperation::kWriteCoil,
                1,
                0,
                1,
                2,
                milliseconds
            )
            && !ar4_protocol::validate_modbus_wait(
                ModbusOperation::kReadCoil,
                1,
                0,
                2,
                2,
                milliseconds
            ),
        "Modbus wait domain changed"
    );
    milliseconds = 17;
    require(
        !ar4_protocol::validate_modbus_wait(
            ModbusOperation::kReadCoil,
            1,
            0,
            1,
            0,
            milliseconds
        )
            && milliseconds == 17U,
        "zero-second Modbus wait skipped polling instead of rejecting"
    );

    const std::string maximum_filename(
        ar4_protocol::kControllerFilenameMaxLength,
        'a'
    );
    const std::string oversized_filename(
        ar4_protocol::kControllerFilenameMaxLength + 1,
        'a'
    );
    const std::string non_ascii_filename = std::string("program")
        + static_cast<char>(0x80);
    require(
        ar4_protocol::valid_controller_filename(
            FirmwareCommandText("program.ar4"),
            0,
            11
        )
            && ar4_protocol::valid_controller_filename(
                FirmwareCommandText(maximum_filename),
                0,
                static_cast<int>(maximum_filename.length())
            )
            && !ar4_protocol::valid_controller_filename(
                FirmwareCommandText("../program"),
                0,
                10
            )
            && !ar4_protocol::valid_controller_filename(
                FirmwareCommandText("folder/program"),
                0,
                14
            )
            && !ar4_protocol::valid_controller_filename(
                FirmwareCommandText(oversized_filename),
                0,
                static_cast<int>(oversized_filename.length())
            )
            && !ar4_protocol::valid_controller_filename(
                FirmwareCommandText(non_ascii_filename),
                0,
                static_cast<int>(non_ascii_filename.length())
            ),
        "controller filename domain changed"
    );
    const std::array<char, 9> fat_reserved_characters = {{
        '"', '*', '/', ':', '<', '>', '?', '\\', '|',
    }};
    for (const char reserved : fat_reserved_characters) {
        const std::string filename = std::string("program") + reserved + ".ar4";
        require(
            !ar4_protocol::valid_controller_filename(
                FirmwareCommandText(filename),
                0,
                static_cast<int>(filename.length())
            ),
            std::string("FAT-reserved filename character was accepted: ")
                + reserved
        );
    }
    const std::string separator_filename =
        std::string("program")
        + ar4_protocol::kControllerDirectorySeparator
        + ".ar4";
    require(
        !ar4_protocol::valid_controller_filename(
            FirmwareCommandText(separator_filename),
            0,
            static_cast<int>(separator_filename.length())
        ),
        "controller directory separator was accepted in a filename"
    );
    for (const char* filename : {" program.txt", "program.txt "}) {
        require(
            !ar4_protocol::valid_controller_filename(
                FirmwareCommandText(filename),
                0,
                static_cast<int>(std::string(filename).length())
            ),
            "outer-space controller filename was accepted"
        );
    }
    require(
        ar4_protocol::valid_controller_filename(
            FirmwareCommandText("program .txt"),
            0,
            12
        ),
        "pre-extension space could not round-trip through storage"
    );
    require(
        ar4_protocol::controller_filenames_equal_ignore_case(
            "Program.TXT",
            "program.txt"
        )
            && ar4_protocol::controller_filenames_equal_ignore_case(
                "A1 -_.txt",
                "a1 -_.TXT"
            )
            && !ar4_protocol::controller_filenames_equal_ignore_case(
                "program.txt",
                "program2.txt"
            )
            && !ar4_protocol::controller_filenames_equal_ignore_case(
                "program.txt",
                "program.txt.bak"
            )
            && !ar4_protocol::controller_filenames_equal_ignore_case(
                nullptr,
                "program.txt"
            ),
        "controller filename identity comparison changed"
    );
    for (const char* filename : {"program.txt", "program .txt", "second.nc"}) {
        const FirmwareCommandText entry(filename);
        require(
            ar4_protocol::valid_controller_directory_entry_filename(
                entry,
                0,
                static_cast<int>(entry.length())
            ),
            "valid controller directory entry was rejected"
        );
    }
    for (const char* filename : {".txt", "..txt", "...txt"}) {
        const FirmwareCommandText entry(filename);
        require(
            !ar4_protocol::valid_controller_directory_entry_filename(
                entry,
                0,
                static_cast<int>(entry.length())
            ),
            "non-reversible G-code directory entry was accepted"
        );
    }
    require(
        ar4_protocol::kControllerDirectoryPayloadMaxLength == 4096
            && ar4_protocol::kControllerDirectoryIdentityPrefixLength == 37
            && ar4_protocol::controller_directory_entry_fits_payload(
                3840,
                255
            )
            && !ar4_protocol::controller_directory_entry_fits_payload(
                3841,
                255
            )
            && !ar4_protocol::controller_directory_entry_fits_payload(0, 0)
            && !ar4_protocol::controller_directory_entry_fits_payload(
                ar4_protocol::kControllerDirectoryPayloadMaxLength,
                1
            ),
        "controller directory payload boundary changed"
    );
    std::string maximum_directory_payload;
    for (int index = 0; index < 16; ++index) {
        const std::string prefix = std::to_string(1000 + index);
        const std::string filename = prefix + std::string(251, 'a');
        require(
            ar4_protocol::controller_directory_entry_fits_payload(
                maximum_directory_payload.length(),
                filename.length()
            ),
            "maximum controller directory payload rejected a valid entry"
        );
        maximum_directory_payload += filename;
        maximum_directory_payload +=
            ar4_protocol::kControllerDirectorySeparator;
    }
    require(
        maximum_directory_payload.length()
                == ar4_protocol::kControllerDirectoryPayloadMaxLength
            && !ar4_protocol::controller_directory_entry_fits_payload(
                maximum_directory_payload.length(),
                1
            ),
        "controller directory payload overflow was accepted"
    );

    const std::array<std::uint8_t, 16> cid_bytes = {{
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0xFF,
    }};
    char media_id[ar4_protocol::kControllerMediaIdCapacity] = {0};
    require(
        ar4_protocol::format_controller_media_id(
            cid_bytes.data(),
            cid_bytes.size(),
            media_id,
            sizeof(media_id)
        )
            && std::string(media_id)
                == "000102030405060708090A0B0C0D0EFF"
            && ar4_protocol::valid_controller_media_id(
                FirmwareCommandText(media_id),
                0,
                static_cast<int>(ar4_protocol::kControllerMediaIdLength)
            ),
        "SD CID media identity was not encoded canonically"
    );
    int storage_filename_begin = 17;
    const FirmwareCommandText storage_target(
        std::string("Mi") + media_id + "Fndemo.txt"
    );
    require(
        ar4_protocol::parse_gcode_media_filename_suffix(
            storage_target,
            storage_filename_begin
        )
            && storage_filename_begin == 36,
        "media-bound G-code filename suffix was rejected"
    );
    storage_filename_begin = 17;
    require(
        !ar4_protocol::parse_gcode_media_filename_suffix(
            FirmwareCommandText(
                "Mi000102030405060708090a0B0C0D0EFFFndemo.txt"
            ),
            storage_filename_begin
        )
            && storage_filename_begin == 17
            && !ar4_protocol::format_controller_media_id(
                cid_bytes.data(),
                cid_bytes.size() - 1,
                media_id,
                sizeof(media_id)
            ),
        "invalid G-code storage identity was accepted or mutated output"
    );

    future_step = 17;
    require(
        ar4_protocol::stored_step_target(50, 25, 1, 100, future_step)
            && future_step == 75
            && ar4_protocol::stored_step_target(50, 25, 0, 100, future_step)
            && future_step == 25
            && !ar4_protocol::stored_step_target(
                50,
                -1,
                1,
                100,
                future_step
            )
            && !ar4_protocol::stored_step_target(
                90,
                20,
                1,
                100,
                future_step
            ),
        "stored step-target domain changed"
    );

    require(
        ar4_protocol::valid_delay_envelope(
            60.0,
            120.0,
            120.0,
            true,
            90.0
        )
            && !ar4_protocol::valid_delay_envelope(
                60.0,
                ar4_protocol::kMaximumPulseDelayMicroseconds + 1.0,
                120.0,
                false,
                0.0
            )
            && !ar4_protocol::valid_delay_envelope(
                60.0,
                120.0,
                120.0,
                true,
                std::numeric_limits<double>::infinity()
            ),
        "controller delay envelope changed"
    );
    std::uint32_t pulse_delay = 17;
    require(
        ar4_protocol::pulse_delay_microseconds(
            100.0,
            30.0,
            60.0,
            pulse_delay
        )
            && pulse_delay == 70U
            && ar4_protocol::pulse_delay_microseconds(
                70.0,
                30.0,
                60.0,
                pulse_delay
            )
            && pulse_delay == 60U,
        "valid controller pulse delay was rejected"
    );
    pulse_delay = 17;
    require(
        !ar4_protocol::pulse_delay_microseconds(
            std::numeric_limits<double>::infinity(),
            0.0,
            60.0,
            pulse_delay
        )
            && pulse_delay == 17U
            && !ar4_protocol::pulse_delay_microseconds(
                ar4_protocol::kMaximumPulseDelayMicroseconds + 1.0,
                0.0,
                60.0,
                pulse_delay
            )
            && pulse_delay == 17U,
        "invalid controller pulse delay was accepted or mutated output"
    );

    int waypoint_count = 0;
    require(
        ar4_protocol::waypoint_count_for_path(
            1.0f,
            0.3f,
            waypoint_count
        )
            && waypoint_count == 4
            && !ar4_protocol::waypoint_count_for_path(
                0.0f,
                0.3f,
                waypoint_count
            )
            && !ar4_protocol::waypoint_count_for_path(
                1.0f,
                0.0f,
                waypoint_count
            ),
        "waypoint-count domain changed"
    );
    require(
        ar4_protocol::interpolated_step_target(
            0,
            10,
            2,
            4,
            10,
            future_step
        )
            && future_step == 5
            && ar4_protocol::interpolated_step_target(
                0,
                10,
                4,
                4,
                10,
                future_step
            )
            && future_step == 10
            && !ar4_protocol::interpolated_step_target(
                0,
                10,
                5,
                4,
                10,
                future_step
            ),
        "interpolated step target changed"
    );

    const float circle_center[3] = {0.0f, 0.0f, 0.0f};
    const float circle_start[3] = {1.0f, 0.0f, 0.0f};
    const float circle_end[3] = {0.0f, 1.0f, 0.0f};
    const float circle_degenerate[3] = {-1.0f, 0.0f, 0.0f};
    require(
        ar4_protocol::supported_trajectory_rotation(0.0f)
            && ar4_protocol::supported_trajectory_rotation(-0.0f)
            && !ar4_protocol::supported_trajectory_rotation(1.0f)
            && !ar4_protocol::supported_trajectory_rotation(
                std::numeric_limits<float>::quiet_NaN()
            ),
        "unsupported trajectory rotation was accepted"
    );
    require(
        ar4_protocol::valid_circle_geometry(
            circle_center,
            circle_start,
            circle_end,
            0.1f
        )
            && !ar4_protocol::valid_circle_geometry(
                circle_center,
                circle_start,
                circle_degenerate,
                0.1f
            ),
        "circle geometry domain changed"
    );
    const float arc_start[3] = {1.0f, 0.0f, 0.0f};
    const float arc_middle[3] = {0.0f, 1.0f, 0.0f};
    const float arc_end[3] = {-1.0f, 0.0f, 0.0f};
    const float arc_collinear_middle[3] = {0.0f, 0.0f, 0.0f};
    const float quarter_arc_middle[3] = {
        0.70710678f,
        0.70710678f,
        0.0f,
    };
    const float quarter_arc_end[3] = {0.0f, 1.0f, 0.0f};
    require(
        ar4_protocol::valid_arc_geometry(
            arc_start,
            arc_middle,
            arc_end,
            0.1f
        )
            && !ar4_protocol::valid_arc_geometry(
                arc_start,
                arc_collinear_middle,
                arc_end,
                0.1f
            ),
        "arc geometry domain changed"
    );
    require(
        ar4_protocol::valid_arc_geometry(
            arc_start,
            quarter_arc_middle,
            quarter_arc_end,
            1.0f
        )
            && !ar4_protocol::valid_arc_geometry(
                arc_start,
                quarter_arc_middle,
                quarter_arc_end,
                1.6f
            ),
        "arc path length omitted part of the traversed central angle"
    );

    const float major_arc_middle[3] = {
        -0.93969262f,
        -0.34202014f,
        0.0f,
    };
    const float major_arc_end[3] = {
        -0.86602540f,
        -0.5f,
        0.0f,
    };
    ar4_protocol::OrderedArcGeometry major_arc_geometry = {};
    require(
        ar4_protocol::valid_arc_geometry(
            arc_start,
            major_arc_middle,
            major_arc_end,
            0.1f,
            &major_arc_geometry
        ),
        "ordered major arc geometry was rejected"
    );
    const double expected_major_arc_radians =
        210.0 * 3.14159265358979323846 / 180.0;
    require(
        std::fabs(
            major_arc_geometry.radians - expected_major_arc_radians
        ) <= 0.000001
            && std::fabs(major_arc_geometry.center[0]) <= 0.000001
            && std::fabs(major_arc_geometry.center[1]) <= 0.000001
            && std::fabs(major_arc_geometry.center[2]) <= 0.000001
            && std::fabs(major_arc_geometry.radius - 1.0) <= 0.000001
            && std::fabs(major_arc_geometry.axis[0]) <= 0.000001
            && std::fabs(major_arc_geometry.axis[1]) <= 0.000001
            && std::fabs(major_arc_geometry.axis[2] - 1.0) <= 0.000001,
        "ordered major arc execution geometry changed"
    );
}

void test_tool_frame_axis_order() {
    const float joints[ROBOT_nDOFs] = {0.0f};
    Matrix4x4 pose;

    configure_identity_kinematic_chain();
    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 90.0f, 0.0f, 0.0f);
    forward_kinematics_arm(joints, pose);
    require_matrix_match(
        pose,
        {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1},
        "tool-frame Rx"
    );

    configure_identity_kinematic_chain();
    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f);
    forward_kinematics_arm(joints, pose);
    require_matrix_match(
        pose,
        {0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
        "tool-frame Ry"
    );

    configure_identity_kinematic_chain();
    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90.0f);
    forward_kinematics_arm(joints, pose);
    require_matrix_match(
        pose,
        {0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
        "tool-frame Rz"
    );
    const std::vector<float> tool_frame = get_robot_tool_frame();
    const std::array<float, 6> expected_tool = {0, 0, 0, 0, 0, 90};
    require(tool_frame.size() == expected_tool.size(), "tool-frame getter size changed");
    for (int index = 0; index < ROBOT_nDOFs; ++index) {
        require(
            std::fabs(tool_frame[index] - expected_tool[index]) <= 0.0001f,
            "tool-frame getter changed the public axis order"
        );
    }

    const float largest_accepted_tool_rotation = std::nextafter(
        std::numeric_limits<float>::max(),
        0.0f
    );
    float firmware_rotation_radians = 0.0f;
    require(
        ar4_protocol::degrees_to_radians(
            largest_accepted_tool_rotation,
            firmware_rotation_radians
        ),
        "firmware rejected the largest native-accepted tool rotation"
    );
    const std::array<float, 6> staged_native_tool = BuildRobotToolFrame(
        0.0f,
        0.0f,
        0.0f,
        largest_accepted_tool_rotation,
        0.0f,
        0.0f
    );
    require(
        staged_native_tool[3] == firmware_rotation_radians,
        "firmware and native tool-angle conversions diverged"
    );
    set_robot_tool_frame(
        0.0f,
        0.0f,
        0.0f,
        largest_accepted_tool_rotation,
        0.0f,
        0.0f
    );
    const std::vector<float> large_tool_frame = get_robot_tool_frame();
    require(
        std::all_of(
            large_tool_frame.begin(),
            large_tool_frame.end(),
            [](float value) { return std::isfinite(value); }
        )
        && large_tool_frame[3] > 0.0f,
        "largest accepted tool rotation produced invalid native state"
    );

    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90.0f);
    const std::vector<float> before_rejected_tool = get_robot_tool_frame();
    float preserved_firmware_rotation = 7.0f;
    require(
        !ar4_protocol::degrees_to_radians(
            std::numeric_limits<float>::max(),
            preserved_firmware_rotation
        )
        && preserved_firmware_rotation == 7.0f,
        "firmware accepted or partially applied a native-rejected tool rotation"
    );
    require_invalid_argument(
        [&]() {
            set_robot_tool_frame(
                0.0f,
                0.0f,
                0.0f,
                std::numeric_limits<float>::max(),
                0.0f,
                0.0f
            );
        },
        "maximum finite tool rotation"
    );
    require(
        get_robot_tool_frame() == before_rejected_tool,
        "rejected tool rotation partially mutated native state"
    );
}

std::array<float, 6> tool_offset_tcp_displacement(
    int frame_index,
    float frame_offset
) {
    configure_tracked_defaults();
    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<float> joints = {
        10.0f,
        -20.0f,
        15.0f,
        35.0f,
        30.0f,
        -25.0f,
    };
    const std::vector<float> target = target_for(joints);
    Matrix4x4 initial_pose;
    forward_kinematics_robot(joints.data(), initial_pose);

    Robot_Kin_Tool[frame_index] = frame_offset;
    const std::vector<float> solution = SolveInverseKinematicsConfigured(
        target,
        joints,
        "A"
    );
    set_robot_tool_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    require(solution.size() == ROBOT_nDOFs, "tool-offset IK returned no solution");

    Matrix4x4 final_pose;
    Matrix4x4 inverse_initial_pose;
    Matrix4x4 relative_pose;
    forward_kinematics_robot(solution.data(), final_pose);
    Matrix_Inv(inverse_initial_pose, initial_pose);
    Matrix_Multiply(relative_pose, inverse_initial_pose, final_pose);

    std::array<float, 6> displacement = {};
    pose_2_xyzuvw(relative_pose, displacement.data());
    for (int index = 3; index < ROBOT_nDOFs; ++index) {
        displacement[index] *= kDegreesPerRadian;
    }
    return displacement;
}

void require_signed_tool_displacement(
    int frame_index,
    float temporary_offset,
    float expected_tcp_displacement,
    const std::string& label
) {
    const std::array<float, 6> displacement = tool_offset_tcp_displacement(
        frame_index,
        temporary_offset
    );
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        const float expected = axis == frame_index
            ? expected_tcp_displacement
            : 0.0f;
        require(
            std::fabs(displacement[axis] - expected) <= 0.05f,
            label + " produced displacement "
                + std::to_string(displacement[axis])
                + " on tool axis " + std::to_string(axis)
        );
    }
}

void test_tool_jog_signed_tcp_displacement() {
    const std::array<char, 6> discrete_axes = {'X', 'Y', 'Z', 'W', 'P', 'R'};
    const std::array<int, 6> live_negative_vectors = {10, 20, 30, 60, 50, 40};

    for (int frame_index = 0; frame_index < ROBOT_nDOFs; ++frame_index) {
        for (int requested_sign : {-1, 1}) {
            const char direction = requested_sign < 0 ? '1' : '0';
            int discrete_index = -1;
            float discrete_offset = 0.0f;
            require(
                ar4_protocol::decode_discrete_tool_offset(
                    discrete_axes[frame_index],
                    direction,
                    1.0f,
                    discrete_index,
                    discrete_offset
                ),
                "discrete tool direction was rejected"
            );
            require(
                discrete_index == frame_index,
                "discrete tool direction selected the wrong frame axis"
            );
            require_signed_tool_displacement(
                frame_index,
                discrete_offset,
                static_cast<float>(requested_sign),
                "discrete tool jog"
            );

            const int live_vector = requested_sign < 0
                ? live_negative_vectors[frame_index]
                : live_negative_vectors[frame_index] + 1;
            int live_index = -1;
            float live_offset = 0.0f;
            require(
                ar4_protocol::decode_live_tool_offset(
                    static_cast<float>(live_vector),
                    1.0f,
                    live_index,
                    live_offset
                ),
                "live tool vector was rejected"
            );
            require(
                live_index == frame_index && live_offset == discrete_offset,
                "live and discrete tool directions diverged"
            );
        }
    }

    int frame_index = -1;
    float frame_offset = 0.0f;
    require(
        !ar4_protocol::decode_discrete_tool_offset(
            'Q',
            '0',
            1.0f,
            frame_index,
            frame_offset
        ),
        "unknown discrete tool axis was accepted"
    );
    require(
        !ar4_protocol::decode_discrete_tool_offset(
            'X',
            '0',
            -1.0f,
            frame_index,
            frame_offset
        ),
        "negative discrete tool distance was accepted"
    );
    require(
        !ar4_protocol::decode_live_tool_offset(
            62.0f,
            1.0f,
            frame_index,
            frame_offset
        ),
        "unknown live tool vector was accepted"
    );

    const float tiny_rotation = std::numeric_limits<float>::denorm_min();
    frame_index = 7;
    frame_offset = 7.0f;
    require(
        !ar4_protocol::decode_discrete_tool_offset(
            'W',
            '0',
            tiny_rotation,
            frame_index,
            frame_offset
        )
        && frame_index == 7
        && frame_offset == 7.0f,
        "discrete tool rotation underflow was not rejected atomically"
    );
    require(
        !ar4_protocol::decode_live_tool_offset(
            60.0f,
            tiny_rotation,
            frame_index,
            frame_offset
        )
        && frame_index == 7
        && frame_offset == 7.0f,
        "live tool rotation underflow was not rejected atomically"
    );
    require(
        ar4_protocol::decode_discrete_tool_offset(
            'X',
            '0',
            tiny_rotation,
            frame_index,
            frame_offset
        )
        && frame_index == 0
        && frame_offset == -tiny_rotation,
        "native-unit tool translation was rejected as an angular underflow"
    );
}

void test_primary_home_reference_contract() {
    ar4_protocol::PrimaryHomeReferenceState state = {
        {false, false},
        {123, -456},
    };
    char response[
        ar4_protocol::kPrimaryHomeReferenceResponseCapacity
    ] = {};
    require(
        ar4_protocol::build_primary_home_reference_response(
            state,
            response,
            sizeof(response)
        )
            && std::strcmp(response, "A0B0C0D0") == 0,
        "invalid primary home references leaked stale coordinates"
    );

    int32_t first = 0;
    int32_t second = 0;
    require(
        ar4_protocol::primary_home_reference_millidegrees(
            163.8004f,
            first
        )
            && first == 163800
            && ar4_protocol::primary_home_reference_millidegrees(
                -38.1996f,
                second
            )
            && second == -38200,
        "primary home-reference conversion changed millidegree rounding"
    );
    ar4_protocol::set_primary_home_reference(state, 0, first);
    ar4_protocol::set_primary_home_reference(state, 1, second);
    require(
        ar4_protocol::build_primary_home_reference_response(
            state,
            response,
            sizeof(response)
        )
            && std::strcmp(response, "A1B163800C1D-38200") == 0,
        "primary home-reference response changed wire framing"
    );

    ar4_protocol::invalidate_primary_home_reference_axis(state, 0);
    require(
        !state.valid[0]
            && state.millidegrees[0] == 0
            && state.valid[1]
            && state.millidegrees[1] == second,
        "axis-local home-reference invalidation changed another axis"
    );

    ar4_protocol::invalidate_primary_home_reference(state);
    require(
        !state.valid[0]
            && !state.valid[1]
            && state.millidegrees[0] == 0
            && state.millidegrees[1] == 0,
        "primary home-reference invalidation retained stale state"
    );
    require(
        !ar4_protocol::primary_home_reference_millidegrees(
            std::numeric_limits<float>::infinity(),
            first
        )
            && first == 163800,
        "non-finite primary home reference was accepted"
    );
}

void test_firmware_identity_contract() {
    require(ar4_protocol::identity_field_valid("AR4"), "valid identity was rejected");
    require(
        ar4_protocol::identity_field_valid("A\\\"B"),
        "escapable identity was rejected"
    );
    require(!ar4_protocol::identity_field_valid(""), "empty identity was accepted");
    require(
        ar4_protocol::identity_field_valid(" AR4")
            && ar4_protocol::identity_field_valid("AR4 ")
            && ar4_protocol::identity_field_valid("AR[4"),
        "legacy printable identity was rejected"
    );
    require(!ar4_protocol::identity_field_valid("AR\n4"), "control byte was accepted");
    require(
        ar4_protocol::identity_field_valid("1234567890123456789012345678901"),
        "maximum-length identity was rejected"
    );
    require(
        !ar4_protocol::identity_field_valid("12345678901234567890123456789012"),
        "overlength identity was accepted"
    );
    char hardware_id[ar4_protocol::kControllerHardwareIdCapacity] = {0};
    require(
        ar4_protocol::format_controller_hardware_id(
            0x12ABEFu,
            hardware_id,
            sizeof(hardware_id)
        )
            && std::string(hardware_id) == "12ABEF"
            && ar4_protocol::controller_hardware_id_valid(hardware_id)
            && !ar4_protocol::format_controller_hardware_id(
                0x1000000u,
                hardware_id,
                sizeof(hardware_id)
            )
            && !ar4_protocol::controller_hardware_id_valid("12abef"),
        "controller hardware identity encoding changed"
    );

    ar4_protocol::IdentitySetCommandFields identity_command = {};
    require(
        ar4_protocol::parse_identity_set_command(
            "[M]AR[4[V]MK]3[B]Teensy 4.1[S]SN-1[A]Asset-1",
            identity_command
        )
            && std::string(identity_command.robot_model) == "AR[4"
            && std::string(identity_command.robot_version) == "MK]3"
            && std::string(identity_command.driver_board) == "Teensy 4.1"
            && std::string(identity_command.serial_number) == "SN-1"
            && std::string(identity_command.asset_tag) == "Asset-1",
        "canonical SR identity payload was rejected or changed"
    );
    std::strcpy(identity_command.robot_model, "sentinel");
    std::strcpy(identity_command.robot_version, "sentinel");
    std::strcpy(identity_command.driver_board, "sentinel");
    std::strcpy(identity_command.serial_number, "sentinel");
    std::strcpy(identity_command.asset_tag, "sentinel");
    const auto identity_command_is_sentinel = [&]() {
        return std::string(identity_command.robot_model) == "sentinel"
            && std::string(identity_command.robot_version) == "sentinel"
            && std::string(identity_command.driver_board) == "sentinel"
            && std::string(identity_command.serial_number) == "sentinel"
            && std::string(identity_command.asset_tag) == "sentinel";
    };
    require(
        !ar4_protocol::parse_identity_set_command(nullptr, identity_command)
            && identity_command_is_sentinel(),
        "null SR identity payload was accepted or mutated output"
    );
    const std::array<const char*, 10> invalid_identity_commands = {{
        "[M]AR[V]4[V]MK3[B]Teensy 4.1[S]SN-1[A]Asset-1",
        "[M]AR4[V]MK[B]3[B]Teensy 4.1[S]SN-1[A]Asset-1",
        "[M]AR4[V]MK3[B]Teensy[S]4.1[S]SN-1[A]Asset-1",
        "[M]AR4[V]MK3[B]Teensy 4.1[S]SN[A]-1[A]Asset-1",
        "[M]AR4[V]MK3[B]Teensy 4.1[S]SN-1[A]Asset[M]-1",
        "[M]AR4[B]Teensy 4.1[V]MK3[S]SN-1[A]Asset-1",
        "[M]12345678901234567890123456789012[V]MK3[B]Teensy 4.1[S]SN-1[A]Asset-1",
        "[M][V]MK3[B]Teensy 4.1[S]SN-1[A]Asset-1",
        "[M]AR4[V]MK3[B]Teensy 4.1[S]SN-1Asset-1",
        "[M]AR4[V]MK3[B]Teensy 4.1[S]SN-1[A]Asset\n1",
    }};
    for (const char* command : invalid_identity_commands) {
        require(
            !ar4_protocol::parse_identity_set_command(
                command,
                identity_command
            )
                && identity_command_is_sentinel(),
            "invalid SR identity payload was accepted or mutated output"
        );
    }

    const char* const protocol_capabilities[] = {
        "JT_WRIST_CONFIG_V1",
        "GCODE_DIRECTORY_FRAMING_V1",
        "GCODE_DELETE_IDENTITY_V1",
        "GCODE_WRITE_IDENTITY_V1",
    };
    char response[ar4_protocol::kIdentityJsonCapacity] = {0};
    require(
        ar4_protocol::build_identity_json(
            "12ABEF",
            "Teensy 4.1",
            "6.7.1-ar4hmi.2",
            "AR4\\\"Model",
            "MK3",
            "SN\\42",
            "Asset-1",
            protocol_capabilities,
            4,
            response,
            sizeof(response)
        ),
        "firmware identity frame was not serialized"
    );
    require(
        std::string(response) ==
            "{\"ControllerHardwareId\":\"12ABEF\",\"DriverModel\":\"Teensy 4.1\","
            "\"FirmwareVersion\":\"6.7.1-ar4hmi.2\","
            "\"RobotModel\":\"AR4\\\\\\\"Model\",\"RobotVersion\":\"MK3\","
            "\"SerialNumber\":\"SN\\\\42\",\"AssetTag\":\"Asset-1\","
            "\"ProtocolCapabilities\":[\"JT_WRIST_CONFIG_V1\","
            "\"GCODE_DIRECTORY_FRAMING_V1\","
            "\"GCODE_DELETE_IDENTITY_V1\","
            "\"GCODE_WRITE_IDENTITY_V1\"]}",
        "firmware identity frame did not escape JSON"
    );
    char short_response[32] = {0};
    require(
        !ar4_protocol::build_identity_json(
            "12ABEF",
            "Teensy 4.1",
            "6.7.1-ar4hmi.2",
            "AR4",
            "MK3",
            "SN",
            "Asset",
            protocol_capabilities,
            4,
            short_response,
            sizeof(short_response)
        ),
        "undersized identity frame buffer was accepted"
    );
    const char* const duplicate_capabilities[] = {
        "JT_WRIST_CONFIG_V1",
        "JT_WRIST_CONFIG_V1",
    };
    require(
        !ar4_protocol::build_identity_json(
            "12ABEF",
            "Teensy 4.1",
            "6.7.1-ar4hmi.2",
            "AR4",
            "MK3",
            "SN",
            "Asset",
            duplicate_capabilities,
            2,
            response,
            sizeof(response)
        ),
        "duplicated firmware capabilities were serialized"
    );
    const char* const malformed_capabilities[] = {
        "lowercase",
    };
    require(
        !ar4_protocol::build_identity_json(
            "12ABEF",
            "Teensy 4.1",
            "6.7.1-ar4hmi.2",
            "AR4",
            "MK3",
            "SN",
            "Asset",
            malformed_capabilities,
            1,
            response,
            sizeof(response)
        ),
        "malformed firmware capability was serialized"
    );

    const std::string maximum_escaped_identity(
        ar4_protocol::kIdentityFieldMaximumLength,
        '\\'
    );
    std::array<
        std::string,
        ar4_protocol::kProtocolCapabilityMaximumCount
    > maximum_capability_values;
    std::array<
        const char*,
        ar4_protocol::kProtocolCapabilityMaximumCount
    > maximum_capabilities;
    for (size_t index = 0; index < maximum_capability_values.size(); ++index) {
        maximum_capability_values[index] = std::string(
            ar4_protocol::kProtocolCapabilityMaximumLength - 1,
            'A'
        ) + static_cast<char>('A' + index);
        maximum_capabilities[index] =
            maximum_capability_values[index].c_str();
    }
    require(
        ar4_protocol::build_identity_json(
            "12ABEF",
            maximum_escaped_identity.c_str(),
            maximum_escaped_identity.c_str(),
            maximum_escaped_identity.c_str(),
            maximum_escaped_identity.c_str(),
            maximum_escaped_identity.c_str(),
            maximum_escaped_identity.c_str(),
            maximum_capabilities.data(),
            maximum_capabilities.size(),
            response,
            sizeof(response)
        ),
        "maximum escaped identity frame exceeded the declared capacity"
    );
}

void test_firmware_command_queue_consumption() {
    using ar4_protocol::MotionCommandStatus;

    require(
        ar4_protocol::should_continue_stored_playback(
            MotionCommandStatus::kCompleted
        )
            && !ar4_protocol::should_continue_stored_playback(
                MotionCommandStatus::kRejected
            )
            && !ar4_protocol::should_continue_stored_playback(
                MotionCommandStatus::kTerminalFaultReported
            )
            && !ar4_protocol::should_emit_generic_motion_error(
                MotionCommandStatus::kCompleted
            )
            && ar4_protocol::should_emit_generic_motion_error(
                MotionCommandStatus::kRejected
            )
            && !ar4_protocol::should_emit_generic_motion_error(
                MotionCommandStatus::kTerminalFaultReported
            ),
        "stored playback motion-result policy changed"
    );

    std::string current = "invalid JT";
    std::string first = "invalid JT";
    std::string second = "valid RJ";
    std::string third = "valid RP";
    ar4_protocol::consume_command_queue(current, first, second, third);
    require(current.empty(), "consumed firmware input remained active");
    require(first == "valid RJ", "next firmware command did not advance");
    require(second == "valid RP", "third firmware command did not advance");
    require(third.empty(), "consumed firmware queue tail was not cleared");

    current = "invalid SR";
    first = "invalid SR";
    second.clear();
    third = "valid HO";
    ar4_protocol::consume_command_queue(current, first, second, third);
    require(first == "valid HO", "sparse firmware queue did not collapse");
    require(second.empty() && third.empty(), "sparse firmware queue retained stale data");

    const std::string motion_payload =
        "MJX1Y2Z3Rz4Ry5Rx6J70J80J90"
        "Sp50Ac10Dc20Rm25WNLm000000";
    FirmwareCommandText serial_payload("unchanged");
    require(
        ar4_protocol::extract_serial_command_payload(
            FirmwareCommandText(motion_payload + "\r\n"),
            serial_payload
        ),
        "valid CRLF firmware frame was rejected"
    );
    require(
        serial_payload.charAt(0) == 'M'
            && serial_payload.charAt(1) == 'J',
        "firmware frame preprocessing changed the opcode"
    );
    ar4_protocol::CartesianMoveCommandFields fields = {};
    require(
        ar4_protocol::parse_cartesian_move_command(
            serial_payload.substring(
                2,
                static_cast<int>(serial_payload.length())
            ),
            fields
        ),
        "preprocessed firmware frame did not reach the strict motion grammar"
    );

    for (const std::string& frame : {
        std::string(" ") + motion_payload + "\n",
        motion_payload + " \n",
    }) {
        FirmwareCommandText malformed_payload("unchanged");
        require(
            ar4_protocol::extract_serial_command_payload(
                FirmwareCommandText(frame),
                malformed_payload
            ),
            "framed whitespace was discarded before strict parsing"
        );
        const bool motion_opcode = malformed_payload.charAt(0) == 'M'
            && malformed_payload.charAt(1) == 'J';
        require(
            !motion_opcode
                || !ar4_protocol::parse_cartesian_move_command(
                    malformed_payload.substring(
                        2,
                        static_cast<int>(malformed_payload.length())
                    ),
                    fields
                ),
            "outer whitespace became an executable firmware motion frame"
        );
    }

    for (const std::string& frame : {
        motion_payload,
        motion_payload + "\rX\n",
        std::string("\n"),
    }) {
        FirmwareCommandText rejected_payload("unchanged");
        require(
            !ar4_protocol::extract_serial_command_payload(
                FirmwareCommandText(frame),
                rejected_payload
            ),
            "invalid firmware line boundary was accepted"
        );
        require(
            std::string(rejected_payload.c_str()) == "unchanged",
            "rejected firmware frame mutated the staged payload"
        );
    }

    const std::string stored_motion = motion_payload.substr(2);
    FirmwareCommandText stored_payload("unchanged");
    require(
        ar4_protocol::extract_stored_command_payload(
            FirmwareCommandText(stored_motion + "\r"),
            stored_payload
        )
            && ar4_protocol::parse_cartesian_move_command(
                stored_payload,
                fields
            ),
        "valid CRLF-backed SD motion row was rejected"
    );
    for (const std::string& row : {
        std::string(" ") + stored_motion,
        stored_motion + " ",
    }) {
        FirmwareCommandText exact_payload("unchanged");
        require(
            ar4_protocol::extract_stored_command_payload(
                FirmwareCommandText(row),
                exact_payload
            )
                && !ar4_protocol::parse_cartesian_move_command(
                    exact_payload,
                    fields
                ),
            "SD preprocessing discarded invalid outer whitespace"
        );
    }
    for (const std::string& row : {
        std::string(),
        stored_motion + "\rX",
        stored_motion + "\n",
    }) {
        FirmwareCommandText rejected_payload("unchanged");
        require(
            !ar4_protocol::extract_stored_command_payload(
                FirmwareCommandText(row),
                rejected_payload
            ),
            "invalid SD row boundary was accepted"
        );
    }
}

void test_firmware_spline_response_contract() {
    require(
        ar4_protocol::should_emit_spline_preface(true, "MS"),
        "active spline motion lost the position preface"
    );
    for (const char* opcode : {"HO", "RP", "DB", "JT", "SS", ""}) {
        require(
            !ar4_protocol::should_emit_spline_preface(true, opcode),
            "non-spline command gained a speculative response frame"
        );
    }
    require(
        !ar4_protocol::should_emit_spline_preface(false, "MS")
        && !ar4_protocol::should_emit_spline_preface(true, nullptr),
        "inactive or invalid spline state emitted a response frame"
    );
}

void test_firmware_debug_command_transaction() {
    ar4_protocol::DebugCommand command = {true, true, true};
    require(
        ar4_protocol::parse_debug_command("[D]1[P]X", command)
            == ar4_protocol::DebugCommandStatus::kInvalidPersistenceValue,
        "invalid debug persistence value was accepted"
    );
    require(
        command.debug_value
            && command.persistence_requested
            && command.persistence_value,
        "invalid debug command mutated the parsed transaction"
    );
    require(
        ar4_protocol::parse_debug_command("[D]1extra", command)
            == ar4_protocol::DebugCommandStatus::kInvalidFormat,
        "trailing debug command data was accepted"
    );
    require(
        ar4_protocol::parse_debug_command("[D]0[P]1", command)
            == ar4_protocol::DebugCommandStatus::kValid,
        "valid persistent debug command was rejected"
    );

    bool live_debug = true;
    bool persistence_called = false;
    const bool failed = ar4_protocol::apply_debug_command(
        command,
        live_debug,
        [&](bool value) {
            persistence_called = true;
            require(value, "parsed persistence value changed before writing");
            return false;
        }
    );
    require(!failed, "failed debug persistence reported success");
    require(persistence_called, "requested debug persistence was not attempted");
    require(live_debug, "failed debug persistence changed live debug state");

    bool persisted_value = false;
    const bool applied = ar4_protocol::apply_debug_command(
        command,
        live_debug,
        [&](bool value) {
            persisted_value = value;
            return true;
        }
    );
    require(applied, "verified debug persistence reported failure");
    require(!live_debug, "valid debug command did not update live state");
    require(persisted_value, "valid debug command wrote the wrong persistence state");

    require(
        ar4_protocol::parse_debug_command("[D]1", command)
            == ar4_protocol::DebugCommandStatus::kValid,
        "valid non-persistent debug command was rejected"
    );
    persistence_called = false;
    require(
        ar4_protocol::apply_debug_command(
            command,
            live_debug,
            [&](bool) {
                persistence_called = true;
                return true;
            }
        ),
        "valid non-persistent debug command reported failure"
    );
    require(live_debug, "non-persistent debug command did not update live state");
    require(!persistence_called, "non-persistent debug command wrote EEPROM");
}

void put_identity_field(FakeEeprom& eeprom, int address, const char* value) {
    require(
        ar4_protocol::write_identity_field(eeprom, address, value),
        "valid identity field did not persist"
    );
}

ar4_protocol::IdentityRecordStatus load_identity_status(FakeEeprom& eeprom) {
    char robot_model[ar4_protocol::kIdentityFieldStorageSize] = {};
    char robot_version[ar4_protocol::kIdentityFieldStorageSize] = {};
    char driver_board[ar4_protocol::kIdentityFieldStorageSize] = {};
    char serial_number[ar4_protocol::kIdentityFieldStorageSize] = {};
    char asset_tag[ar4_protocol::kIdentityFieldStorageSize] = {};
    return ar4_protocol::load_identity_record(
        eeprom,
        robot_model,
        robot_version,
        driver_board,
        serial_number,
        asset_tag
    );
}

void test_firmware_eeprom_persistence_contract() {
    {
        FakeEeprom eeprom;
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kUninitialized,
            "absent identity marker was not classified as uninitialized"
        );
    }

    {
        FakeEeprom eeprom;
        put_identity_field(eeprom, ar4_protocol::kRobotModelAddress, "AR4");
        put_identity_field(eeprom, ar4_protocol::kRobotVersionAddress, "MK3");
        put_identity_field(
            eeprom,
            ar4_protocol::kDriverBoardAddress,
            "Teensy 4.1"
        );
        put_identity_field(eeprom, ar4_protocol::kSerialNumberAddress, "SN-1");
        put_identity_field(eeprom, ar4_protocol::kAssetTagAddress, "Asset-1");
        const std::uint32_t legacy_marker =
            ar4_protocol::kIdentityLegacyMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, legacy_marker);

        std::uint8_t untouched_debug = 0;
        eeprom.get(ar4_protocol::kDebugValueAddress, untouched_debug);
        require(
            untouched_debug == ar4_protocol::kLegacyErasedDebugValue,
            "old SR fixture did not preserve erased debug storage"
        );
        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kMigrated,
            "old SR identity record with erased debug storage did not migrate"
        );
        bool loaded_debug = true;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug)
                && loaded_debug == ar4_protocol::kLegacyDefaultDebugValue,
            "erased legacy debug storage did not migrate to the safe default"
        );
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kValid,
            "old SR identity record was not loadable after migration"
        );
    }

    const std::array<std::uint8_t, 3> invalid_legacy_debug_values = {
        2,
        0x7F,
        0xFE,
    };
    for (const std::uint8_t invalid_debug : invalid_legacy_debug_values) {
        FakeEeprom eeprom;
        put_identity_field(eeprom, ar4_protocol::kRobotModelAddress, "AR4");
        put_identity_field(eeprom, ar4_protocol::kRobotVersionAddress, "MK3");
        put_identity_field(
            eeprom,
            ar4_protocol::kDriverBoardAddress,
            "Teensy 4.1"
        );
        put_identity_field(eeprom, ar4_protocol::kSerialNumberAddress, "SN-1");
        put_identity_field(eeprom, ar4_protocol::kAssetTagAddress, "Asset-1");
        eeprom.put(ar4_protocol::kDebugValueAddress, invalid_debug);
        const std::uint32_t legacy_marker =
            ar4_protocol::kIdentityLegacyMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, legacy_marker);

        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kFailed,
            "invalid legacy debug byte migrated"
        );

        std::uint32_t observed_identity_marker = 0;
        std::uint32_t observed_debug_marker = 0;
        std::uint8_t observed_debug = 0;
        eeprom.get(
            ar4_protocol::kIdentityMagicAddress,
            observed_identity_marker
        );
        eeprom.get(ar4_protocol::kDebugMagicAddress, observed_debug_marker);
        eeprom.get(ar4_protocol::kDebugValueAddress, observed_debug);
        require(
            observed_identity_marker == legacy_marker
                && observed_debug_marker
                    == std::numeric_limits<std::uint32_t>::max()
                && observed_debug == invalid_debug,
            "invalid legacy debug migration rewrote persistence"
        );
    }

    {
        FakeEeprom eeprom;
        const std::uint32_t marker =
            ar4_protocol::kIdentityTransactionMarker;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, marker);
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kCorrupt,
            "in-progress identity marker was not classified as corrupt"
        );
    }

    {
        FakeEeprom eeprom;
        const std::uint32_t interrupted_marker = 0;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, interrupted_marker);
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kCorrupt,
            "unknown identity marker was not classified as corrupt"
        );
    }

    {
        FakeEeprom eeprom;
        put_identity_field(eeprom, ar4_protocol::kRobotModelAddress, "AR4");
        put_identity_field(eeprom, ar4_protocol::kRobotVersionAddress, "MK3");
        put_identity_field(
            eeprom,
            ar4_protocol::kDriverBoardAddress,
            "Teensy 4.1"
        );
        put_identity_field(eeprom, ar4_protocol::kSerialNumberAddress, "SN-1");
        put_identity_field(eeprom, ar4_protocol::kAssetTagAddress, "Asset-1");
        const std::uint32_t marker = ar4_protocol::kIdentityMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, marker);

        char robot_model[ar4_protocol::kIdentityFieldStorageSize] = {};
        char robot_version[ar4_protocol::kIdentityFieldStorageSize] = {};
        char driver_board[ar4_protocol::kIdentityFieldStorageSize] = {};
        char serial_number[ar4_protocol::kIdentityFieldStorageSize] = {};
        char asset_tag[ar4_protocol::kIdentityFieldStorageSize] = {};
        require(
            ar4_protocol::load_identity_record(
                eeprom,
                robot_model,
                robot_version,
                driver_board,
                serial_number,
                asset_tag
            ) == ar4_protocol::IdentityRecordStatus::kValid,
            "complete committed identity record was not loadable"
        );
        require(
            std::string(driver_board) == "Teensy 4.1",
            "loaded identity record changed a stored field"
        );

        char corrupt[ar4_protocol::kIdentityFieldStorageSize] = {};
        std::strcpy(corrupt, "invalid\nvalue");
        eeprom.put(ar4_protocol::kDriverBoardAddress, corrupt);
        require(
            ar4_protocol::load_identity_record(
                eeprom,
                robot_model,
                robot_version,
                driver_board,
                serial_number,
                asset_tag
            ) == ar4_protocol::IdentityRecordStatus::kCorrupt,
            "marker-valid corrupt identity record was accepted"
        );
    }

    {
        FakeEeprom eeprom;
        put_identity_field(eeprom, ar4_protocol::kRobotModelAddress, " AR[4 ");
        put_identity_field(eeprom, ar4_protocol::kRobotVersionAddress, "MK3");
        put_identity_field(
            eeprom,
            ar4_protocol::kDriverBoardAddress,
            "Teensy 4.1"
        );
        put_identity_field(eeprom, ar4_protocol::kSerialNumberAddress, "SN-1");
        const std::uint8_t legacy_debug = 1;
        eeprom.put(ar4_protocol::kDebugValueAddress, legacy_debug);
        const std::uint32_t legacy_marker =
            ar4_protocol::kIdentityLegacyMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, legacy_marker);
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kMigrationRequired,
            "legacy identity record was not routed to migration"
        );
        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kMigrated,
            "valid legacy persistence did not migrate"
        );

        char robot_model[ar4_protocol::kIdentityFieldStorageSize] = {};
        char robot_version[ar4_protocol::kIdentityFieldStorageSize] = {};
        char driver_board[ar4_protocol::kIdentityFieldStorageSize] = {};
        char serial_number[ar4_protocol::kIdentityFieldStorageSize] = {};
        char asset_tag[ar4_protocol::kIdentityFieldStorageSize] = {};
        require(
            ar4_protocol::load_identity_record(
                eeprom,
                robot_model,
                robot_version,
                driver_board,
                serial_number,
                asset_tag
            ) == ar4_protocol::IdentityRecordStatus::kValid,
            "migrated identity record was not loadable"
        );
        require(
            std::string(robot_model) == " AR[4 "
                && std::string(asset_tag) == "Unset",
            "legacy identity migration changed data or missed an erased field"
        );
        bool loaded_debug = false;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug)
                && loaded_debug,
            "legacy debug state did not migrate"
        );
        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kNotRequired,
            "completed persistence migration was not idempotent"
        );
    }

    {
        FakeEeprom eeprom;
        put_identity_field(eeprom, ar4_protocol::kRobotModelAddress, "AR4");
        put_identity_field(eeprom, ar4_protocol::kRobotVersionAddress, "MK3");
        put_identity_field(
            eeprom,
            ar4_protocol::kDriverBoardAddress,
            "Teensy 4.1"
        );
        put_identity_field(eeprom, ar4_protocol::kSerialNumberAddress, "SN-1");
        put_identity_field(eeprom, ar4_protocol::kAssetTagAddress, "Asset-1");
        const std::uint32_t legacy_marker =
            ar4_protocol::kIdentityLegacyMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, legacy_marker);
        eeprom.fail_address_write(ar4_protocol::kDebugMagicAddress);
        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kFailed,
            "failed legacy debug verification reported migration success"
        );
        require(
            ar4_protocol::eeprom_marker_valid(
                eeprom,
                ar4_protocol::kIdentityMagicAddress,
                ar4_protocol::kIdentityLegacyMagicNumber
            ),
            "pre-identity migration failure destroyed the legacy marker"
        );
    }

    {
        FakeEeprom eeprom;
        char invalid[ar4_protocol::kIdentityFieldStorageSize] = {};
        std::strcpy(invalid, "AR4\n");
        eeprom.put(ar4_protocol::kRobotModelAddress, invalid);
        const std::uint32_t legacy_marker =
            ar4_protocol::kIdentityLegacyMagicNumber;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, legacy_marker);
        require(
            ar4_protocol::migrate_legacy_persistence(eeprom)
                == ar4_protocol::PersistenceMigrationStatus::kFailed,
            "invalid legacy identity migrated"
        );
    }

    {
        FakeEeprom eeprom;
        bool loaded_debug = true;
        require(
            !ar4_protocol::load_debug_record(eeprom, loaded_debug),
            "uninitialized EEPROM exposed a debug value"
        );
        require(loaded_debug, "invalid debug record changed the caller value");

        const std::uint8_t stale_debug = 1;
        eeprom.put(ar4_protocol::kDebugValueAddress, stale_debug);
        require(
            ar4_protocol::save_identity_record(eeprom, []() { return true; }),
            "first identity record did not commit"
        );
        require(
            !ar4_protocol::load_debug_record(eeprom, loaded_debug),
            "first identity commit activated an uncommitted debug byte"
        );
    }

    {
        FakeEeprom eeprom;
        require(
            ar4_protocol::save_debug_record(eeprom, true),
            "valid debug record did not commit"
        );
        bool loaded_debug = false;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug) && loaded_debug,
            "committed debug record did not survive reload"
        );
        require(
            ar4_protocol::save_debug_record(eeprom, false),
            "replacement debug record did not commit"
        );
        loaded_debug = true;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug) && !loaded_debug,
            "replacement debug value did not survive reload"
        );
    }

    {
        FakeEeprom eeprom;
        require(ar4_protocol::save_debug_record(eeprom, true), "debug seed failed");
        eeprom.fail_address_write(ar4_protocol::kDebugMagicAddress);
        require(
            !ar4_protocol::save_debug_record(eeprom, false),
            "failed debug invalidation reported success"
        );
        bool loaded_debug = false;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug) && loaded_debug,
            "failed invalidation mutated the prior committed debug record"
        );
    }

    {
        FakeEeprom eeprom;
        require(ar4_protocol::save_debug_record(eeprom, true), "debug seed failed");
        eeprom.fail_address_write(ar4_protocol::kDebugValueAddress);
        require(
            !ar4_protocol::save_debug_record(eeprom, false),
            "failed debug value write reported success"
        );
        bool loaded_debug = false;
        require(
            !ar4_protocol::load_debug_record(eeprom, loaded_debug),
            "failed debug value write left a loadable record"
        );
    }

    {
        FakeEeprom eeprom;
        eeprom.fail_address_write(ar4_protocol::kDebugMagicAddress, 2);
        require(
            !ar4_protocol::save_debug_record(eeprom, true),
            "failed debug commit marker reported success"
        );
        bool loaded_debug = false;
        require(
            !ar4_protocol::load_debug_record(eeprom, loaded_debug),
            "failed debug commit marker left a loadable record"
        );
    }

    {
        FakeEeprom eeprom;
        const std::uint32_t valid_marker = ar4_protocol::kIdentityMagicNumber;
        const std::uint32_t prior_field = 17;
        eeprom.put(ar4_protocol::kIdentityMagicAddress, valid_marker);
        eeprom.put(ar4_protocol::kRobotModelAddress, prior_field);
        eeprom.fail_address_write(ar4_protocol::kIdentityMagicAddress);
        bool fields_called = false;
        require(
            !ar4_protocol::save_identity_record(eeprom, [&]() {
                fields_called = true;
                const std::uint32_t replacement = 23;
                eeprom.put(ar4_protocol::kRobotModelAddress, replacement);
                return true;
            }),
            "failed identity invalidation reported success"
        );
        require(!fields_called, "identity fields changed before invalidation verified");
        std::uint32_t loaded_field = 0;
        eeprom.get(ar4_protocol::kRobotModelAddress, loaded_field);
        require(loaded_field == prior_field, "failed invalidation changed identity data");
        require(
            ar4_protocol::eeprom_marker_valid(
                eeprom,
                ar4_protocol::kIdentityMagicAddress,
                ar4_protocol::kIdentityMagicNumber
            ),
            "failed invalidation damaged the prior identity marker"
        );
    }

    {
        FakeEeprom eeprom;
        const char* maximum_field = "1234567890123456789012345678901";
        require(
            std::strlen(maximum_field)
                == ar4_protocol::kIdentityFieldMaximumLength,
            "maximum identity fixture has the wrong length"
        );
        eeprom.fail_byte_write(
            ar4_protocol::kRobotModelAddress
            + ar4_protocol::kIdentityFieldStorageSize
            - 1
        );
        require(
            !ar4_protocol::save_identity_record(eeprom, [&]() {
                return ar4_protocol::write_identity_field(
                    eeprom,
                    ar4_protocol::kRobotModelAddress,
                    maximum_field
                );
            }),
            "failed identity terminator write reported success"
        );
        require(
            !ar4_protocol::eeprom_marker_valid(
                eeprom,
                ar4_protocol::kIdentityMagicAddress,
                ar4_protocol::kIdentityMagicNumber
            ),
            "partial identity terminator remained loadable"
        );
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kCorrupt,
            "partial identity field reload was not classified as corrupt"
        );
    }

    {
        FakeEeprom eeprom;
        eeprom.fail_address_write(ar4_protocol::kIdentityMagicAddress, 2);
        require(
            !ar4_protocol::save_identity_record(eeprom, []() { return true; }),
            "failed identity commit marker reported success"
        );
        require(
            !ar4_protocol::eeprom_marker_valid(
                eeprom,
                ar4_protocol::kIdentityMagicAddress,
                ar4_protocol::kIdentityMagicNumber
            ),
            "failed identity commit marker left a loadable record"
        );
        require(
            load_identity_status(eeprom)
                == ar4_protocol::IdentityRecordStatus::kCorrupt,
            "failed identity commit reload was not classified as corrupt"
        );
    }

    {
        FakeEeprom eeprom;
        require(ar4_protocol::save_debug_record(eeprom, true), "debug seed failed");
        require(
            ar4_protocol::save_identity_record(eeprom, []() { return true; }),
            "identity record did not commit"
        );
        bool loaded_debug = false;
        require(
            ar4_protocol::load_debug_record(eeprom, loaded_debug) && loaded_debug,
            "identity commit invalidated the independent debug record"
        );
    }
}

void test_wrist_branch_selection() {
    configure_tracked_defaults();
    const std::vector<float> positive = {10.0f, -20.0f, 15.0f, 35.0f, 30.0f, -25.0f};
    const std::vector<float> target = target_for(positive);

    const std::vector<float> automatic = SolveInverseKinematicsConfigured(
        target,
        positive,
        "a"
    );
    require_pose_match(target, automatic, "automatic wrist solution");
    require(automatic[4] > 0.5f, "automatic wrist solution lost seed continuity");

    const std::vector<float> flipped = SolveInverseKinematicsConfigured(
        target,
        positive,
        "N"
    );
    require_pose_match(target, flipped, "negative wrist solution");
    require(flipped[4] < -0.5f, "negative wrist branch was not selected");

    const std::vector<float> restored = SolveInverseKinematicsConfigured(
        target,
        flipped,
        "F"
    );
    require_pose_match(target, restored, "positive wrist solution");
    require(restored[4] > 0.5f, "positive wrist branch was not selected");

    Robot_JointLimits_Lower[4] = 0.0f;
    const std::vector<float> unavailable_negative =
        SolveInverseKinematicsConfigured(target, positive, "N");
    require(
        unavailable_negative.empty(),
        "native solver returned an unavailable requested wrist branch"
    );
}

void test_firmware_wrist_selection_contract() {
    const float maximum = std::numeric_limits<float>::max();
    const float large_difference = ar4_protocol::wrist_angular_difference(
        maximum,
        -maximum
    );
    require(
        std::isfinite(large_difference)
        && std::fabs(large_difference) <= 180.0f,
        "large finite wrist difference did not normalize"
    );

    float solutions[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    int solution_count = 0;
    const std::array<float, 6> positive = {0, -20, 15, 35, 30, -25};
    const std::array<float, 6> negative = {0, -20, 15, -145, -30, 155};
    require(
        ar4_protocol::append_wrist_solution(
            solutions,
            solution_count,
            positive.data()
        ),
        "positive firmware wrist candidate was not recorded"
    );
    require(
        ar4_protocol::append_wrist_solution(
            solutions,
            solution_count,
            negative.data()
        ),
        "negative firmware wrist candidate was not recorded"
    );
    require(
        ar4_protocol::select_wrist_solution(
            solutions,
            solution_count,
            positive.data(),
            'F'
        ) == 0,
        "firmware selector did not choose the positive branch"
    );
    require(
        ar4_protocol::select_wrist_solution(
            solutions,
            solution_count,
            positive.data(),
            'N'
        ) == 1,
        "firmware selector did not choose the negative branch"
    );
    require(
        ar4_protocol::select_wrist_solution(
            solutions,
            solution_count,
            positive.data(),
            'X'
        ) == -1,
        "firmware selector accepted an unknown wrist mode"
    );

    float generated[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    const std::array<float, 6> target = {};
    const std::array<float, 6> limits = {
        180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f,
    };
    std::array<float, 6> generated_estimate = positive;
    generated_estimate[4] = 25.0f;
    std::vector<float> observed_seeds;
    const int generated_count = ar4_protocol::generate_wrist_solutions(
        generated,
        target.data(),
        generated_estimate.data(),
        limits.data(),
        limits.data(),
        [](const float*, const float*, float, float) {
            return true;
        },
        [&](const float*, float*, const float* seed) {
            observed_seeds.push_back(seed[4]);
            return true;
        }
    );
    require(
        generated_count == ar4_protocol::kMaximumWristSolutions
        && observed_seeds.size()
            == static_cast<std::size_t>(
                ar4_protocol::kWristFixedSeedCount
            ),
        "shared wrist generator did not evaluate the complete seed contract"
    );
    for (int index = 0; index < ar4_protocol::kWristFixedSeedCount; ++index) {
        require(
            observed_seeds[static_cast<std::size_t>(index)]
                == ar4_protocol::wrist_seed_degrees(index),
            "shared wrist generator changed seed ordering"
        );
    }

    float negative_only[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    int negative_count = 0;
    require(
        ar4_protocol::append_wrist_solution(
            negative_only,
            negative_count,
            negative.data()
        ),
        "negative-only firmware candidate was not recorded"
    );
    require(
        ar4_protocol::select_wrist_solution(
            negative_only,
            negative_count,
            positive.data(),
            'F'
        ) == -1,
        "firmware selector bypassed an unavailable requested branch"
    );

    float capacity[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    int capacity_count = 0;
    for (int index = 0; index < ar4_protocol::kMaximumWristSolutions; ++index) {
        std::array<float, 6> candidate = positive;
        candidate[0] = static_cast<float>(index);
        require(
            ar4_protocol::append_wrist_solution(
                capacity,
                capacity_count,
                candidate.data()
            ),
            "firmware wrist candidate capacity ended early"
        );
    }
    std::array<float, 6> overflow = positive;
    overflow[0] = 100.0f;
    require(
        !ar4_protocol::append_wrist_solution(
            capacity,
            capacity_count,
            overflow.data()
        )
        && capacity_count == ar4_protocol::kMaximumWristSolutions,
        "firmware wrist candidate capacity overflowed"
    );

    float large_singular[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    large_singular[3][0] = maximum;
    large_singular[5][0] = maximum;
    const float large_estimate[ar4_protocol::kWristJointCount] = {
        0.0f,
        0.0f,
        0.0f,
        -maximum,
        0.0f,
        -maximum,
    };
    require(
        ar4_protocol::select_wrist_solution(
            large_singular,
            1,
            large_estimate,
            'A'
        ) == 0,
        "large finite singular wrist values were not selectable"
    );

    const std::array<float, 6> multi_turn_candidate = {
        740.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    };
    const std::array<float, 6> multi_turn_estimate = {
        730.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    };
    const std::array<float, 6> multi_turn_limits = {
        800.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f,
    };
    std::array<float, 6> normalized_multi_turn = {};
    require(
        ar4_protocol::normalize_wrist_candidate(
            multi_turn_candidate.data(),
            multi_turn_estimate.data(),
            multi_turn_limits.data(),
            multi_turn_limits.data(),
            normalized_multi_turn.data()
        ),
        "valid multi-turn wrist candidate did not normalize"
    );
    require(
        std::fabs(normalized_multi_turn[0] - 740.0f) <= 0.001f,
        "multi-turn wrist candidate did not select the nearest equivalent"
    );
}

void test_singularity_continuity() {
    configure_tracked_defaults();
    const std::vector<float> singular = {10.0f, -20.0f, 15.0f, 35.0f, 0.0f, -25.0f};
    const std::vector<float> target = target_for(singular);
    for (const char* wrist_config : {"A", "F", "N"}) {
        const std::vector<float> solution = SolveInverseKinematicsConfigured(
            target,
            singular,
            wrist_config
        );
        require_pose_match(
            target,
            solution,
            std::string("singular wrist solution ") + wrist_config
        );
        for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
            require(
                std::fabs(angular_difference(solution[joint], singular[joint])) <= 0.05f,
                "singular wrist solution did not preserve the exact seed"
            );
        }
    }

    for (float j5 : {-0.1f, 0.1f}) {
        std::vector<float> estimate = singular;
        estimate[4] = j5;
        std::vector<float> near_singular_target = estimate;
        near_singular_target[3] += 0.2f;
        near_singular_target[5] -= 0.2f;
        const std::vector<float> near_target = target_for(near_singular_target);
        const std::vector<float> solution = SolveInverseKinematicsConfigured(
            near_target,
            estimate,
            "A"
        );
        require_pose_match(near_target, solution, "near-singular wrist solution");
        require(
            std::fabs(angular_difference(solution[3] + solution[5], 10.0f)) <= 0.1f,
            "near-singular wrist sum lost continuity"
        );
    }
}

void test_wrapping_limits_and_no_solution() {
    configure_tracked_defaults();
    const std::vector<float> wrapped = {10.0f, -20.0f, 15.0f, 179.9f, 25.0f, -179.8f};
    const std::vector<float> wrapped_target = target_for(wrapped);
    std::vector<float> wrapped_estimate = wrapped;
    wrapped_estimate[3] = 179.8f;
    wrapped_estimate[5] = -179.7f;
    const std::vector<float> wrapped_solution = SolveInverseKinematicsConfigured(
        wrapped_target,
        wrapped_estimate,
        "F"
    );
    require_pose_match(wrapped_target, wrapped_solution, "wrapped wrist solution");
    require(
        std::fabs(wrapped_solution[3] - wrapped[3]) <= 0.1f,
        "J4 crossed the wrap boundary"
    );
    require(
        std::fabs(wrapped_solution[5] - wrapped[5]) <= 0.1f,
        "J6 crossed the wrap boundary"
    );

    const std::vector<float> seam_target_joints = {
        10.0f,
        -20.0f,
        15.0f,
        179.0f,
        30.0f,
        -179.0f,
    };
    const std::vector<float> seam_target = target_for(seam_target_joints);
    const std::vector<float> seam_estimate = {
        10.0f,
        -20.0f,
        15.0f,
        -179.0f,
        30.0f,
        179.0f,
    };
    const std::vector<float> seam_solution = SolveInverseKinematicsConfigured(
        seam_target,
        seam_estimate,
        "A"
    );
    require_pose_match(seam_target, seam_solution, "cross-seam wrist solution");

    std::array<float, ROBOT_nDOFs> seam_native_target = {};
    std::copy(
        seam_target.begin(),
        seam_target.end(),
        seam_native_target.begin()
    );
    for (int index = 3; index < ROBOT_nDOFs; ++index) {
        seam_native_target[index] /= kDegreesPerRadian;
    }
    Matrix4x4 seam_target_pose;
    xyzuvw_2_pose(seam_native_target.data(), seam_target_pose);
    float firmware_solutions[
        ar4_protocol::kWristJointCount
    ][ar4_protocol::kMaximumWristSolutions] = {};
    const int firmware_solution_count =
        ar4_protocol::generate_wrist_solutions(
            firmware_solutions,
            seam_native_target.data(),
            seam_estimate.data(),
            Robot_JointLimits_Upper,
            Robot_JointLimits_Lower,
            [&seam_target_pose](
                const float*,
                const float* candidate,
                float position_tolerance,
                float rotation_tolerance
            ) {
                Matrix4x4 candidate_pose;
                forward_kinematics_robot(candidate, candidate_pose);
                return ar4_protocol::wrist_pose_matches(
                    candidate_pose,
                    seam_target_pose,
                    position_tolerance,
                    rotation_tolerance
                );
            },
            [](const float* solver_target, float* candidate, const float* seed) {
                return inverse_kinematics_robot_xyzuvw<float>(
                    solver_target,
                    candidate,
                    seed
                ) != 0;
            }
        );
    const int firmware_selection = ar4_protocol::select_wrist_solution(
        firmware_solutions,
        firmware_solution_count,
        seam_estimate.data(),
        'A'
    );
    require(
        firmware_selection >= 0,
        "shared firmware wrist contract found no cross-seam solution"
    );
    for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        require(
            std::fabs(seam_solution[joint] - seam_estimate[joint]) <= 180.001f,
            "cross-seam solution selected an avoidable near-full J"
                + std::to_string(joint + 1)
                + " rotation: estimate=" + std::to_string(seam_estimate[joint])
                + ", solution=" + std::to_string(seam_solution[joint])
        );
        require(
            std::fabs(
                seam_solution[joint]
                - firmware_solutions[joint][firmware_selection]
            ) <= 0.001f,
            "native and firmware wrist contracts selected different branches"
        );
    }

    configure_tracked_defaults();
    Robot_JointLimits_Upper[0] = 800.0f;
    Robot_JointLimits_Lower[0] = 800.0f;
    const std::vector<float> multi_turn = {
        740.0f, -20.0f, 15.0f, 35.0f, 30.0f, -25.0f,
    };
    std::vector<float> multi_turn_estimate = multi_turn;
    multi_turn_estimate[0] = 730.0f;
    const std::vector<float> multi_turn_target = target_for(multi_turn);
    const std::vector<float> multi_turn_solution =
        SolveInverseKinematicsConfigured(
            multi_turn_target,
            multi_turn_estimate,
            "A"
        );
    require_pose_match(
        multi_turn_target,
        multi_turn_solution,
        "multi-turn wrist solution"
    );
    require(
        std::fabs(multi_turn_solution[0] - multi_turn[0]) <= 0.1f,
        "multi-turn wrist solution selected a remote equivalent"
    );

    configure_tracked_defaults();

    std::vector<float> bounded = wrapped;
    bounded[3] = 179.98f;
    Robot_JointLimits_Upper[3] = 179.98f;
    const std::vector<float> bounded_target = target_for(bounded);
    const std::vector<float> bounded_solution = SolveInverseKinematicsConfigured(
        bounded_target,
        bounded,
        "F"
    );
    require_pose_match(bounded_target, bounded_solution, "bounded wrap solution");
    require(
        bounded_solution[3] <= 179.981f,
        "wrap normalization exceeded the configured upper limit"
    );

    const std::vector<float> unreachable = {
        1000000.0f,
        1000000.0f,
        1000000.0f,
        0.0f,
        0.0f,
        0.0f,
    };
    require(
        SolveInverseKinematicsConfigured(unreachable, wrapped, "A").empty(),
        "unreachable target returned a solution"
    );

    const std::vector<float> ordinary = {10.0f, -20.0f, 15.0f, 35.0f, 30.0f, -25.0f};
    const std::vector<float> ordinary_target = target_for(ordinary);
    for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        Robot_JointLimits_Upper[joint] = 1.0f;
        Robot_JointLimits_Lower[joint] = 1.0f;
    }
    require(
        SolveInverseKinematicsConfigured(ordinary_target, ordinary, "A").empty(),
        "joint-limit exclusion returned a solution"
    );
    configure_tracked_defaults();
}

void test_parallel_determinism() {
    configure_tracked_defaults();
    const std::vector<float> seed = {10.0f, -20.0f, 15.0f, 35.0f, 30.0f, -25.0f};
    const std::vector<float> target = target_for(seed);
    const std::vector<float> expected = SolveInverseKinematicsConfigured(target, seed, "N");
    require_pose_match(target, expected, "parallel reference solution");

    std::atomic<bool> failed(false);
    std::vector<std::thread> workers;
    for (int worker = 0; worker < 8; ++worker) {
        workers.emplace_back([&]() {
            for (int iteration = 0; iteration < 100; ++iteration) {
                const std::vector<float> solution = SolveInverseKinematicsConfigured(
                    target,
                    seed,
                    "N"
                );
                if (solution.size() != expected.size()) {
                    failed.store(true);
                    return;
                }
                for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
                    if (
                        std::fabs(angular_difference(solution[joint], expected[joint]))
                        > 0.001f
                    ) {
                        failed.store(true);
                        return;
                    }
                }
            }
        });
    }
    for (std::thread& worker : workers) worker.join();
    require(!failed.load(), "parallel inverse-kinematics calls were nondeterministic");
}

int run_protocol_contract_probe(const std::string& kind, const std::string& value) {
    bool accepted = false;
    if (kind == "ramp") {
        const FirmwareCommandText command(
            "A1B2C3D4E5F6J70J80J90Sp50Ac10Dc20Rm"
            + value
            + "WNLm000000"
        );
        ar4_protocol::JointMoveCommandFields fields = {};
        accepted = ar4_protocol::parse_joint_move_command(command, fields);
    } else if (kind == "filename") {
        const FirmwareCommandText filename(value);
        accepted = ar4_protocol::valid_controller_filename(
            filename,
            0,
            static_cast<int>(filename.length())
        );
    } else {
        return 2;
    }
    std::cout << (accepted ? "accepted" : "rejected") << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 4 && std::string(argv[1]) == "--protocol-probe") {
        return run_protocol_contract_probe(argv[2], argv[3]);
    }
    try {
        test_rejected_motion_mode_transaction_atomicity();
        test_bounded_serial_frame_accumulator();
        test_directory_entry_name_contract();
        test_singularity_continuity();
        test_boundary_validation();
        test_cartesian_pose_order_contract();
        test_firmware_numeric_parse_contract();
        test_controller_domain_contract();
        test_tool_frame_axis_order();
        test_tool_jog_signed_tcp_displacement();
        test_primary_home_reference_contract();
        test_firmware_identity_contract();
        test_firmware_command_queue_consumption();
        test_firmware_spline_response_contract();
        test_firmware_debug_command_transaction();
        test_firmware_eeprom_persistence_contract();
        test_wrist_branch_selection();
        test_firmware_wrist_selection_contract();
        test_wrapping_limits_and_no_solution();
        test_parallel_determinism();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
