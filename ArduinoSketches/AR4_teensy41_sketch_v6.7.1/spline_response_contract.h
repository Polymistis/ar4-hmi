#ifndef AR4_SPLINE_RESPONSE_CONTRACT_H
#define AR4_SPLINE_RESPONSE_CONTRACT_H

namespace ar4_protocol {

inline bool should_emit_spline_preface(
    bool spline_active,
    const char* opcode
) {
    return spline_active
        && opcode != nullptr
        && opcode[0] == 'M'
        && opcode[1] == 'S'
        && opcode[2] == '\0';
}

}  // namespace ar4_protocol

#endif
