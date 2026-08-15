#pragma once

#include "servo/servo_types.hpp"

#include <cstddef>
#include <cstdint>

namespace protocol {

constexpr size_t MAX_LINE_LENGTH = 128;

enum class CommandType {
    None,
    Ping,
    Status,
    AllCenter,
    Zero,
    Stand,
    EnableAll,
    DisableAll,
    March,
    WalkDemo,
    WalkStop,
    GaitStatus,
    Servo,
    Leg,
    CalShow,
    CalSetMinus45,
    CalSetPlus45,
    CalSave,
    CalLoad,
    CalReset,
    Help
};

enum class ParseError {
    None,
    UnknownCommand,
    InvalidArgument,
    InvalidLeg,
    InvalidJoint,
    AngleOutOfRange,
    PulseOutOfRange,
    LineTooLong
};

struct Command {
    CommandType type = CommandType::None;
    servo::Leg leg = servo::Leg::FR;
    servo::Joint joint = servo::Joint::Coxa;
    float angle_a = 0.0f;
    float angle_b = 0.0f;
    float angle_c = 0.0f;
    uint16_t pulse_us = 0;
};

struct ParseResult {
    ParseError error = ParseError::None;
    Command command{};
};

ParseResult parse_command(const char *line);
const char *error_text(ParseError error);

class UartLineParser {
public:
    bool push_char(char ch, char *out_line, size_t out_line_size);
    bool overflowed() const { return overflowed_; }
    void clear_overflow() { overflowed_ = false; }

private:
    char buffer_[MAX_LINE_LENGTH + 1]{};
    size_t length_ = 0;
    bool last_was_cr_ = false;
    bool overflowed_ = false;
};

} // namespace protocol
