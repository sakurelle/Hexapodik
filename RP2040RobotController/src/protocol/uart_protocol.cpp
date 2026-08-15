#include "protocol/uart_protocol.hpp"
#include "servo/servo_config.hpp"

#include <cctype>
#include <cstdlib>
#include <cstring>

namespace protocol {

static void upper_copy(char *dst, size_t dst_size, const char *src) {
    size_t i = 0;
    for (; src[i] && i + 1 < dst_size; ++i) {
        dst[i] = static_cast<char>(toupper(static_cast<unsigned char>(src[i])));
    }
    dst[i] = '\0';
}

static bool parse_leg(const char *text, servo::Leg &leg) {
    if (strcmp(text, "FR") == 0) { leg = servo::Leg::FR; return true; }
    if (strcmp(text, "MR") == 0) { leg = servo::Leg::MR; return true; }
    if (strcmp(text, "RR") == 0) { leg = servo::Leg::RR; return true; }
    if (strcmp(text, "RL") == 0) { leg = servo::Leg::RL; return true; }
    if (strcmp(text, "ML") == 0) { leg = servo::Leg::ML; return true; }
    if (strcmp(text, "FL") == 0) { leg = servo::Leg::FL; return true; }
    return false;
}

static bool parse_joint(const char *text, servo::Joint &joint) {
    if (strcmp(text, "COXA") == 0) { joint = servo::Joint::Coxa; return true; }
    if (strcmp(text, "FEMUR") == 0) { joint = servo::Joint::Femur; return true; }
    if (strcmp(text, "TIBIA") == 0) { joint = servo::Joint::Tibia; return true; }
    return false;
}

static bool parse_float_token(const char *text, float &value) {
    char *end = nullptr;
    value = strtof(text, &end);
    return end && *end == '\0';
}

static bool parse_u16_token(const char *text, uint16_t &value) {
    char *end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (!end || *end != '\0' || parsed < 0 || parsed > 65535) {
        return false;
    }
    value = static_cast<uint16_t>(parsed);
    return true;
}

ParseResult parse_command(const char *line) {
    char upper[MAX_LINE_LENGTH + 1]{};
    upper_copy(upper, sizeof(upper), line);

    char *tokens[8]{};
    size_t count = 0;
    char *cursor = upper;
    while (*cursor && count < 8) {
        while (*cursor == ' ' || *cursor == '\t') {
            ++cursor;
        }
        if (!*cursor) {
            break;
        }
        tokens[count++] = cursor;
        while (*cursor && *cursor != ' ' && *cursor != '\t') {
            ++cursor;
        }
        if (*cursor) {
            *cursor++ = '\0';
        }
    }

    ParseResult result{};
    if (count == 0) {
        return result;
    }

    if (strcmp(tokens[0], "PING") == 0 && count == 1) { result.command.type = CommandType::Ping; return result; }
    if (strcmp(tokens[0], "STATUS") == 0 && count == 1) { result.command.type = CommandType::Status; return result; }
    if (strcmp(tokens[0], "ZERO") == 0 && count == 1) { result.command.type = CommandType::Zero; return result; }
    if (strcmp(tokens[0], "STAND") == 0 && count == 1) { result.command.type = CommandType::Stand; return result; }
    if (strcmp(tokens[0], "HELP") == 0 && count == 1) { result.command.type = CommandType::Help; return result; }
    if (strcmp(tokens[0], "CAL") == 0 && count == 2 && strcmp(tokens[1], "SAVE") == 0) { result.command.type = CommandType::CalSave; return result; }
    if (strcmp(tokens[0], "CAL") == 0 && count == 2 && strcmp(tokens[1], "LOAD") == 0) { result.command.type = CommandType::CalLoad; return result; }
    if (strcmp(tokens[0], "CAL") == 0 && count == 2 && strcmp(tokens[1], "RESET") == 0) { result.command.type = CommandType::CalReset; return result; }
    if (strcmp(tokens[0], "ALL") == 0 && count == 2 && strcmp(tokens[1], "CENTER") == 0) { result.command.type = CommandType::AllCenter; return result; }
    if (strcmp(tokens[0], "ENABLE") == 0 && count == 2 && strcmp(tokens[1], "ALL") == 0) { result.command.type = CommandType::EnableAll; return result; }
    if (strcmp(tokens[0], "DISABLE") == 0 && count == 2 && strcmp(tokens[1], "ALL") == 0) { result.command.type = CommandType::DisableAll; return result; }
    if (strcmp(tokens[0], "MARCH") == 0 && count == 1) { result.command.type = CommandType::March; return result; }
    if (strcmp(tokens[0], "WALK") == 0 && count == 2 && strcmp(tokens[1], "DEMO") == 0) { result.command.type = CommandType::WalkDemo; return result; }
    if (strcmp(tokens[0], "WALK") == 0 && count == 2 && strcmp(tokens[1], "STOP") == 0) { result.command.type = CommandType::WalkStop; return result; }
    if (strcmp(tokens[0], "GAIT") == 0 && count == 2 && strcmp(tokens[1], "STATUS") == 0) { result.command.type = CommandType::GaitStatus; return result; }

    if (strcmp(tokens[0], "SERVO") == 0 && count == 4) {
        result.command.type = CommandType::Servo;
        if (!parse_leg(tokens[1], result.command.leg)) { result.error = ParseError::InvalidLeg; return result; }
        if (!parse_joint(tokens[2], result.command.joint)) { result.error = ParseError::InvalidJoint; return result; }
        if (!parse_float_token(tokens[3], result.command.angle_a)) { result.error = ParseError::InvalidArgument; return result; }
        if (result.command.angle_a < -45.0f || result.command.angle_a > 45.0f) { result.error = ParseError::AngleOutOfRange; }
        return result;
    }

    if (strcmp(tokens[0], "LEG") == 0 && count == 5) {
        result.command.type = CommandType::Leg;
        if (!parse_leg(tokens[1], result.command.leg)) { result.error = ParseError::InvalidLeg; return result; }
        if (!parse_float_token(tokens[2], result.command.angle_a) ||
            !parse_float_token(tokens[3], result.command.angle_b) ||
            !parse_float_token(tokens[4], result.command.angle_c)) {
            result.error = ParseError::InvalidArgument;
            return result;
        }
        if (result.command.angle_a < -45.0f || result.command.angle_a > 45.0f ||
            result.command.angle_b < -45.0f || result.command.angle_b > 45.0f ||
            result.command.angle_c < -45.0f || result.command.angle_c > 45.0f) {
            result.error = ParseError::AngleOutOfRange;
        }
        return result;
    }

    if (strcmp(tokens[0], "CAL") == 0 && count == 4 && strcmp(tokens[1], "SHOW") == 0) {
        result.command.type = CommandType::CalShow;
        if (!parse_leg(tokens[2], result.command.leg)) { result.error = ParseError::InvalidLeg; return result; }
        if (!parse_joint(tokens[3], result.command.joint)) { result.error = ParseError::InvalidJoint; return result; }
        return result;
    }

    if (strcmp(tokens[0], "CAL") == 0 && count == 6 && strcmp(tokens[1], "SET") == 0) {
        if (!parse_leg(tokens[2], result.command.leg)) { result.error = ParseError::InvalidLeg; return result; }
        if (!parse_joint(tokens[3], result.command.joint)) { result.error = ParseError::InvalidJoint; return result; }
        if (!parse_u16_token(tokens[5], result.command.pulse_us)) { result.error = ParseError::InvalidArgument; return result; }
        if (result.command.pulse_us < servo::SAFE_MIN_PULSE_US || result.command.pulse_us > servo::SAFE_MAX_PULSE_US) {
            result.error = ParseError::PulseOutOfRange;
            return result;
        }
        if (strcmp(tokens[4], "MINUS45") == 0) { result.command.type = CommandType::CalSetMinus45; return result; }
        if (strcmp(tokens[4], "PLUS45") == 0) { result.command.type = CommandType::CalSetPlus45; return result; }
        result.error = ParseError::InvalidArgument;
        return result;
    }

    result.error = ParseError::UnknownCommand;
    return result;
}

const char *error_text(ParseError error) {
    switch (error) {
    case ParseError::None: return "OK";
    case ParseError::UnknownCommand: return "ERR UNKNOWN_COMMAND";
    case ParseError::InvalidArgument: return "ERR INVALID_ARGUMENT";
    case ParseError::InvalidLeg: return "ERR INVALID_LEG";
    case ParseError::InvalidJoint: return "ERR INVALID_JOINT";
    case ParseError::AngleOutOfRange: return "ERR ANGLE_OUT_OF_RANGE";
    case ParseError::PulseOutOfRange: return "ERR PULSE_OUT_OF_RANGE";
    case ParseError::LineTooLong: return "ERR INVALID_ARGUMENT";
    }
    return "ERR INVALID_ARGUMENT";
}

bool UartLineParser::push_char(char ch, char *out_line, size_t out_line_size) {
    if (ch == '\n' && last_was_cr_) {
        last_was_cr_ = false;
        return false;
    }

    if (ch == '\r' || ch == '\n') {
        last_was_cr_ = ch == '\r';
        if (length_ == 0) {
            return false;
        }
        const size_t copy_len = length_ < out_line_size - 1 ? length_ : out_line_size - 1;
        memcpy(out_line, buffer_, copy_len);
        out_line[copy_len] = '\0';
        length_ = 0;
        return true;
    }

    last_was_cr_ = false;
    if (length_ >= MAX_LINE_LENGTH) {
        overflowed_ = true;
        length_ = 0;
        return false;
    }

    buffer_[length_++] = ch;
    return false;
}

} // namespace protocol
