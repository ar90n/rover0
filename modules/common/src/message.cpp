#include "message.hpp"

namespace message {
MsgType get_msg_type(uint32_t raw) {
  union {
    uint32_t raw;
    struct {
      MsgType type;
      uint8_t _place_holder[3];
    } msg;
  } accessor{.raw = raw};
  return accessor.msg.type;
}

std::optional<RxMsg> parse_rx_msg(uint32_t raw) {
  switch (get_msg_type(raw)) {
  case MsgType::MOTOR: {
    return deserialize<MotorMsg>(raw);
  } break;
  case MsgType::ENCODER: {
    return deserialize<EncoderMsg>(raw);
  } break;
  case MsgType::IMU: {
    return deserialize<ImuMsg>(raw);
  } break;
  default:
    return std::nullopt;
  }
}

std::optional<TxMsg> parse_tx_msg(uint32_t raw) {
  switch (get_msg_type(raw)) {
  case MsgType::ENCODER:
    return deserialize<EncoderMsg>(raw);
  case MsgType::IMU:
    return deserialize<ImuMsg>(raw);
  default:
    return std::nullopt;
  }
}

} // namespace message