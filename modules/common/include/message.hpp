#pragma once

#include <cstdint>
#include <optional>
#include <variant>

namespace message
{

  enum class MsgType : uint8_t
  {
    MOTOR,
    ENCODER,
    IMU,
  };

  enum class MotorDevice : uint8_t
  {
    REAR_LEFT,
    REAR_RIGHT,
    FRONT_LEFT,
    FRONT_RIGHT
  };

  enum class ImuData : uint8_t
  {
    ACCEL_X,
    ACCEL_Y,
    ACCEL_Z,
    GYRO_X,
    GYRO_Y,
    GYRO_Z,
    TEMP
  };

  template <MsgType T, typename TP, typename TV>
  struct Message
  {
    using param_type = TP;
    using value_type = TV;

    static constexpr MsgType type{T};
    TP const param{};
    TV const value{};
  };

  using MotorMsg = Message<MsgType::MOTOR, MotorDevice, int16_t>;
  using EncoderMsg = Message<MsgType::ENCODER, MotorDevice, uint16_t>;
  using ImuMsg = Message<MsgType::IMU, ImuData, int16_t>;
  using RxMsg = std::variant<MotorMsg, EncoderMsg, ImuMsg>;
  using TxMsg = std::variant<EncoderMsg, ImuMsg>;

  template <typename T>
  uint32_t serialize(T const &msg)
  {
    union
    {
      uint32_t raw;
      struct
      {
        MsgType type;
        typename T::param_type param;
        typename T::value_type value;
      } msg;
    } ser{.msg = {msg.type, msg.param, msg.value}};
    return ser.raw;
  }

  template <typename T>
  T deserialize(uint32_t raw)
  {
    union
    {
      uint32_t raw;
      struct
      {
        MsgType type;
        typename T::param_type param;
        typename T::value_type value;
      } msg;
    } deser{.raw = raw};
    return T{.param = deser.msg.param, .value = deser.msg.value};
  }

  MsgType get_msg_type(uint32_t raw);
  std::optional<RxMsg> parse_rx_msg(uint32_t raw);
  std::optional<TxMsg> parse_tx_msg(uint32_t raw);
} // namespace message