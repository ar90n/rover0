#include "utest.h"

#include "message.hpp"

using namespace message;

UTEST(message, check_MsgTYpe_size)
{
  ASSERT_EQ(sizeof MsgType::MOTOR, 1);
}

UTEST(message, check_MotorDevice_size)
{
  ASSERT_EQ(sizeof MotorDevice::REAR_LEFT, 1);
}

UTEST(message, check_ImuData_size)
{
  ASSERT_EQ(sizeof ImuData::ACCEL_X, 1);
}

UTEST(message, check_msg_size)
{
  ASSERT_EQ(sizeof MotorMsg{}, 4);
  ASSERT_EQ(sizeof EncoderMsg{}, 4);
  ASSERT_EQ(sizeof ImuMsg{}, 4);
}

UTEST(message, check_motor_msg_serialization)
{
  MotorMsg msg{ .param = MotorDevice::FRONT_LEFT, .value = -1 };
  ASSERT_EQ(serialize(msg), 0xffff0200);
}

UTEST(message, check_encoder_msg_serialization)
{
  EncoderMsg msg{ .param = MotorDevice::FRONT_RIGHT, .value = 0x12ff };
  ASSERT_EQ(serialize(msg), 0x12ff0301);
}

UTEST(message, check_imu_msg_serialization)
{
  ImuMsg msg{ .param = ImuData::GYRO_X, .value = 0x0012 };
  ASSERT_EQ(serialize(msg), 0x00120302);
}

UTEST(message, check_deserialization)
{
  MotorMsg expect{ .param = MotorDevice::FRONT_LEFT, .value = -1 };
  MotorMsg actual = deserialize<MotorMsg>(0xffff0200);
  ASSERT_EQ(expect.param, expect.param);
  ASSERT_EQ(expect.value, expect.value);
}

UTEST(message, check_get_msg_type)
{
  ASSERT_EQ(get_msg_type(0xffff0200), MsgType::MOTOR);
  ASSERT_EQ(get_msg_type(0x12ff0301), MsgType::ENCODER);
  ASSERT_EQ(get_msg_type(0x00120302), MsgType::IMU);
}

UTEST(message, check_parse_rx_msg)
{
  MotorMsg expect{ .param = MotorDevice::FRONT_LEFT, .value = -1 };

  std::optional<RxMsg> rx_msg = parse_rx_msg(0xffff0200);
  ASSERT_TRUE(rx_msg.has_value());
  ASSERT_EQ(std::get<MotorMsg>(*rx_msg).param, expect.param);
  ASSERT_EQ(std::get<MotorMsg>(*rx_msg).value, expect.value);
}

UTEST(message, check_parse_rx_msg_fail)
{
  std::optional<RxMsg> rx_msg = parse_rx_msg(0x12ff0301);
  ASSERT_FALSE(rx_msg.has_value());
}

UTEST(message, check_parse_tx_msg)
{
  ImuMsg expect{ .param = ImuData::GYRO_X, .value = 0x0012 };

  std::optional<TxMsg> tx_msg = parse_tx_msg(0x00120302);
  ASSERT_TRUE(tx_msg.has_value());
  ASSERT_EQ(std::get<ImuMsg>(*tx_msg).param, expect.param);
  ASSERT_EQ(std::get<ImuMsg>(*tx_msg).value, expect.value);
}

UTEST(message, check_parse_tx_msg_fail)
{
  std::optional<TxMsg> tx_msg = parse_tx_msg(0xffff0200);
  ASSERT_FALSE(tx_msg.has_value());
}