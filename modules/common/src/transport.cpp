#include "transport.hpp"
#include <array>

namespace
{
  static constexpr uint8_t HEADER_BIT = 0x80;
  static constexpr uint8_t DATA_MASK = 0x7F;

  transport::Writer writer = [](const uint8_t *, size_t) {};

  enum class State : uint8_t
  {
    HEADER_CHECKSUM = 0,
    DATA1,
    DATA2,
    DATA3,
    DATA4,
    DATA5,
  };

  static State state{State::HEADER_CHECKSUM};
  static std::array<uint8_t, 5> buffer;
  static size_t buffer_index = 0;
  static uint8_t expected_checksum = 0;

  uint8_t calculate_checksum(const std::array<uint8_t, 5> &buffer)
  {
    uint8_t sum = 0;
    for (auto b : buffer)
    {
      sum += b & DATA_MASK;
    }
    return sum & DATA_MASK;
  }
} // namespace

namespace transport
{

  void init(Writer &&writer_func)
  {
    writer = writer_func;
  }

  void reset()
  {
    state = State::HEADER_CHECKSUM;
    buffer_index = 0;
  }

  std::optional<uint32_t> consume(uint8_t byte)
  {
    std::optional<uint32_t> ret{std::nullopt};

    if (byte & HEADER_BIT)
    {
      state = State::DATA1;
      buffer_index = 0;
      expected_checksum = byte & DATA_MASK;
      return ret;
    }

    switch (state)
    {
    case State::DATA1:
      buffer[buffer_index++] = byte & DATA_MASK;
      state = State::DATA2;
      break;

    case State::DATA2:
      buffer[buffer_index++] = byte & DATA_MASK;
      state = State::DATA3;
      break;

    case State::DATA3:
      buffer[buffer_index++] = byte & DATA_MASK;
      state = State::DATA4;
      break;

    case State::DATA4:
      buffer[buffer_index++] = byte & DATA_MASK;
      state = State::DATA5;
      break;

    case State::DATA5:
    {
      buffer[buffer_index] = byte & DATA_MASK;
      if (calculate_checksum(buffer) == expected_checksum)
      {
        uint32_t value = 0;
        value |= (buffer[0] & DATA_MASK) << 28;
        value |= (buffer[1] & DATA_MASK) << 21;
        value |= (buffer[2] & DATA_MASK) << 14;
        value |= (buffer[3] & DATA_MASK) << 7;
        value |= (buffer[4] & DATA_MASK);
        ret = value;
      }
      state = State::HEADER_CHECKSUM;
    }
    break;

    case State::HEADER_CHECKSUM:
      break;
    }
    return ret;
  }

  void send(uint32_t data)
  {
    std::array<uint8_t, 5> buffer;
    buffer[0] = (data >> 28) & DATA_MASK;
    buffer[1] = (data >> 21) & DATA_MASK;
    buffer[2] = (data >> 14) & DATA_MASK;
    buffer[3] = (data >> 7) & DATA_MASK;
    buffer[4] = data & DATA_MASK;

    uint8_t header = calculate_checksum(buffer) | HEADER_BIT;
    writer(&header, 1);

    for (const auto &byte : buffer)
    {
      writer(&byte, 1);
    }
  }

} // namespace transport
