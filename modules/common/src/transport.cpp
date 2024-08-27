#include <functional>
#include <cstdint>

#include "endian_utils.hpp"
#include "queue.hpp"
#include "transport.hpp"

namespace {
static constexpr uint32_t QUEUE_SIZE               = 128;

FixedSizeQueue<uint32_t, QUEUE_SIZE> queue;
transport::Writer                   writer = [](const uint8_t*, size_t) {};

enum class State : uint8_t
{
  BYTE0 = 0,
  BYTE1,
  BYTE2,
  BYTE3,
  CHECKSUM,
};
size_t state_index(State state)
{
  return static_cast<size_t>(state);
}
State next_state(State state)
{
  return static_cast<State>(state_index(state) + 1);
}
uint8_t checksum(std::array<uint8_t, 4> const& buffer)
{
  uint8_t sum{ 0 };
  for (auto b : buffer) {
    sum += b;
  }
  return sum;
}

static State                  state{ State::CHECKSUM };
static std::array<uint8_t, 4> buffer;
}

namespace transport {
void init(Writer&& writer)
{
  ::writer = writer;
}

std::optional<uint32_t> consume(uint8_t byte)
{
  std::optional<uint32_t> ret{ std::nullopt };
  switch (state) {
    case State::BYTE0:
    case State::BYTE1:
    case State::BYTE2:
    case State::BYTE3: {
      buffer[state_index(state)] = byte;
      state                      = next_state(state);
    } break;
    case State::CHECKSUM: {
      uint32_t const data{ pack_uint32_be(buffer.data()) };
      uint8_t const  expect_checksum{ byte };
      uint8_t const  actual_checksum{ checksum(buffer) };

      if (actual_checksum == expect_checksum) {
        ret = data;
      }
      state = State::BYTE0;
    } break;
  }
  return ret;
}

void send(uint32_t data)
{
  std::array<uint8_t, 4> const buffer{ unpack_uint32_be(data) };
  uint8_t const                checksum{ ::checksum(buffer) };

  writer(buffer.data(), buffer.size());
  writer(&checksum, 1);
}
}