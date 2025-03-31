#include "hardware/irq.h"
#include "pico/multicore.h"

#include "comm_proc.hpp"
#include "config.hpp"
#include "device.hpp"
#include "message.hpp"
#include "queue.hpp"
#include "transport.hpp"

namespace
{

auto uart_control   = device::UartControl::instance();
auto intercore_fifo = device::Core1IntercoreFIFO::instance();

static FixedSizeQueue<uint32_t, Config::QUEUE_SIZE> to_controller_queue;

void uart_write(const uint8_t* buf, size_t len)
{
  uart_control.write(buf, len);
}

}

namespace comm_proc
{
void run()
{
  multicore_fifo_clear_irq();

  ::uart_control.init(115200);
  ::intercore_fifo.init();

  transport::init(uart_write);

  while (1)
  {
    while (::intercore_fifo.has_data())
    {
      uint32_t const bytes = intercore_fifo.read();
      transport::send(bytes);
    }

    while (::uart_control.has_data())
    {
      auto const byte{ ::uart_control.read() };
      if (auto const recv{ transport::consume(byte) }; recv.has_value())
      {
        multicore_fifo_push_blocking(recv.value());
      }
    }
  }
}
}
