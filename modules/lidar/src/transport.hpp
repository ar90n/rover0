#ifndef TRANSPORT_HPP_
#define TRANSPORT_HPP_

#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "uart.hpp"

template <typename T>
concept TransportImpl =
    requires(T t, uint baurate, uint8_t *buf, size_t len, int timeout) {
      { t.init(baurate) } -> std::same_as<void>;
      { t.close() } -> std::same_as<void>;
      { t.write(buf, len) } -> std::same_as<void>;
      { t.read(buf, len, timeout) } -> std::same_as<size_t>;
    };

template <TransportImpl T> class UartTransport {
public:
  UartTransport(T &uart, uint baurate) : uart(uart), baurate(baurate) {}

  static bool open(struct uxrCustomTransport *transport) {
    if (auto *self = static_cast<UartTransport *>(transport->args)) {
      self->uart.init(self->baurate);
      return true;
    }

    return false;
  }

  static bool close(struct uxrCustomTransport *transport) {
    if (auto *self = static_cast<UartTransport *>(transport->args)) {
      self->uart.close();
      return true;
    }

    return false;
  }
  static size_t write(struct uxrCustomTransport *transport, const uint8_t *buf,
                      size_t len, uint8_t *err) {
    if (auto *self = static_cast<UartTransport *>(transport->args)) {
      self->uart.write(buf, len);
      return len;
    }

    return 0;
  }
  static size_t read(struct uxrCustomTransport *transport, uint8_t *buf,
                     size_t len, int timeout, uint8_t *err) {
    if (auto *self = static_cast<UartTransport *>(transport->args)) {
      return self->uart.read(buf, len, timeout);
    }

    return 0;
  }

private:
  T &uart;
  uint baurate;
};

#endif // TRANSPORT_HPP_
