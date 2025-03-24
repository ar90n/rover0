#pragma once

#include <array>
#include <cassert>
#include <iostream>
#include <new>
#include <optional>
#include <stdexcept>
#include <utility>

template <typename T, size_t Size>
class FixedSizeQueue {
public:
  ~FixedSizeQueue() {
    while (!empty()) {
      pop();
    }
  }

  bool push(T const &value) {
    if (count == Size) {  // キューが満杯かどうかは count で判断
      return false;
    }
    new (get(tail)) T(std::move(value));
    tail = (tail + 1) % Size;
    ++count;
    return true;
  }

  std::optional<T> pop() {
    if (count == 0) {
      return std::nullopt;
    }
    std::optional<T> result(*get(head));
    get(head)->~T();
    head = (head + 1) % Size;
    --count;
    return result;
  }

  bool empty() const { return count == 0; }
  bool full() const { return count == Size; }
  size_t size() const { return count; }

private:
  alignas(T) std::array<std::byte, sizeof(T) * Size> data;
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  T *get(size_t index) {
    return std::launder(reinterpret_cast<T *>(&data[index * sizeof(T)]));
  }
  const T *get(size_t index) const {
    return std::launder(reinterpret_cast<const T *>(&data[index * sizeof(T)]));
  }
};
