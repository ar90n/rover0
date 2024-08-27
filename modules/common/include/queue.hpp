#pragma once

#include <array>
#include <cassert>
#include <iostream>
#include <new>
#include <optional>
#include <stdexcept>
#include <utility>

template <typename T, size_t Size> class FixedSizeQueue {
public:
  ~FixedSizeQueue() {
    while (!empty()) {
      pop();
    }
  }

  bool push(T const &value) {
    if (full_flag) {
      return false;
    }

    new (get(tail)) T(std::move(value));
    tail = (tail + 1) % Size;
    if (tail == head) {
      full_flag = true;
    }
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
    full_flag = false;
    --count;
    return result;
  }

  bool empty() const { return count == 0; }

  bool full() const { return full_flag; }

  size_t size() const { return count; }

private:
  alignas(T) std::array<std::byte, sizeof(T) * Size> data;
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;
  bool full_flag = false;

  T *get(size_t index) {
    return std::launder(reinterpret_cast<T *>(&data[index * sizeof(T)]));
  }

  const T *get(size_t index) const {
    return std::launder(reinterpret_cast<const T *>(&data[index * sizeof(T)]));
  }
};