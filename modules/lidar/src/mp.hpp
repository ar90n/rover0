#ifndef MP_HPP_
#define MP_HPP_

#include <concepts>
#include <tuple>
#include <type_traits>

namespace mp {
namespace assoc_list {

template <typename T> constexpr bool always_false_v = false;

template <unsigned N, unsigned K, typename T> struct element_t {
  static constexpr unsigned n = N;
  static constexpr unsigned k = K;
  using value_type = T;
};

template <unsigned N, typename Tag> struct reader {
  constexpr friend auto element_func(reader<N, Tag>);
};

template <unsigned N, unsigned K, typename T, typename Tag> struct setter {
  constexpr friend auto element_func(reader<N, Tag>) {
    return std::make_tuple(K, T{});
  }

  static constexpr element_t<N, K, T> element{};
};

template <unsigned N, typename Tag, auto EvalTag> constexpr auto get_element() {
  constexpr auto KS = element_func(reader<N, Tag>{});
  return element_t<N, std::get<0>(KS), decltype(std::get<1>(KS))>{};
}

template <unsigned K, typename Tag, auto EvalTag, unsigned N = 0>
constexpr auto find_element_impl() {
  constexpr bool counted_past_n =
      requires(reader<N, Tag> r) { element_func(r); };

  if constexpr (counted_past_n) {
    constexpr auto KS = element_func(reader<N, Tag>{});
    if constexpr (std::get<0>(KS) == K) {
      return get_element<N, Tag, EvalTag>();
    } else {

      return find_element_impl<K, Tag, EvalTag, N + 1>();
    }
  } else {

    static_assert(always_false_v<Tag>, "invalid key");
  }
}

template <unsigned K, typename Tag, auto EvalTag = [] {},
          auto State = find_element_impl<K, Tag, EvalTag>()>
using find = typename std::remove_cvref_t<typename decltype(State)::value_type>;

template <typename Tag, auto EvalTag, unsigned N = 0>
[[nodiscard]]
consteval auto get_size() {
  constexpr bool counted_past_n =
      requires(reader<N, Tag> r) { element_func(r); };

  if constexpr (counted_past_n) {
    return get_size<Tag, EvalTag, N + 1>();
  } else {
    return N - 1;
  }
}

template <unsigned K, typename T, typename Tag, auto EvalTag, unsigned N = 0>
[[nodiscard]]
consteval auto append_impl() {
  constexpr bool counted_past_n =
      requires(reader<N, Tag> r) { element_func(r); };

  if constexpr (counted_past_n) {
    constexpr auto KS = element_func(reader<N, Tag>{});
    if constexpr (std::get<0>(KS) == K) {
      static_assert(always_false_v<Tag>, "duplicated key");
    } else {

      return append_impl<K, T, Tag, EvalTag, N + 1>();
    }
  } else {
    setter<N, K, T, Tag> s;
    return s.element;
  }
}

template <unsigned K, typename T, typename Tag, auto EvalTag = [] {},
          auto State = append_impl<K, T, Tag, EvalTag>()>
constexpr auto append = [] { return State; };

} // namespace assoc_list
} // namespace mp

#endif // MP_HPP_