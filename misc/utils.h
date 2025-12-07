#pragma once

#include <utility>
#include <type_traits>

template <int... indices, typename Func>
inline void static_for(const Func& func, std::integer_sequence<int, indices...> sequence) {
    (func(indices), ...);
}

template <int N, typename Func>
inline void static_for(const Func& func) {
    static_for(func, std::make_integer_sequence<int, N>{});
}
