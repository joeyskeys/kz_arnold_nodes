#pragma once

#include <array>
#include <algorithm>

#include <ai.h>

#include "utils.h"

template <typename T>
struct vec3 {
    union {
        struct {
            T x, y, z;
        };
        std::array<T, 3> data;
    }

    // ctors
    constexpr vec3() {
        data.fill(static_cast<T>(0));
    }

    template <typename ...Ts>
    constexpr VT(Ts... args) {
        static_assert(sizeof...(args) == 1 || sizeof...(args) == 3, "vec3 constructor requires 1 or 3 arguments");
        if constexpr (sizeof...(args) == 1) {
            data.fill(static_cast<T>(args...));
        } else {
            data = { static_cast<T>(args)... };
        }
    }

    vec3(const VT&) = default;
    vec3(VT&&) = default;
    vec3(const AtRGB& c) : x(static_cast<T>(c.r)), y(static_cast<T>(c.g)), z(static_cast<T>(c.b)) {}

    // ops
    #define DEFINE_VEC3_OP(OP) \
    const vec3 operator OP(const vec3& other) const { \
        vec3 ret; \
        static_for<3>([&](const auto i) { \
            ret.data[i] = this->data[i] OP other.data[i]; \
        }); \
        return ret; \
    }

    #define DEFINE_VEC3_OP_ASSIGN(OP) \
    vec3& operator OP##=(const vec3& other) { \
        static_for<3>([&](const auto i) { \
            this->data[i] OP##= other.data[i]; \
        }); \
        return *this; \
    }

    #define DEFINE_VEC3_OP_SCALAR(OP) \
    const vec3 operator OP(const T& scalar) const { \
        vec3 ret; \
        static_for<3>([&](const auto i) { \
            ret.data[i] = this->data[i] OP scalar; \
        }); \
        return ret; \
    }

    #define DEFINE_VEC3_OP_ASSIGN_SCALAR(OP) \
    vec3& operator OP##=(const T& scalar) { \
        static_for<3>([&](const auto i) { \
            this->data[i] OP##= scalar; \
        }); \
        return *this; \
    }

    #define DEFINE_VEC3_OP_SCALAR_FRIEND(OP) \
    friend const vec3 operator OP(const T& scalar, const vec3& v) { \
        vec3 ret; \
        static_for<3>([&](const auto i) { \
            ret.data[i] = scalar OP v.data[i]; \
        }); \
        return ret; \
    }

    #define DEFINE_VEC3_OP_WITH_ASSIGN(OP) \
    DEFINE_VEC3_OP(OP) \
    DEFINE_VEC3_OP_ASSIGN(OP) \
    DEFINE_VEC3_OP_SCALAR(OP) \
    DEFINE_VEC3_OP_ASSIGN_SCALAR(OP) \
    DEFINE_VEC3_OP_SCALAR_FRIEND(OP)

    DEFINE_VEC3_OP_WITH_ASSIGN(+)
    DEFINE_VEC3_OP_WITH_ASSIGN(-)
    DEFINE_VEC3_OP_WITH_ASSIGN(*)
    DEFINE_VEC3_OP_WITH_ASSIGN(/)
};

#define VEC_MAP(FUNC) \
template <typename T> \
inline vec3<T> FUNC(const vec3<T>& v) { \
    vec3<T> ret; \
    static_for<3>([&](const auto i) { \
        ret.data[i] = FUNC(v.data[i]); \
    }); \
    return ret; \
}

#define VEC_MAP2(FUNC) \
template <typename T> \
inline vec3<T> FUNC(const vec3<T>& v1, const vec3<T>& v2) { \
    vec3<T> ret; \
    static_for<3>([&](const auto i) { \
        ret.data[i] = std::FUNC(v1.data[i], v2.data[i]); \
    }); \
    return ret; \
}

VEC_MAP(sqrt)
VEC_MAP2(max)
VEC_MAP2(pow)

using vec3f = vec3<float>;