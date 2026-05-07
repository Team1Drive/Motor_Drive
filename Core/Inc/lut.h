#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#ifndef LUT_ENABLE_SPECIAL_CASES
#define LUT_ENABLE_SPECIAL_CASES 0
#endif

#if defined(__GNUC__) || defined(__clang__)
#define LUT_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define LUT_ALWAYS_INLINE inline
#endif

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

namespace lut {

constexpr std::uint32_t LUT_POINTS = 4096U;

namespace detail {

static_assert((LUT_POINTS & (LUT_POINTS - 1U)) == 0U, "LUT_POINTS must be a power of two");

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = 0.5f * PI;
constexpr float INV_TWO_PI = 1.0f / TWO_PI;
constexpr float TRIG_SCALE = static_cast<float>(LUT_POINTS) * INV_TWO_PI;
constexpr float RATIO_SCALE = static_cast<float>(LUT_POINTS - 1U);
constexpr std::uint32_t LUT_MASK = LUT_POINTS - 1U;
constexpr std::uint32_t COS_OFFSET = LUT_POINTS / 4U;

struct Tables {
    float sin[LUT_POINTS];
    float atan[LUT_POINTS];
    float hypot[LUT_POINTS];
};

constexpr float abs_constexpr(float value) {
    return value < 0.0f ? -value : value;
}

constexpr float sin_constexpr(float radians) {
    while (radians > PI) {
        radians -= TWO_PI;
    }

    while (radians < -PI) {
        radians += TWO_PI;
    }

    if (radians > HALF_PI) {
        radians = PI - radians;
    } else if (radians < -HALF_PI) {
        radians = -PI - radians;
    }

    const float x2 = radians * radians;
    float term = radians;
    float result = term;

    term *= -x2 / (2.0f * 3.0f);
    result += term;
    term *= -x2 / (4.0f * 5.0f);
    result += term;
    term *= -x2 / (6.0f * 7.0f);
    result += term;
    term *= -x2 / (8.0f * 9.0f);
    result += term;
    term *= -x2 / (10.0f * 11.0f);
    result += term;

    return result;
}

constexpr float atan_series(float x) {
    const float x2 = x * x;
    float term = x;
    float result = term;

    term *= -x2;
    result += term / 3.0f;
    term *= -x2;
    result += term / 5.0f;
    term *= -x2;
    result += term / 7.0f;
    term *= -x2;
    result += term / 9.0f;
    term *= -x2;
    result += term / 11.0f;
    term *= -x2;
    result += term / 13.0f;
    term *= -x2;
    result += term / 15.0f;
    term *= -x2;
    result += term / 17.0f;

    return result;
}

constexpr float atan_constexpr(float ratio) {
    if (ratio <= 0.5f) {
        return atan_series(ratio);
    }

    return PI * 0.25f + atan_series((ratio - 1.0f) / (ratio + 1.0f));
}

constexpr float sqrt_constexpr(float value) {
    float estimate = value;

    for (std::uint32_t i = 0; i < 8U; ++i) {
        estimate = 0.5f * (estimate + value / estimate);
    }

    return estimate;
}

constexpr Tables makeTables() {
    Tables tables{};

    for (std::uint32_t i = 0; i < LUT_POINTS; ++i) {
        const float phase = static_cast<float>(i) * TWO_PI / static_cast<float>(LUT_POINTS);
        const float ratio = static_cast<float>(i) / static_cast<float>(LUT_POINTS - 1U);

        tables.sin[i] = sin_constexpr(phase);
        tables.atan[i] = atan_constexpr(ratio);
        tables.hypot[i] = sqrt_constexpr(1.0f + ratio * ratio);
    }

    return tables;
}

inline Tables TABLES __attribute__((section(".data.lut"), aligned(32))) = makeTables();

LUT_ALWAYS_INLINE float interpolate(const float* table, float scaled_index) {
    const auto index = static_cast<std::uint32_t>(scaled_index);
    if (index >= LUT_POINTS - 1U) {
        return table[LUT_POINTS - 1U];
    }

    const float fraction = scaled_index - static_cast<float>(index);
    const float a = table[index];
    const float b = table[index + 1U];

    return a + (b - a) * fraction;
}

LUT_ALWAYS_INLINE float interpolateWrapped(const float* table, float scaled_index) {
    auto base_index = static_cast<std::int32_t>(scaled_index);
    if (scaled_index < static_cast<float>(base_index)) {
        --base_index;
    }

    const auto index = static_cast<std::uint32_t>(base_index) & (LUT_POINTS - 1U);
    const auto next_index = (index + 1U) & (LUT_POINTS - 1U);
    const float fraction = scaled_index - static_cast<float>(base_index);
    const float a = table[index];
    const float b = table[next_index];

    return a + (b - a) * fraction;
}

LUT_ALWAYS_INLINE float interpolateTrigPositiveScaled(float scaled_index) {
    const auto index = static_cast<std::uint32_t>(scaled_index);
    const auto next_index = (index + 1U) & LUT_MASK;
    const float fraction = scaled_index - static_cast<float>(index);
    const float a = TABLES.sin[index];
    const float b = TABLES.sin[next_index];

    return a + (b - a) * fraction;
}

LUT_ALWAYS_INLINE float lookupTrigPositive(float radians) {
    return interpolateTrigPositiveScaled(radians * TRIG_SCALE);
}

LUT_ALWAYS_INLINE float lookupCosTrigPositive(float radians) {
    float scaled_index = radians * TRIG_SCALE + static_cast<float>(COS_OFFSET);

    if (scaled_index >= static_cast<float>(LUT_POINTS)) {
        scaled_index -= static_cast<float>(LUT_POINTS);
    }

    return interpolateTrigPositiveScaled(scaled_index);
}

LUT_ALWAYS_INLINE float lookupTrig(float radians) {
    return interpolateWrapped(TABLES.sin, radians * TRIG_SCALE);
}

LUT_ALWAYS_INLINE float lookupRatio(const float* table, float ratio) {
    if (ratio <= 0.0f) {
        return table[0];
    }

    if (ratio >= 1.0f) {
        return table[LUT_POINTS - 1U];
    }

    return interpolate(table, ratio * RATIO_SCALE);
}

LUT_ALWAYS_INLINE float absf(float value) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_fabsf(value);
#else
    return value < 0.0f ? -value : value;
#endif
}

LUT_ALWAYS_INLINE bool isfinitef(float value) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_isfinite(value);
#else
    return std::isfinite(value);
#endif
}

LUT_ALWAYS_INLINE bool isinff(float value) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_isinf(value);
#else
    return std::isinf(value);
#endif
}

LUT_ALWAYS_INLINE bool isnanf(float value) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_isnan(value);
#else
    return std::isnan(value);
#endif
}

LUT_ALWAYS_INLINE bool signbitf(float value) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_signbit(value);
#else
    return std::signbit(value);
#endif
}

} // namespace detail

LUT_ALWAYS_INLINE float sinf(float radians) {
#if LUT_ENABLE_SPECIAL_CASES
    if (!detail::isfinitef(radians)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
#endif

    if (radians >= 0.0f && radians < detail::TWO_PI) {
        return detail::lookupTrigPositive(radians);
    }

    return detail::lookupTrig(radians);
}

LUT_ALWAYS_INLINE float cosf(float radians) {
#if LUT_ENABLE_SPECIAL_CASES
    if (!detail::isfinitef(radians)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
#endif

    if (radians >= 0.0f && radians < detail::TWO_PI) {
        return detail::lookupCosTrigPositive(radians);
    }

    return detail::lookupTrig(radians + detail::HALF_PI);
}

LUT_ALWAYS_INLINE float sinf_0_2pi(float radians) {
    return detail::lookupTrigPositive(radians);
}

LUT_ALWAYS_INLINE float cosf_0_2pi(float radians) {
    return detail::lookupCosTrigPositive(radians);
}

LUT_ALWAYS_INLINE float atan2f(float y, float x) {
#if LUT_ENABLE_SPECIAL_CASES
    if (detail::isnanf(x) || detail::isnanf(y)) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    if (detail::isinff(y) && detail::isinff(x)) {
        const float ay = detail::signbitf(y) ? -1.0f : 1.0f;
        const float ax = detail::signbitf(x) ? -1.0f : 1.0f;
        return lut::atan2f(ay, ax);
    }

    if (detail::isinff(y)) {
        return detail::signbitf(y) ? -detail::HALF_PI : detail::HALF_PI;
    }

    if (detail::isinff(x)) {
        if (detail::signbitf(x)) {
            return detail::signbitf(y) ? -detail::PI : detail::PI;
        }

        return detail::signbitf(y) ? -0.0f : 0.0f;
    }
#endif

    if (x == 0.0f) {
        if (y > 0.0f) {
            return detail::HALF_PI;
        }

        if (y < 0.0f) {
            return -detail::HALF_PI;
        }

        return detail::signbitf(x) ? (detail::signbitf(y) ? -detail::PI : detail::PI)
                                   : (detail::signbitf(y) ? -0.0f : 0.0f);
    }

    if (y == 0.0f) {
        if (x < 0.0f) {
            return detail::signbitf(y) ? -detail::PI : detail::PI;
        }

        return detail::signbitf(y) ? -0.0f : 0.0f;
    }

    const float ax = detail::absf(x);
    const float ay = detail::absf(y);
    const bool steep = ay > ax;
    const float ratio = steep ? (ax / ay) : (ay / ax);
    float angle = detail::lookupRatio(detail::TABLES.atan, ratio);

    if (steep) {
        angle = detail::HALF_PI - angle;
    }

    if (x < 0.0f) {
        angle = detail::PI - angle;
    }

    return y < 0.0f ? -angle : angle;
}

LUT_ALWAYS_INLINE float hypotf(float x, float y) {
#if LUT_ENABLE_SPECIAL_CASES
    if (detail::isinff(x) || detail::isinff(y)) {
        return std::numeric_limits<float>::infinity();
    }

    if (detail::isnanf(x) || detail::isnanf(y)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
#endif

    float ax = detail::absf(x);
    float ay = detail::absf(y);

    if (ax < ay) {
        const float tmp = ax;
        ax = ay;
        ay = tmp;
    }

    if (ax == 0.0f) {
        return 0.0f;
    }

    return ax * detail::lookupRatio(detail::TABLES.hypot, ay / ax);
}

} // namespace lut

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#undef LUT_ALWAYS_INLINE
