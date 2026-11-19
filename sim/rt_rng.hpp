// rt_rng.hpp - portable random numbers and float math for the Robot Tuner engine.
//
// The upstream simulator uses std::mt19937 with std::uniform_real_distribution<float> and
// std::normal_distribution<float>. The ENGINE (mt19937) is specified by the C++ standard, but the
// DISTRIBUTIONS are not: libstdc++ (g++), libc++ (clang, Emscripten) and MSVC produce different numbers
// from the same seed. To keep the browser (WebAssembly, libc++) bit-identical to the native C++ build AND
// to the upstream simulator built with g++, the engine uses these exact copies of libstdc++'s algorithms:
//
//   uniform_real_distribution<float>(a, b):  a + (b - a) * generate_canonical<float, 24>(g)
//       generate_canonical with a 32-bit engine: float(g() - min) / 2^32, clamped below 1.
//   normal_distribution<float>(mean, sd):    Marsaglia polar method, the second value is SAVED in the
//       distribution object and returned by the next call (a fresh object discards it).
//
// tests/native/equiv_upstream.cpp checks these against the real libstdc++ objects.
//
// Float math: RT_PORTABLE_MATH 1 (the default, used by the WASM build and by its native reference build)
// evaluates sin/cos/exp/log/pow/atan2/... in double and rounds to float. glibc and musl (Emscripten) float
// functions are not guaranteed to round the same way; the double versions are far more accurate than a
// float ulp, so after rounding both platforms agree. RT_PORTABLE_MATH 0 calls the float std:: functions
// exactly like the upstream simulator (used by the upstream equivalence test).
#pragma once
#include <cmath>
#include <cstdint>
#include <random>

#ifndef RT_PORTABLE_MATH
#define RT_PORTABLE_MATH 1
#endif

namespace rt {

// ---- float math --------------------------------------------------------------------------------
#if RT_PORTABLE_MATH
inline float Cos(float x) { return (float)std::cos((double)x); }
inline float Sin(float x) { return (float)std::sin((double)x); }
inline float Exp(float x) { return (float)std::exp((double)x); }
inline float Log(float x) { return (float)std::log((double)x); }
inline float Pow(float a, float b) { return (float)std::pow((double)a, (double)b); }
inline float Atan2(float y, float x) { return (float)std::atan2((double)y, (double)x); }
inline float Atan(float x) { return (float)std::atan((double)x); }
inline float Tan(float x) { return (float)std::tan((double)x); }
inline float Hypot(float a, float b) { return (float)std::hypot((double)a, (double)b); }
#else
inline float Cos(float x) { return std::cos(x); }
inline float Sin(float x) { return std::sin(x); }
inline float Exp(float x) { return std::exp(x); }
inline float Log(float x) { return std::log(x); }
inline float Pow(float a, float b) { return std::pow(a, b); }
inline float Atan2(float y, float x) { return std::atan2(y, x); }
inline float Atan(float x) { return std::atan(x); }
inline float Tan(float x) { return std::tan(x); }
inline float Hypot(float a, float b) { return std::hypot(a, b); }
#endif
// sqrt, floor, fabs, lround are exact (IEEE) on every platform.

using Rng = std::mt19937;  // the engine itself is fully specified by the standard

// generate_canonical<float, 24>(g) as libstdc++ computes it for a 32-bit engine.
inline float Canonical(Rng& g) {
    float sum = (float)(uint32_t)(g() - Rng::min());
    float ret = sum / 4294967296.0f;
    if (ret >= 1.0f) ret = std::nextafter(1.0f, 0.0f);
    return ret;
}

// std::uniform_real_distribution<float>(a, b)(g)
inline float Uniform(Rng& g, float a, float b) { return Canonical(g) * (b - a) + a; }

// std::normal_distribution<float>: keep ONE object where upstream keeps one (the saved value matters).
struct Normal {
    float mean, sd;
    float saved = 0.0f;
    bool has_saved = false;
    Normal(float m, float s) : mean(m), sd(s) {}
    float operator()(Rng& g) {
        float ret;
        if (has_saved) {
            has_saved = false;
            ret = saved;
        } else {
            float x, y, r2;
            do {
                x = (float)(2.0f * Canonical(g) - 1.0);
                y = (float)(2.0f * Canonical(g) - 1.0);
                r2 = x * x + y * y;
            } while (r2 > 1.0 || r2 == 0.0);
            const float mult = std::sqrt(-2 * Log(r2) / r2);
            saved = x * mult;
            has_saved = true;
            ret = y * mult;
        }
        return ret * sd + mean;
    }
};

// A freshly constructed normal_distribution used for a single draw (upstream's common pattern).
inline float NormalOnce(Rng& g, float mean, float sd) { Normal n(mean, sd); return n(g); }


// Cheap integer mix (upstream train.cpp SeedMix), so nearby seeds do not give correlated streams.
inline unsigned int SeedMix(unsigned int a, unsigned int b) {
    unsigned int h = a * 0x9E3779B9u ^ (b + 0x85EBCA6Bu + (a << 6) + (a >> 2));
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
    return h;
}

}  // namespace rt
