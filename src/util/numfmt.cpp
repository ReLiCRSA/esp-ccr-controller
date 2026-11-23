// ============================================================================
// numfmt.cpp — see numfmt.h for why this file exists (float printf blew the
// uart2 task's stack; digits here cost tens of bytes and cannot grow).
// ============================================================================
#include "util/numfmt.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace numfmt {

namespace {

constexpr int kMaxDecimals = 6;

long long ipow10(int d) {
    long long r = 1;
    for (int i = 0; i < d; ++i) r *= 10;
    return r;
}

char* empty(char* buf) {
    buf[0] = '\0';
    return buf;
}

// Shared digit writer: '-' if neg, then whole, then '.' + `decimals` digits of
// frac (frac is in [0, 10^decimals)). Refuses rather than emitting a partial
// number, because a truncated ppO2 field is worse than a missing line.
char* render(char* buf, std::size_t cap, bool neg, long long whole, long long frac,
             int decimals) {
    if (cap == 0) return buf;
    char tmp[32];                        // sign + 20 digits + '.' + 6 + NUL
    std::size_t n = 0;
    if (neg) tmp[n++] = '-';

    char digits[21];
    std::size_t dn = 0;
    long long w = whole;
    if (w == 0) {
        digits[dn++] = '0';
    } else {
        char rev[21];
        std::size_t rn = 0;
        while (w > 0) { rev[rn++] = static_cast<char>('0' + (w % 10)); w /= 10; }
        while (rn > 0) digits[dn++] = rev[--rn];
    }
    if (n + dn > sizeof(tmp) - 1) return empty(buf);
    std::memcpy(tmp + n, digits, dn);
    n += dn;

    if (decimals > 0) {
        tmp[n++] = '.';
        char fbuf[kMaxDecimals];
        long long f = frac;
        for (int i = decimals - 1; i >= 0; --i) {
            fbuf[i] = static_cast<char>('0' + (f % 10));
            f /= 10;
        }
        if (n + static_cast<std::size_t>(decimals) > sizeof(tmp) - 1) return empty(buf);
        std::memcpy(tmp + n, fbuf, static_cast<std::size_t>(decimals));
        n += static_cast<std::size_t>(decimals);
    }
    if (cap < n + 1) return empty(buf);
    tmp[n] = '\0';
    std::memcpy(buf, tmp, n + 1);
    return buf;
}

} // namespace

char* fixedScaled(char* buf, std::size_t cap, long long scaled, int decimals) {
    if (!buf || cap == 0) return buf;   // nothing can be stored: caller error
    empty(buf);
    if (decimals < 0) decimals = 0;
    if (decimals > kMaxDecimals) decimals = kMaxDecimals;
    const long long scale = ipow10(decimals);
    const bool neg = scaled < 0;
    const long long mag = neg ? -scaled : scaled;
    if (mag > std::numeric_limits<long long>::max() / 2) return empty(buf);
    return render(buf, cap, neg, mag / scale, mag % scale, decimals);
}

char* fixed(char* buf, std::size_t cap, double v, int decimals) {
    if (!buf || cap == 0) return buf;  // nothing can be stored: caller error
    empty(buf);
    if (decimals < 0) decimals = 0;
    if (decimals > kMaxDecimals) decimals = kMaxDecimals;

    if (std::isnan(v)) {
        if (cap < 4) return empty(buf);
        std::memcpy(buf, "nan", 4);
        return buf;
    }

    const bool neg = std::signbit(v);          // also true for -0.0, like printf
    if (std::isinf(v)) {
        const char* s = neg ? "-inf" : "inf";
        const std::size_t n = std::strlen(s);
        if (cap < n + 1) return empty(buf);
        std::memcpy(buf, s, n + 1);
        return buf;
    }

    const double scale = static_cast<double>(ipow10(decimals));
    const double mag = std::fabs(v);

    // Beyond this the scaled value cannot be held, and no float fallback is
    // allowed (that is the whole point of this file). Refuse instead of lying.
    if (!(mag * scale < static_cast<double>(std::numeric_limits<long long>::max() / 8))) {
        return empty(buf);
    }

    // printf rounds the exact binary value to nearest, ties to EVEN. llround()
    // rounds ties away from zero and would print a different byte on a tie, so
    // the tie rule is spelled out here: 0.125 at %.2f is "0.12", 1.5 at %.0f is
    // "2" (1 is odd), 0.5 at %.0f is "0" (0 is even).
    //
    // The scaling multiply must be EXACT, not merely rounded, or the tie rule
    // gets applied to a value that was never a tie: 0.0105 is stored slightly
    // ABOVE 0.0105, but `0.0105 * 1000.0` rounds to exactly 10.5, so a naive
    // implementation sees a tie and emits "0.010" where printf says "0.011" — a
    // ppO2 field off by one digit, for values that happen to land there
    // (test_awkward_step_values found 40 under 0.4). fma() supplies the exact
    // residual of the product so the comparison below decides on the true value.
    const double hi = mag * scale;
    const double lo = std::fma(mag, scale, -hi);   // exact product is hi + lo
    double fl = std::floor(hi);
    double r = (hi - fl) + lo;
    if (r < 0.0) { fl -= 1.0; r += 1.0; }          // hi rounded up past an integer
    else if (r >= 1.0) { fl += 1.0; r -= 1.0; }
    long long scaled = static_cast<long long>(fl);
    if (r > 0.5) scaled += 1;
    else if (r == 0.5 && (scaled & 1)) scaled += 1;

    return render(buf, cap, neg, scaled / static_cast<long long>(scale),
                  scaled % static_cast<long long>(scale), decimals);
}

} // namespace numfmt
