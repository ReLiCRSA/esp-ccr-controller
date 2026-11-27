#pragma once
#include <cstddef>
#include <limits>

// ============================================================================
// numfmt — fixed-point number rendering, because float printf is a stack bomb
//
// newlib's printf("%f") reaches _dtoa_r through cvt/_vsnprintf_r, and that chain
// wants several kilobytes of stack. Every number this device puts on the handset
// wire, in a dive log, or on the console goes through it, on tasks whose stacks
// are fixed and small. That is not theoretical: on 2026-11-23 the first `GET T`
// from a real handset tripped the FreeRTOS stack canary in the uart2 task
// (6144 B, the largest stack on the board) inside _dtoa_r, and the supervisor
// dropped the controller into SAFE mode mid-bench. The 2 Hz downlink survived
// because it reaches emit() through a shallower call chain than a routed
// command does — the margin was one stack frame.
//
// So: no float format specifier anywhere in a bounded-stack task. Digits here
// cost tens of bytes and cannot grow.
//
// Output is meant to be byte-identical to printf("%.Nf") for the values this
// device carries, including round-half-to-even on exact ties (0.125 at %.2f is
// "0.12", not "0.13" — llround would give the wrong digit), the "-0.00" printf
// prints for a small negative, and nan/inf spelled as strtod expects.
// ============================================================================
namespace numfmt {

// Renders v into buf with `decimals` places.
//
// Returns buf. If the whole number does not fit in cap, buf is set to "" and
// the caller's existing truncation guard drops the whole line (cap == 0 writes
// nothing at all: no byte can be stored, so that is a caller error) — a missing
// telemetry line is survivable, a plausible-looking wrong ppO2 is not. The
// same "" is returned for a value too large for fixed digits, which cannot
// happen for the physical quantities here (depth, bar, volts, minutes, CNS %)
// but is stated rather than assumed.
char* fixed(char* buf, std::size_t cap, double v, int decimals);

// Renders an integer that is ALREADY scaled: fixedScaled(buf, cap, 1234, 2)
// writes "12.34". The dive log stores depth as cm and ppO2 as thousandths, so
// that path has no float in it at all — exact, and cheaper than the divide it
// replaces. It also fixes a hand-rolled format in dive_store.cpp that printed a
// negative value as "-5.-30" (both halves carry the sign).
char* fixedScaled(char* buf, std::size_t cap, long long scaled, int decimals);

// The one way to turn a physical value into a stored integer: `scaled(v, d)`
// stores v in units of 10^-d, so a field written with 3 decimals is stored with
// scaled(v, 3) and read back with fixedScaled(..., 3). Same `d` on both sides,
// and nothing else.
//
// This exists because a plain cast truncates toward zero, and a decimal that is
// not exact in binary can land just BELOW the integer it should be: 0.29f is
// 0.28999999..., and 20.30f * 100.0f is 2029.999878, so (int32_t)(depth * 100.0f)
// stored 2029 cm for a depth the same build printed as 20.30. Multiplying by a
// DOUBLE literal is worse — the float is promoted and more values fall short:
// ceiling 11.15 m times 100.0 is 1114.999962, and 0.21f, 0.29f, 0.35f, 0.45f,
// 0.70f, 11.9f and 16.8f all lose a count that way. Measured on 2026-11-24, when
// the dive file was found to be one least-significant-digit away from the wire it
// is supposed to match.
//
// The rounding is deliberately the SAME rule fixed() uses — nearest, ties to
// even, decided on the exact product (fma) rather than the rounded one — because
// the invariant that matters is:
//
//     fixedScaled(buf, cap, scaled(v, d), d)  ==  fixed(buf, cap, v, d)
//
// byte for byte. A stored value must never disagree with the number the handset
// was answered for the same sample; inventing a second tie rule here would put a
// digit between the two. (That is why this is NOT lround(): lround rounds ties
// away from zero and fixed() does not, so the pair would differ on a tie.)
//
// Returns 0 only for a non-finite value or one too large to hold. Callers keep
// their own clamps where the field is narrower than long long — and that clamp
// is scaledInto below, never a static_cast.
long long scaled(double v, int decimals);

// Narrowing an INTEGER into a type that may not hold it: saturates instead of
// wrapping. The same rule as scaledInto below, for the values that are already
// integers — the dive summary's extremes and counters, where the accumulation
// is wider than the column it lands in (`totalPulses` counts a dive's injections
// in uint32 and exports a uint16; a wrap there says "a quarter of the injections
// happened", which is the same class of lie as a wrapped ceiling).
template <class T>
inline T narrow(long long v) {
    const long long hi = static_cast<long long>(std::numeric_limits<T>::max());
    const long long lo = static_cast<long long>(std::numeric_limits<T>::min());
    if (v > hi) return std::numeric_limits<T>::max();
    if (v < lo) return std::numeric_limits<T>::min();
    return static_cast<T>(v);
}

// scaled() fits in long long. A log field usually does not: ceiling, NDL, deco
// minutes, stop depths, ppO2 and battery are int16 or uint16, and a plain
// static_cast of an out-of-range long long is a WRAP, not a refusal — 95336 cm
// of ceiling becomes 29800, and 32768 becomes -32768. That is not a cosmetic
// overflow: `ceiling_m` and `ceiling_cm` are read as "0 or negative = no
// decompression obligation" by the handset, the web page and the dive file, so
// a wrapped ceiling tells a diver he may ascend.
//
// Saturation is the only narrowing that cannot make an obligation disappear.
// An unrepresentable value becomes the largest number the field holds (or the
// smallest, for a negative one), which keeps the diver MORE obligated than the
// truth rather than less, and it is at least a number a client can recognise as
// the field's own ceiling instead of a plausible reading from a different dive.
//
// No dive reaches this. The engine's ceiling is bounded by the deepest tissue
// load it will accept (deco::kSeedMaxBar = 100 bar, about 950 m), which a real
// dive at the deepest legal depth (~16 bar) cannot come near, so the only way
// here is a continuity record that is well-formed and wrong — a future format
// or unit mistake. The guard is the safety net for that case, and the native
// test feeds exactly that seed (test_controlplane).
//
// A non-finite input takes scaled()'s 0, which is a refusal, not a claim: the
// engine gates its inputs (setTissueState rejects non-finite, schedule() is
// bounded twice), so nothing that reaches here is a NaN.
template <class T>
inline T scaledInto(double v, int decimals) { return narrow<T>(scaled(v, decimals)); }

} // namespace numfmt
