#pragma once
#include <cstddef>

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
// their own clamps where the field is narrower than long long (an int16_t count
// of centimetres saturates at 327.67 m, which no dive reaches).
long long scaled(double v, int decimals);

} // namespace numfmt
