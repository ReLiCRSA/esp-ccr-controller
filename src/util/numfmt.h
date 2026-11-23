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

} // namespace numfmt
