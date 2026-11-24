// ============================================================================
// depth_announce.h — what the boot console says about the depth source.
//
// Why this is a state machine and not a print at the moment the driver locks:
// `Lps28::begin()`/`Ms5837::begin()` report READY as soon as the chip has been
// IDENTIFIED (WHO_AM_I / PROM CRC), but the first pressure conversion comes
// later — the read is paced by the driver's own sample window. An announce
// wired to the lock therefore prints a placeholder `0.0 hPa`, and that line is
// the bench's evidence for "which sensor, at which address, reading what"
// (BENCH.md C3-a). A number that is always zero is worse than no number: it
// reads like a measurement.
//
// So the announce waits for the first sample the CONTROL PATH would actually
// use — the caller passes `saneSample`, the same gate main.cpp applies before
// deriving depth, so there is one rule about what counts as a reading and it
// cannot drift. And a source that identifies but never produces one says so out
// loud: "no sensor" and "sensor present, no data" are different facts, and the
// boot banner is the instrument that tells them apart (same reasoning as the
// surface log's `no records yet` vs `** will not read`).
//
// Pure: no Wire, no Serial, no Arduino header — unit-tested on the host, and
// the exact bytes are pinned there.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace press {

class Announcer {
public:
    /// How long a locked source gets to produce one sane reading before the
    /// console says it is identified but deaf. Generous against the drivers'
    /// own timing (a fitted sensor's first conversion is paced by a ~100 ms
    /// sample window) and still far inside the boot window the bench captures.
    static constexpr uint32_t NO_DATA_GRACE_MS = 3000;

    enum class Action : uint8_t { NONE, READY, NO_DATA };

    /// One call per sensor cycle. `locked` = a depth source owns depth;
    /// `saneSample` = this cycle produced an absolute pressure the control path
    /// used; `pressureBar` = that cycle's reading (ignored unless a line is
    /// printed; pass 0 when unlocked). Returns the action to print, if any.
    ///
    /// Each kind prints once per boot. NO_DATA first and a real reading later
    /// still prints READY, because the record should end on the truth.
    Action update(uint32_t nowMs, bool locked, bool saneSample, float pressureBar);

    /// The exact line for an action, into out/cap. Returns bytes written, or 0
    /// for NONE and for a buffer too small — a truncated boot line is a boot
    /// line a reader misreads.
    size_t line(Action action, const char* identity, char* out, size_t cap) const;

private:
    bool lockSeen_ = false;       // a source has owned depth at least one cycle
    uint32_t lockSeenMs_ = 0;     // when that first happened
    float lastBar_ = 0.0f;        // last reading seen, sane or not
    bool readyDone_ = false;      // READY line already printed
    bool noDataDone_ = false;     // NO_DATA line already printed
};

}  // namespace press
