// ============================================================================
// contgate.h — who writes a continuity record, and who must not (review H1,
// under review H2's discipline).
//
// The record has to be taken from the deco engine, and the deco engine belongs
// to ControlTask (rowfill.h N8: control-thread ownership, every other reader
// takes a copy). The record then has to reach FLASH, and flash is behind the
// same lock a multi-second log export holds. So the two halves are split:
//
//   * post() — ControlTask. A struct copy under the gate's own mutex, ~90 bytes,
//     and NOTHING else: no store call, no lock the command task holds, no flash.
//     Deliberately stricter than GridGate (rowgate.h), which retries its row on
//     the control task with a zero wait: a dive-log grid row is 36 bytes the
//     control task already owns and was cheap to attempt inline, while a
//     continuity record's write is a LittleFS commit, and the task carrying the
//     1500 ms stall budget has no business being inside one. ControlTask
//     publishes; it never writes.
//   * drain() — a LOWER-priority task (HudTask on the board, the rig in the
//     simulator). That is where the bounded store wait and the flash commit
//     live, and HudTask's stall budget is 3000 ms, twice ControlTask's. The
//     store is called OUTSIDE the gate's mutex: the mutex is held only to copy
//     a record in or out, which is what makes a control-task wait on it
//     impossible in practice (the only other holder is a lower-priority task on
//     the same core).
//
// What a pending record costs when a newer one arrives before it was written:
// the older snapshot is LOST, and lostSnapshots() counts it — the HB `contskip=`
// field. For a restore source the newer record is the better one anyway (it is
// closer to the diver's actual tissues), but "better" is not "written": the
// counter is how the bench learns the store is not keeping up.
//
// Crash-window meaning, which is what the write cadence is sized against:
// restore always uses the NEWEST WRITTEN record, so the worst a power cut can
// produce is a picture one write period old, plus the torn write itself — which
// the two-slot layout absorbs (continuity.h: the older slot stays valid), and
// which the next periodic snapshot replaces on the next drain.
// ============================================================================
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>

#include "log/continuity.h"

namespace logfmt {

class ContinuityGate {
public:
    enum Result : uint8_t {
        WRITTEN = 0,   // the store took the record
        BUSY = 1,      // the store lock was not granted inside the budget
        FAILED = 2,    // the store took the lock and still could not write
    };
    /// The seam the native tests drive, exactly as GridGate's appender is: the
    /// firmware's version wraps a bounded xSemaphoreTake + ContinuityStore::write;
    /// the tests substitute a store that lies about being busy. It is handed the
    /// record the gate is holding, so what gets written is exactly what was
    /// claimed, and the writer runs with no gate lock held.
    using Writer = std::function<Result(const ContinuityRecord&, uint32_t budgetMs)>;

    void setWriter(Writer w) { writer_ = std::move(w); }

    /// ControlTask: a snapshot is due. A copy, and never the writer. A pending
    /// record that never got written and is now displaced is LOST and counted.
    /// (Sequence numbers are assigned by the store at write time — they are the
    /// durable counter the slots themselves carry — so the gate cannot compare
    /// them and does not try.)
    void post(const ContinuityRecord& r) {
        std::lock_guard<std::mutex> g(mux_);
        if (due_) lost_++;      // displaced and never written: a fact, not a shrug
        pending_ = r;
        due_ = true;
        gen_++;
        posts_++;
    }

    /// The lower-priority task: write what is pending, with `budgetMs` as the
    /// caller's bound on waiting for the STORE lock (0 = try-lock). The writer
    /// runs with no gate lock held.
    Result drain(uint32_t budgetMs) {
        ContinuityRecord copy{};
        uint32_t gen = 0;
        {
            std::lock_guard<std::mutex> g(mux_);
            if (!due_) return WRITTEN;    // nothing to do is not a failure
            copy = pending_;
            gen = gen_;
        }
        attempts_++;
        const Result r = writer_ ? writer_(copy, budgetMs) : FAILED;
        if (r == WRITTEN) {
            std::lock_guard<std::mutex> g(mux_);
            written_++;
            // Clear the slot only if nothing newer arrived while the flash write
            // was running. If it did, that record is still pending — and this
            // write does not count for it.
            if (gen_ == gen) due_ = false;
        } else if (r == FAILED) {
            // The store took the lock and still could not write. Retrying a
            // refusal is not going to make the flash work, and keeping a stale
            // snapshot in the only slot means newer ones queue behind a write
            // that is never coming. Drop it, and count it where the bench can
            // see it (HB `contskip=`).
            std::lock_guard<std::mutex> g(mux_);
            if (gen_ == gen) due_ = false;
            lost_++;
        }
        return r;
    }

    bool pending() const {
        std::lock_guard<std::mutex> g(mux_);
        return due_;
    }
    ContinuityRecord pendingRecord() const {
        std::lock_guard<std::mutex> g(mux_);
        return pending_;
    }

    /// Snapshots that came due and never reached the store. The HB `contskip=`.
    uint32_t lostSnapshots() const { return lost_; }
    /// Diagnostics: how many posts arrived, how many writes happened and how
    /// many attempts they needed.
    uint32_t posted() const { return posts_; }
    uint32_t writtenThroughGate() const { return written_; }
    uint32_t retryCount() const { return attempts_; }

private:
    Writer writer_;
    mutable std::mutex mux_;
    ContinuityRecord pending_{};
    bool due_ = false;
    uint32_t gen_ = 0;
    uint32_t lost_ = 0;
    uint32_t posts_ = 0;
    uint32_t written_ = 0;
    uint32_t attempts_ = 0;
};

} // namespace logfmt
