// ============================================================================
// rowgate.h — the dive-log grid row's journey when the log is BUSY, so that
// ControlTask never waits on it (review H2).
//
// The 30 s grid row is appended BY ControlTask (single-owner file writes: the
// command task opens/closes the file, the control task only ever appends the
// grid). The store lives behind a mutex the handset/web command task also
// takes — and a log EXPORT holds that mutex across a multi-second CSV pass.
// Until 2026-11-26 ControlTask took it with portMAX_DELAY: a diver browsing
// his dive list while diving could stall the control loop, and the supervisor
// rebooted the controller for it (1500 ms stall budget). The row was never
// worth a control stall. Not one.
//
// The policy, host-tested here and wired by session_wiring.cpp:
//   * a due row is posted to the store with a SMALL bounded wait; if the lock
//     is not granted inside the budget the row is NOT dropped — it becomes the
//     one pending row, and every later control cycle retries the append with
//     a zero wait (try-lock). ControlTask's wait for the log mutex is thereby
//     bounded by the post budget, full stop; retries cost a try-lock per beat.
//   * when a NEW row comes due while an old one is still pending, the pending
//     row is displaced: that row is LOST, lostRows() counts it, and the HB
//     `logskip=` field publishes the count. A lost row is a fact, and the
//     machine says it out loud instead of quietly editing the dive.
//   * a row that does get written is written INTACT and carries its own due
//     timestamp, not the retry time: the file may be late, it is never wrong.
//     The torn-write/CRC contract is untouched — this module never touches
//     bytes, only WHEN the append is attempted.
//
// What a reader of the dive file can tell: every row timestamps itself, so
// missing 30 s instants are visible as gaps in the column, and `M,`'s snapshot
// count comes from the store (the object that wrote rows), so the file's own
// summary never over-claims. What the file CANNOT tell is WHY a gap exists —
// a stall-defeat, a torn tail, a log that was off: those read the same from
// inside. The HB counter is the cross-check for that, and it is why this
// counter exists at all.
// ============================================================================
#pragma once

#include <cstdint>
#include <functional>

#include "log/rowfill.h"

namespace logfmt {

class GridGate {
public:
    enum Result : uint8_t {
        WRITTEN = 0,   // append completed under the bounded wait
        BUSY = 1,      // lock not granted inside the budget: nothing was written
    };
    // The seam the native tests drive: attempt the append with at most
    // budgetMs of waiting and report WRITTEN / BUSY. The firmware's version
    // wraps a bounded xSemaphoreTake + DiveStore::append; the tests substitute
    // a store that lies about being busy.
    using Appender = std::function<Result(uint32_t budgetMs)>;

    void setAppender(Appender a) { appender_ = std::move(a); }

    // A row just came due. `snap` is final: whoever retries writes THESE bytes.
    void post(uint32_t nowMs, const Snapshot& snap) {
        flushPending();                     // the older row gets its last shot
        pending_ = snap;
        pendingDue_ = true;
        if (appender_ && appender_(firstBudgetMs_) == WRITTEN) pendingDue_ = false;
        (void)nowMs;  // the timestamp already lives in the snapshot; named for
                      // the caller's sake: a post is a due-event, not a write
    }

    // Every control cycle: retry the pending append with a zero wait. Costs
    // one try-lock; never blocks.
    void retry() {
        if (!pendingDue_ || !appender_) return;
        retries_++;
        if (appender_(0u) == WRITTEN) {
            pendingDue_ = false;
            flushed_++;
        }
    }

    // End-of-dive (or any moment worth one last bounded attempt): the pending
    // row belongs to the dive that is closing, so the closer flushes it with a
    // real budget on the command task — where waiting is allowed. If even
    // that cannot get the lock, the row is lost and counted.
    void flush(uint32_t budgetMs) {
        if (pendingDue_ && appender_ && appender_(budgetMs) == WRITTEN) {
            pendingDue_ = false;
            flushed_++;
        }
        flushPending();
    }

    // The closer wrote the pending row itself (inside the dive-close lock,
    // where waiting is allowed). Same fact as WRITTEN: the row is in the
    // file, and the count columns come from the store. Claiming it here is
    // what keeps a flush-at-close from later being miscounted as a loss.
    void claimPendingWritten() {
        if (!pendingDue_) return;
        pendingDue_ = false;
        flushed_++;
    }

    bool pending() const { return pendingDue_; }
    const Snapshot& pendingRow() const { return pending_; }

    // Rows that came due and never made it into the file. THE number the HB
    // line publishes as `logskip=`.
    uint32_t lostRows() const { return lost_; }
    // Diagnostics only: how many retries a row needed, how many made it.
    uint32_t retryCount() const { return retries_; }
    uint32_t writtenThroughGate() const { return flushed_; }

    void setFirstAttemptBudgetMs(uint32_t ms) { firstBudgetMs_ = ms; }

private:
    void flushPending() {
        if (pendingDue_) {
            lost_++;       // displaced by a newer row: a fact, not a shrug
            pendingDue_ = false;
        }
    }

    Appender appender_;
    Snapshot pending_{};
    bool pendingDue_ = false;
    uint32_t lost_ = 0;
    uint32_t retries_ = 0;
    uint32_t flushed_ = 0;
    uint32_t firstBudgetMs_ = 2;
};

}  // namespace logfmt
