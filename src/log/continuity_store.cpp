// ============================================================================
// continuity_store.cpp — see continuity_store.h. File plumbing only: every
// decision (which slot, what decodes, what the next sequence is) is
// logfmt::ContinuitySlots, which the native tests and the soak drive directly.
// ============================================================================
#include "continuity_store.h"

#include <LittleFS.h>

bool ContinuityStore::readFile() {
    File f = LittleFS.open(kPath, "r");
    if (!f) { fileOk_ = false; return false; }
    fileOk_ = true;
    uint8_t buf[kImageSize];
    const int n = f.read(buf, sizeof(buf));
    f.close();
    slots_.load(buf, n > 0 ? static_cast<size_t>(n) : 0u);
    return true;
}

bool ContinuityStore::begin() {
    // LittleFS is mounted by diveLogBoot(), which runs before this at boot (and
    // formats the partition once if the mount failed). If the partition is not
    // mounted at all, exists() is false and the honest answer is "no record",
    // which the boot banner says next to the `log:` line that already says the
    // partition itself is offline.
    if (!LittleFS.exists(kPath)) {
        slots_.clear();
        fileOk_ = false;
        present_ = false;
        return true;                       // no file is not a failure
    }
    present_ = true;
    return readFile();
}

bool ContinuityStore::ensureFile() {
    if (LittleFS.exists(kPath)) {
        File f = LittleFS.open(kPath, "r");
        if (f) {
            const unsigned long size = static_cast<unsigned long>(f.size());
            const bool rightSize = size == kImageSize;
            f.close();
            if (rightSize) return true;
            // A file of the wrong length is a format this build does not own
            // (a stride change, or something else wrote this name). Rebuilding
            // it costs the continuity record and nothing else, and it must not
            // be silent: the console says so at the moment it happens.
            Serial.printf("continuity: %s is %lu bytes, not %lu — rebuilding it "
                          "(the carried dive record is lost)\n",
                          kPath, size, (unsigned long)kImageSize);
            LittleFS.remove(kPath);
        }
    }
    File f = LittleFS.open(kPath, "w");
    if (!f) return false;
    uint8_t zeros[kImageSize];
    for (size_t i = 0; i < kImageSize; ++i) zeros[i] = 0;
    const bool ok = f.write(zeros, sizeof(zeros)) == (int)sizeof(zeros);
    f.flush();
    f.close();
    if (!ok) { LittleFS.remove(kPath); present_ = false; return false; }
    slots_.clear();
    fileOk_ = true;
    present_ = true;
    return true;
}

bool ContinuityStore::write(logfmt::ContinuityRecord& r) {
    if (!ensureFile()) return false;

    logfmt::ContinuityRecord rec = r;
    rec.seq = slots_.nextSeq();          // the records are the only durable counter
    const int slot = slots_.freeSlot();  // the OLDER slot: the newer one survives
                                         // a reset in the middle of this write
    uint8_t buf[logfmt::CONT_RECORD_SIZE];
    if (logfmt::packContinuity(rec, buf) != logfmt::CONT_RECORD_SIZE) return false;

    File f = LittleFS.open(kPath, "r+");   // r+, never w: a truncate would take
                                           // BOTH slots down to zero length
    if (!f) return false;
    bool ok = f.seek(static_cast<off_t>(slot) * logfmt::CONT_SLOT_STRIDE);
    if (ok) ok = f.write(buf, sizeof(buf)) == static_cast<int>(sizeof(buf));
    if (ok) f.flush();
    f.close();
    if (!ok) return false;

    // The RAM image follows the flash, it never leads it. `newest()` is what
    // seeds the next dive, and a seed that exists only in RAM would make this
    // boot and the boot after a reset disagree about the diver's tissues.
    slots_.write(slot, rec);
    fileOk_ = true;
    present_ = true;
    r.seq = rec.seq;   // stamped copy back to the caller (the surface store's
                       // rule: the seq only exists once the store assigns it)
    return true;
}

bool ContinuityStore::newest(logfmt::ContinuityRecord& out) const {
    return slots_.readNewest(out);
}

void ContinuityStore::eraseAll() {
    slots_.clear();
    fileOk_ = false;
    present_ = false;
    if (LittleFS.exists(kPath)) LittleFS.remove(kPath);
}
