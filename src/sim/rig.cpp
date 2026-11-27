// ============================================================================
// rig.cpp — see rig.h. All control behaviour lives in ctrl::Loop; this file is
// only the bolt between it and the gas model.
// ============================================================================
#include "sim/rig.h"

namespace sim {
namespace {
constexpr float kTickS = Rig::kTickMs / 1000.0f;
}

Rig::Rig(const RigOpts& opts)
    : opts_(opts), world_(opts.world), loop_(opts), vbat_(opts.vbat),
      vbatMid_(opts.vbatMid) {}

void Rig::setCellsFitted(uint8_t mask) {
    opts_.presentMask = mask;
    loop_.setConfig(opts_);
}

void Rig::tick() {
    const uint32_t now = tickMs_ + kTickMs;
    tickMs_ = now;

    // ---- physics advances under the valve state commanded last cycle -------
    // A stuck-open solenoid flows whether or not the controller asks: the
    // control chain sees the consequence (ppO2 rising, no command), which is
    // exactly how it has to detect it.
    world_.step(kTickS, lastSolOn_);

    // ---- the same SensorData the firmware would fill in --------------------
    ctrl::SensorData sd;
    for (int i = 0; i < 3; i++)
        sd.cells[i] = world_.cellValue(static_cast<uint8_t>(i), kTickS);
    // The depth leg of the world can fail now (World::DepthFault), and the way it
    // fails is the way the driver fails on the board: the last measured sample
    // keeps being delivered and the health flag goes false. Not 0.0f — that
    // substitution IS the C1 bug, and SensorData.depthM in control/loop.h now
    // says it is forbidden. The physics never follows the fault: the world stays
    // as deep as the profile says (and keeps burning SAC at that pressure),
    // which is the whole point of injecting it.
    sd.depthM = world_.depthReadingM();
    sd.pressureValid = world_.depthValid();
    sd.cellsFresh = true;
    sd.vbat = vbat_;
    sd.vbatMid = vbatMid_;

    const ctrl::State st = loop_.cycle(sd, now);
    lastState_ = st;  // published state, for state() — see rig.h
    lastSolOn_ = st.solenoid || world_.solenoidStuckOpen;

    // ---- running envelope, for scenario asserts ----------------------------
    // Only while the vote is real: a cold start (no samples yet) or a failed
    // sensor set legitimately reads 0 and must not pollute the dive envelope.
    if (st.haveVote) {
        // The envelope counts MEASUREMENTS: st.shownPpo2 is the held display
        // number, and a held value must not be counted again as if it were fresh.
        if (st.shownPpo2 > maxVotedSeen) maxVotedSeen = st.shownPpo2;
        if (st.shownPpo2 < minVotedSeen) minVotedSeen = st.shownPpo2;
    }
    const float truePpo2 = world_.ppo2True();
    if (truePpo2 > maxTrueSeen) maxTrueSeen = truePpo2;
    if (truePpo2 < minTrueSeen) minTrueSeen = truePpo2;

    // ---- dive continuity (H1): the board's cadence, minus the file calls ----
    // Exactly the three edges main.cpp's ControlTask uses (dive start -> the
    // verdict; dive end -> the record dive 2 is seeded from; the snapshot grid ->
    // the crash window) plus the 1 Hz seed refresh at the surface. With
    // opts_.continuity off, none of this runs and a dive start resets the tissues
    // the way it always did, which is why every pre-H1 soak witness is
    // byte-identical.
    if (opts_.continuity) continuityStep(now, st.diveActive);
}

// ---- dive continuity (H1): the rig side ------------------------------------
// The decisions below are copies of the shipping ones, not re-implementations:
// ContinuityStore::write's ordering (seq off the slots, then the OLDER slot, then
// pack), comms::continuityRefreshSeed's publish (newest record + the interval
// decided AS OF NOW), and main.cpp's composition of the loss reason from the
// store's state. What is absent is LittleFS and the FreeRTOS locks, so a soak
// witness about dive 2 is a witness about the shipping logic.

bool Rig::contWriteNow(uint8_t kind) {
    logfmt::ContinuityRecord r{};
    if (!logfmt::fromTissueState(loop_.deco().tissueState(), kind, r)) return false;
    r.epoch = epoch();
    if (r.epoch == 0u) r.flags |= logfmt::CONT_FLAG_CLOCK_UNSET;
    r.seq = contSlots_.nextSeq();          // the records are the only durable counter
    const int slot = contSlots_.freeSlot();  // the OLDER one: a torn write costs a
                                             // snapshot, never the picture
    if (!contSlots_.write(slot, r)) return false;
    contBytesPresent_ = true;
    contRefreshSeed();
    return true;
}

void Rig::contRefreshSeed() {
    ctrl::Loop::DiveSeed s{};
    logfmt::ContinuityRecord r{};
    if (contSlots_.readNewest(r) && logfmt::toTissueState(r, s.st)) {
        const logfmt::SurfaceInterval iv =
            logfmt::surfaceIntervalS(r.kind, r.epoch, epoch());
        s.valid = true;
        s.recordSeq = r.seq;
        s.recordEpoch = r.epoch;
        s.kind = r.kind;
        s.creditS = iv.credit ? iv.seconds : 0u;
        s.creditWhy = iv.why;
        contSeedOffered_ = true;
    } else {
        contSeedOffered_ = false;
    }
    loop_.postDiveSeed(s);
}

void Rig::contBootImage(const uint8_t* bytes, size_t len) {
    // The boot read, simulated: this rig becomes a board that has just mounted a
    // partition it did not write. It goes through the same ContinuitySlots::load
    // the store's readFile() uses, so the CRC gate, the "a torn file is no record"
    // padding and the slot choice are the shipping ones — only LittleFS is absent.
    contSlots_.load(bytes, len);
    contBytesPresent_ = false;
    for (size_t i = 0; i < len && i < logfmt::ContinuitySlots::kImageSize; ++i)
        if (bytes && bytes[i]) { contBytesPresent_ = true; break; }
    contRefreshSeed();   // what the boot read found is what the first dive is seeded from
}

void Rig::contCorruptNewest() {
    const int s = contSlots_.newest();
    if (s < 0) return;
    uint8_t* b = contSlots_.rawSlotBytes(s);
    if (!b) return;
    // A bit flipped in the PAYLOAD, not in the magic: the record still looks like
    // a record, and only the CRC knows it is not. That is the difference between
    // CONT_LOSS_ABSENT ("this board has never dived") and CONT_LOSS_BAD ("this
    // board forgot"), and the diver is owed different sentences about them.
    b[logfmt::CONT_RECORD_SIZE / 2] ^= 0x40;
}

void Rig::contCorruptAll() {
    // Every slot damaged: the picture itself is gone, not just the last snapshot.
    // One torn slot is survivable by design (continuity.h); this is the case that
    // is not, and it is the case EVT_CONT_LOST exists to report.
    for (int s = 0; s < logfmt::ContinuitySlots::kSlots; ++s) {
        uint8_t* b = contSlots_.rawSlotBytes(s);
        if (b) b[logfmt::CONT_RECORD_SIZE / 2] ^= 0x40;
    }
}

void Rig::continuityStep(uint32_t now, bool diving) {
    // The dive-start edge, taken off diveStarts() rather than diveActive(): with
    // a deferred start the flag can rise a cycle before the tissues exist, and a
    // verdict composed from a seed that has not been consumed yet is a verdict
    // about a dive that has not started.
    const uint32_t starts = loop_.diveStarts();
    if (starts != contStarts_) {
        contStarts_ = starts;
        contSnapMs_ = now;      // the grid belongs to the dive, not to boot
        contSeed_ = loop_.lastSeed();
        contLoss_ = logfmt::CONT_LOSS_NONE;
        if (!contSeed_.applied) {
            // main.cpp's order, and CONT_LOSS_STORE is unreachable here: the rig
            // has no mount to fail, and saying so is better than inventing a
            // failure the simulator cannot produce.
            if (contCorrupt()) contLoss_ = logfmt::CONT_LOSS_BAD;
            else if (!contSeedOffered_) contLoss_ = logfmt::CONT_LOSS_ABSENT;
            else contLoss_ = logfmt::CONT_LOSS_SEED;
        }
    }

    if (diving) {
        if (now - contSnapMs_ >= CONTINUITY_SNAPSHOT_MS) {
            contSnapMs_ = now;
            contWriteNow(logfmt::CONT_KIND_PERIODIC);
        }
    } else if (contWasDiving_) {
        contWriteNow(logfmt::CONT_KIND_DIVE_END);
    }
    contWasDiving_ = diving;

    if (!diving && now - contRefreshAt_ >= CONTINUITY_SEED_REFRESH_MS) {
        contRefreshAt_ = now;
        contRefreshSeed();
    }
}

}  // namespace sim
