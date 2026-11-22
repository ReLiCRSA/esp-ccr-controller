// ============================================================================
// hud.cpp — 3-LED pattern engine (PLAN.md 12).
//
// HB:     1 Hz = AUTO alive | slow = standby | double-beat = degraded/SAFE.
// ERR:    off=clean | slow=warning | fast=alarm | solid=hard fault / SAFE.
// DECO:   solid while a deco stop is demanded.
//
// The pattern clock runs in HudTask. If HudTask itself dies the HB stops
// visibly AND the supervisor restarts the MCU — a dead heartbeat is never
// mistaken for a live system.
// ============================================================================
#pragma once
#include <Arduino.h>

enum class HudState : uint8_t { STANDBY, AUTO, DEGRADED, SAFE };
enum class ErrLevel : uint8_t { NONE, WARNING, ALARM, HARD };

void hudInit();
void hudUpdate(uint32_t nowMs);            // call from HudTask every ~10 ms
void hudSetState(HudState s);              // HB pattern source
void hudSetError(ErrLevel lvl);            // ERR pattern source
void hudSetDeco(bool on);                  // DECO solid on
