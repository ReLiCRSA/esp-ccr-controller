# ESP-CCR web UI

Mobile-first, dark, framework-free pages served from LittleFS on the device.
They speak the same line grammar as the handset (over `/ws` WebSocket).

## Preview on the desktop (no hardware needed)

Open any `.html` file directly in a browser — with no WebSocket present the
shared `app.js` falls back to a built-in dive simulator, so layout, state
colors, histogram and settings widgets can be reviewed end-to-end.

For a phone on the same machine (dev only):

```bash
cd web && python3 -m http.server 8080   # then browse http://<host>:8080 from the phone
```

## Files

| File | Purpose |
|---|---|
| `app.css` | design system + responsive grid (1 col phone → dashboard wide) |
| `app.js` | WebSocket/mock source, T/D/E parser shared by all pages, header injection, toasts, staleness banner |
| `index.html` | Status: big ppO₂, depth, cells, deco ladder, system state |
| `histogram.html` | per-cell ppO₂ histogram (pre-dive cell health picture) |
| `checklist.html` | pre-dive stepper + calibration actions (two-step CAL O₂) |
| `settings.html` | grouped registry editor; ranges mirrored, device is authority |
| `dives.html` | 10-dive list, profile chart, CSV export |

When wired to firmware, ESP32 pushes `T,` at 2 Hz, `D,` at 1 Hz, `E,` on
change, and answers GET/SET exactly like the handset protocol.
