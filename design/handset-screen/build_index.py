#!/usr/bin/env python3
"""Regenerates index.html — the contact sheet for the 41 handset screens.

The sheet is generated from `screens/*.svg` and nothing else, so the inline copy
of a screen can never disagree with the file it came from. (It happened once: an
edit applied to `screens/` and hand-mirrored into the sheet, and the mirror went
stale.) Screen state, cadence and the note come from `DESIGN.md` §6's tables, so
the inventory text has one source too.

    python3 build_index.py

Writes `index.html` in this directory. No network, no fonts, no build step: the
page reads `tokens.css` over a relative link and inlines the SVGs.
"""
import glob
import html
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__)) or '.'

# ---------------------------------------------------------------- inventory
# DESIGN.md §6: | `id` | state | source | cadence | note |
META = {}
with open(os.path.join(HERE, 'DESIGN.md'), encoding='utf-8') as fh:
    for line in fh:
        if not line.startswith('| `'):
            continue
        # Cell separators are the pipes with space on both sides. A wire pair
        # (`6|4.5`) has none, so it survives inside a cell.
        cells = re.split(r' \| ', line.strip().strip('|').strip())
        if len(cells) != 5:
            continue
        key = cells[0].strip('`')
        if key in META:
            raise SystemExit('duplicate inventory row for %s' % key)
        META[key] = {'state': cells[1], 'source': cells[2],
                     'cadence': cells[3], 'note': cells[4]}

def body_of(svg_path):
    """Everything inside the <svg> root, plus the root's viewBox/size."""
    src = open(svg_path, encoding='utf-8').read()
    root = re.search(r'<svg\b[^>]*>', src)
    if not root:
        raise SystemExit('%s has no <svg> root' % svg_path)
    view = re.search(r'viewBox="0 0 320 172"', root.group(0))
    if not view:
        raise SystemExit('%s is not 320x172' % svg_path)
    inner = src[root.end():src.rfind('</svg>')]
    return view.group(0), inner

def desc_of(inner):
    m = re.search(r'<desc[^>]*>(.*?)</desc>', inner, re.S)
    return html.unescape(m.group(1)).strip() if m else ''

def strip_meta(inner):
    """Drop <desc>/<title>: the caption below the frame carries that text."""
    inner = re.sub(r'\s*<(desc|title)[^>]*>.*?</\1>', '', inner, flags=re.S)
    return inner

screens = []
for path in sorted(glob.glob(os.path.join(HERE, 'screens', '*.svg'))):
    sid = os.path.splitext(os.path.basename(path))[0]
    view, inner = body_of(path)
    screens.append({
        'id': sid,
        'view': view,
        'inner': strip_meta(inner),
        'desc': desc_of(inner),
        'meta': META.get(sid),
    })

missing = [s['id'] for s in screens if not s['meta']]
if missing:
    raise SystemExit('no DESIGN.md §6 row for: %s' % ', '.join(missing))

nav_view, nav_inner = body_of(os.path.join(HERE, 'nav.svg')) \
    if 'viewBox="0 0 320 172"' in open(os.path.join(HERE, 'nav.svg')).read() \
    else (None, None)
nav_src = open(os.path.join(HERE, 'nav.svg'), encoding='utf-8').read()

# ---------------------------------------------------------------- page
CARDS = []
for s in screens:
    m = s['meta']
    CARDS.append("""    <figure class="card">
      <div class="frame">{view}
{inner}
      </svg>
      </div>
      <figcaption>
        <p class="cid">{id}</p>
        <p class="state">{state} · <span class="cad">{cadence}</span></p>
        <p class="wire">{wire}</p>
        <p class="note">{note}</p>
      </figcaption>
    </figure>""".format(view=s['view'], inner=s['inner'].rstrip(), id=s['id'],
                        state=html.escape(m['state']), cadence=html.escape(m['cadence']),
                        wire=html.escape(s['desc'] or m['source']),
                        note=html.escape(m['note'])))

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP-CCR wired handset — 41 screens at 320 x 172</title>
<link rel="stylesheet" href="tokens.css">
<style>
  /* Chrome for the sheet only. Nothing here is part of a screen. */
  body {{ margin: 0; padding: 24px 20px 60px; background: #0a0f15; color: var(--ink);
         font-family: var(--font-ui); }}
  h1 {{ font-size: 20px; margin: 0 0 4px; letter-spacing: .3px; }}
  .sub {{ color: var(--dim); font-size: 13px; margin: 0 0 18px; max-width: 78ch; }}
  .controls {{ display: flex; flex-wrap: wrap; gap: 8px; align-items: center;
               margin: 0 0 22px; padding: 10px 12px; background: var(--panel);
               border: 1px solid var(--line); }}
  .controls span {{ font-size: 11px; color: var(--dim); letter-spacing: .4px; }}
  .controls button {{ font-family: var(--font-ui); font-size: 11px; letter-spacing: .4px;
                      padding: 5px 9px; background: var(--bg); color: var(--dim);
                      border: 1px solid var(--line); border-radius: 0; cursor: pointer; }}
  .controls button[aria-pressed="true"] {{ background: var(--live); color: var(--onfill);
                                          border-color: var(--live); }}
  .legend {{ font-size: 11px; color: var(--dim); margin: 0 0 22px; max-width: 92ch; }}
  .legend b {{ color: var(--ink); }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(340px, 1fr));
           gap: 22px; }}
  .card {{ margin: 0; }}
  .frame {{ width: 320px; height: 172px; border: 1px solid var(--line); background: var(--bg); }}
  .frame svg {{ display: block; width: 320px; height: 172px; }}
  .zoom-2 .frame, .zoom-2 .frame svg {{ width: 640px; height: 344px; }}
  .zoom-3 .frame, .zoom-3 .frame svg {{ width: 960px; height: 516px; }}
  figcaption {{ padding: 8px 2px 0; }}
  .cid {{ margin: 0; font-size: 12px; color: var(--live); letter-spacing: .4px; }}
  .state {{ margin: 2px 0 6px; font-size: 11px; color: var(--dim); }}
  .cad {{ color: var(--held); }}
  .wire {{ margin: 0 0 5px; font-size: 11px; color: var(--dim); line-height: 1.45;
           font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }}
  .note {{ margin: 0; font-size: 12px; color: var(--ink); line-height: 1.45; max-width: 62ch; }}
  /* The three lighting states a diver actually reads this panel in. A screen
     that only works in one of them is a screen that fails rule 2 of tokens.css:
     every meaning must also be a word and a shape. */
  .mode-sun .frame svg {{ filter: brightness(1.45) contrast(.82) saturate(.75); }}
  .mode-low .frame svg {{ filter: brightness(.5) contrast(1.06); }}
  .mode-grey .frame svg {{ filter: grayscale(1) contrast(1.12); }}
  .navbox {{ margin: 0 0 26px; padding: 16px; background: var(--panel);
             border: 1px solid var(--line); }}
  .navbox svg, .navbox img {{ display: block; max-width: 100%; height: auto; }}
  .navbox p {{ margin: 10px 0 0; font-size: 12px; color: var(--dim); max-width: 78ch; }}
  footer {{ margin-top: 34px; font-size: 12px; color: var(--dim); max-width: 88ch;
            line-height: 1.55; }}
  footer code {{ color: var(--live); }}
</style>
</head>
<body>
<h1>ESP-CCR wired handset — 41 screens, 320 x 172 px, 2 Hz, three keys</h1>
<p class="sub">Every frame is a real screen file, inlined unchanged: <code>screens/*.svg</code>
  is the only source this page reads. The numbers are real bench wire lines, quoted under
  each frame; the caption's last line is the defect that screen exists to prevent.
  <code>DESIGN.md</code> holds the rules, <code>tokens.css</code> the tokens.</p>

<div class="controls">
  <span>ZOOM</span>
  <button type="button" data-zoom="1" aria-pressed="true">1x</button>
  <button type="button" data-zoom="2" aria-pressed="false">2x</button>
  <button type="button" data-zoom="3" aria-pressed="false">3x</button>
  <span>LIGHT</span>
  <button type="button" data-mode="" aria-pressed="true">panel</button>
  <button type="button" data-mode="mode-sun" aria-pressed="false">sunlight</button>
  <button type="button" data-mode="mode-low" aria-pressed="false">low light</button>
  <button type="button" data-mode="mode-grey" aria-pressed="false">no colour</button>
</div>

<p class="legend"><b>Why the light toggles are there:</b> colour is reserved for meaning
  (<code>live</code>, <code>ok</code>, <code>warn</code>, <code>crit</code>,
  <code>held</code>), so <b>no colour</b> must still leave every screen readable —
  each meaning also has a word and a shape. Sunlight and low light are the two angles
  where a thin outline disappears; that is why a critical alarm is a solid filled band
  with dark text on it and never a tint.</p>

<div class="navbox">{nav}
  <p>The whole navigation model in one drawing: three keys, one cursor, one held gesture.
     ENTER held 1.5 s goes home from anywhere, which is what makes "every screen is
     escapable" true without a back key. The red dashed edge is the ACK override — it
     exists only while a <b>critical</b> FLAGS bit is set, because
     <code>Manager::ack()</code> suppresses nothing else.</p>
</div>

<div class="grid">
{cards}
</div>

<footer>
  Regenerate with <code>python3 build_index.py</code>. Screens are verified against the
  panel's own type metrics: no <code>&lt;text&gt;</code> crosses the 320 px edge, no two
  strings on one baseline overlap, and every number drawn is a token of the wire line
  quoted in that screen's <code>&lt;desc&gt;</code>.
</footer>

<script>
  const body = document.body;
  function press(group, btn) {{
    group.forEach(b => b.setAttribute('aria-pressed', String(b === btn)));
  }}
  const zoom = [...document.querySelectorAll('[data-zoom]')];
  zoom.forEach(b => b.addEventListener('click', () => {{
    body.classList.remove('zoom-2', 'zoom-3');
    if (b.dataset.zoom !== '1') body.classList.add('zoom-' + b.dataset.zoom);
    press(zoom, b);
  }}));
  const mode = [...document.querySelectorAll('[data-mode]')];
  mode.forEach(b => b.addEventListener('click', () => {{
    ['mode-sun', 'mode-low', 'mode-grey'].forEach(c => body.classList.remove(c));
    if (b.dataset.mode) body.classList.add(b.dataset.mode);
    press(mode, b);
  }}));
</script>
</body>
</html>
""".format(nav=nav_src.strip(), cards='\n'.join(CARDS))

out = os.path.join(HERE, 'index.html')
with open(out, 'w', encoding='utf-8') as fh:
    fh.write(PAGE)
print('wrote %s: %d screens, %d bytes' % (out, len(screens), len(PAGE)))
