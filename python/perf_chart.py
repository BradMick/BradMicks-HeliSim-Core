"""Max torque available, FAT against %Q, one line per pressure altitude - the rig's answer laid out
like the AH-64D's Maximum Torque Available charts.

Run it:  python python/engine.py perf results.json
         python python/perf_chart.py results.json chart.html
"""
import json
import sys

#Categorical slots 1-6, fixed order (dataviz reference palette), light / dark.
SERIES = [('#2a78d6', '#3987e5'), ('#eb6834', '#d95926'), ('#1baf7a', '#199e70'),
          ('#eda100', '#c98500'), ('#e87ba4', '#d55181'), ('#008300', '#008300')]
#Torque lines on the book's charts: continuous 2-engine, continuous 1-engine, 2.5-min 1-engine.
TQ_LINES = [(100, 'Continuous 2-Eng'), (110, 'Continuous 1-Eng'), (122, '2.5-Min 1-Eng')]

W, H = 620, 560
L, R, T, B = 56, 16, 20, 48
#The TM charts' scales, so the two lay side by side.
X0, X1 = 50, 140
Y0, Y1 = -60, 60


def sx(v): return L + (v - X0) / (X1 - X0) * (W - L - R)
def sy(v): return T + (Y1 - v) / (Y1 - Y0) * (H - T - B)


def interp_tq(pts, fat):
    """Rig torque on a PA line at a given FAT, or None if the line does not span it."""
    for a, b in zip(pts, pts[1:]):
        if a['fat'] <= fat <= b['fat']:
            k = (fat - a['fat']) / (b['fat'] - a['fat'])
            return (a['tq'] + (b['tq'] - a['tq']) * k) * 100
    return None


def panel(rows, chartSwitch, title):
    pas = sorted({r['pa'] for r in rows})
    out = ['<figure><figcaption>%s</figcaption>' % title,
           '<svg viewBox="0 0 %d %d" role="img" aria-label="%s">' % (W, H, title)]
    for v in range(X0, X1 + 1, 10):
        out.append('<line class="grid" x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f"/>' % (sx(v), sx(v), T, H - B))
        out.append('<text class="tick" x="%.1f" y="%.1f" text-anchor="middle">%d</text>' % (sx(v), H - B + 16, v))
    for v in range(Y0, Y1 + 1, 10):
        out.append('<line class="grid" x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f"/>' % (L, W - R, sy(v), sy(v)))
        out.append('<text class="tick" x="%.1f" y="%.1f" text-anchor="end">%d</text>' % (L - 6, sy(v) + 4, v))
    out.append('<text class="axis" x="%.1f" y="%d" text-anchor="middle">Power Available - %%Q</text>' % ((L + W - R) / 2, H - 10))
    out.append('<text class="axis" transform="translate(14,%.1f) rotate(-90)" text-anchor="middle">Free Air Temperature - Deg C</text>' % ((T + H - B) / 2))
    for q, name in TQ_LINES:
        out.append('<line class="ref" x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f"/>' % (sx(q), sx(q), T, H - B))
        out.append('<text class="reflab" transform="translate(%.1f,%.1f) rotate(-90)">%s</text>' % (sx(q) - 4, sy(Y1) + 110, name))

    #Anything under 50 %Q runs off the left edge, as on the TM charts.
    clip = 'clip%d' % abs(hash(title))
    out.append('<clipPath id="%s"><rect x="%.1f" y="%.1f" width="%.1f" height="%.1f"/></clipPath><g clip-path="url(#%s)">'
               % (clip, L, T, W - L - R, H - T - B, clip))
    for k, pa in enumerate(pas):
        pts = sorted((r for r in rows if r['pa'] == pa), key=lambda r: r['fat'])
        cls = 's%d' % k
        path = ' '.join('%s%.1f,%.1f' % ('M' if i == 0 else 'L', sx(p['tq'] * 100), sy(p['fat']))
                        for i, p in enumerate(pts))
        out.append('<path class="line %s" d="%s"/>' % (cls, path))
        for p in pts:
            ng = p['held'] == 'Ng'
            out.append('<circle class="pt %s%s" cx="%.1f" cy="%.1f" r="4"><title>PA %s ft, FAT %d C: %.1f %%Q, '
                       'TGT %.0f C, Ng %.3f, %s-limited</title></circle>'
                       % (cls, ' ng' if ng else '', sx(p['tq'] * 100), sy(p['fat']), format(pa, ','), p['fat'],
                          p['tq'] * 100, p['tgt'], p['ng'], p['held']))
        top = pts[-1]
        out.append('<text class="lab %s" x="%.1f" y="%.1f">%d</text>' % (cls, sx(top['tq'] * 100) + 6, sy(top['fat']) + 4, pa // 1000))
        #Where the book says this PA switches from Ng- to TGT-limited.
        sw = chartSwitch.get(str(pa))
        tq = interp_tq(pts, sw) if sw is not None else None
        if tq is not None:
            out.append('<path class="book" d="M%.1f,%.1f l6,6 l-6,6 l-6,-6 z" transform="translate(0,-6)">'
                       '<title>Chart: PA %s ft switches Ng to TGT at %d C</title></path>'
                       % (sx(tq), sy(sw), format(pa, ','), sw))
    out.append('</g>')
    out.append('<text class="tick" x="%.1f" y="%.1f">PA - 1000 ft (label at line top)</text>' % (L + 6, T + 14))
    out.append('</svg></figure>')
    return '\n'.join(out)


def table(rows, name):
    body = ''.join('<tr><td>%s</td><td>%d</td><td>%.1f</td><td>%.0f</td><td>%.3f</td><td>%s</td></tr>'
                   % (format(r['pa'], ','), r['fat'], r['tq'] * 100, r['tgt'], r['ng'], r['held'])
                   for r in sorted(rows, key=lambda r: (r['pa'], r['fat'])))
    return ('<details><summary>%s - data table</summary><table><thead><tr><th>PA ft</th><th>FAT C</th>'
            '<th>%%Q</th><th>TGT C</th><th>Ng</th><th>Held by</th></tr></thead><tbody>%s</tbody></table></details>'
            % (name, body))


def main(src, dst):
    d = json.load(open(src))
    css_series = '\n'.join('.%s{--c:%s}' % ('s%d' % i, c[0]) for i, c in enumerate(SERIES))
    css_dark = '\n'.join('.%s{--c:%s}' % ('s%d' % i, c[1]) for i, c in enumerate(SERIES))
    html = """<!doctype html><html><head><meta charset="utf-8"><title>Rig Max Torque</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;--grid:#e6e5e0;--ref:#b4b2aa}
%s
@media (prefers-color-scheme:dark){:root{--bg:#1a1a19;--ink:#fff;--ink2:#c3c2b7;--muted:#898781;--grid:#2e2e2c;--ref:#5a5955}
%s}
body{background:var(--bg);color:var(--ink);font:14px system-ui,sans-serif;margin:16px}
h1{font-size:18px;margin:0 0 4px}p{color:var(--ink2);margin:0 0 12px;max-width:1240px}
.wrap{display:flex;flex-wrap:wrap;gap:16px}figure{margin:0;flex:1 1 560px;max-width:640px}
figcaption{font-weight:600;margin-bottom:4px}svg{width:100%%;height:auto}
.grid{stroke:var(--grid);stroke-width:1}.ref{stroke:var(--ref);stroke-width:1.5}
.tick{fill:var(--muted);font-size:11px}.axis{fill:var(--ink2);font-size:12px}.reflab{fill:var(--muted);font-size:10px}
.line{fill:none;stroke:var(--c);stroke-width:2}
.pt{fill:var(--c);stroke:var(--bg);stroke-width:2}.pt.ng{fill:var(--bg);stroke:var(--c)}
.pt:hover{r:6}.lab{fill:var(--ink);font-size:12px;font-weight:600}
.book{fill:var(--ink);stroke:var(--bg);stroke-width:1.5}
.key{display:flex;gap:18px;color:var(--ink2);font-size:13px;margin:0 0 10px;flex-wrap:wrap}
.key svg{width:auto;height:14px;vertical-align:-2px}
table{border-collapse:collapse;font-size:12px;margin:6px 0}td,th{padding:2px 8px;text-align:right;border-bottom:1px solid var(--grid)}
details{margin-top:8px}
</style></head><body>
<h1>Rig: Maximum Torque Available at 101%% NR, hover</h1>
<p>From <code>python engine.py perf</code>: at each PA and FAT, the highest collective that still holds Nr on speed,
and which limit binds there. Laid out like the AH-64D charts. Lines stop where nothing holds Nr on speed.</p>
<div class="key">
<span><svg viewBox="0 0 14 14"><circle cx="7" cy="7" r="4" fill="var(--ink2)"/></svg> TGT-limited</span>
<span><svg viewBox="0 0 14 14"><circle cx="7" cy="7" r="4" fill="var(--bg)" stroke="var(--ink2)" stroke-width="2"/></svg> Ng-limited</span>
<span><svg viewBox="0 0 14 14"><path d="M7,1 l6,6 l-6,6 l-6,-6 z" fill="var(--ink)"/></svg> Book: temperature where that PA switches Ng to TGT</span>
<span>Line colour = PA, labelled in 1000 ft at the top of each line</span>
</div>
<div class="wrap">%s%s</div>
%s%s
</body></html>""" % (css_series, css_dark,
                     panel(d['DE'], d['chartSwitch']['DE'], 'Dual engine - 10-min limit (TGT 867)'),
                     panel(d['SE'], d['chartSwitch']['SE'], 'Single engine - 2.5-min limit (TGT 896)'),
                     table(d['DE'], 'Dual engine'), table(d['SE'], 'Single engine'))
    open(dst, 'w', encoding='utf-8').write(html)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
