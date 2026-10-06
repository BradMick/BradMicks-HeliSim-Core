"""Reads the flight log (CBA setting Enable Flight Log) back out of an Arma RPT.

Run it:  python python/dev/flightlog.py [rpt] [--csv out.csv]

With no RPT given, takes the newest in %LOCALAPPDATA%\\Arma 3. Each BMKHSLOG_HDR starts a new
table; the rows that follow it are parsed against its columns. Prints a summary of each table
and, with --csv, writes the last one out.
"""
import argparse
import csv
import glob
import os


def newest_rpt():
    rpts = glob.glob(os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Arma 3', '*.rpt'))
    if not rpts:
        raise SystemExit('No RPT found in %LOCALAPPDATA%\\Arma 3')
    return max(rpts, key=os.path.getmtime)


def tables(path):
    """[(columns, [row dicts])], one per header."""
    out = []
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            i = line.find('BMKHSLOG')
            if i < 0:
                continue
            parts = line[i:].rstrip('\r\n').strip('"').split(',')
            if parts[0] == 'BMKHSLOG_HDR':
                out.append((parts[1:], []))
            elif parts[0] == 'BMKHSLOG' and out:
                cols = out[-1][0]
                if len(parts) - 1 == len(cols):
                    out[-1][1].append(dict(zip(cols, parts[1:])))
    return out


def num(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return None


def summary(cols, rows):
    if not rows:
        print('  (no rows)')
        return
    t0, t1 = num(rows[0], 't'), num(rows[-1], 't')
    print('  %d rows, t %.1f -> %.1f s, %d columns' % (len(rows), t0, t1, len(cols)))
    for key in ('kts', 'altFt', 'vsFpm', 'pitch', 'bank', 'pRateDps', 'rRateDps', 'yRateDps',
                'tq1', 'nr', 'coll', 'attP', 'attR', 'altC', 'hdgY', 'fdFpm', 'fdWantPitch', 'fdWantBank'):
        vals = [v for v in (num(r, key) for r in rows) if v is not None]
        if vals:
            print('    %-12s min %9.3f  max %9.3f  last %9.3f' % (key, min(vals), max(vals), vals[-1]))
    modes = sorted({r.get('fdModes', '') for r in rows} - {''})
    if modes:
        print('    FD modes seen: %s' % ', '.join(modes))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('rpt', nargs='?')
    ap.add_argument('--csv')
    a = ap.parse_args()
    path = a.rpt or newest_rpt()
    print(path)
    ts = tables(path)
    if not ts:
        raise SystemExit('No BMKHSLOG in it - was Enable Flight Log on?')
    for n, (cols, rows) in enumerate(ts):
        print('table %d' % n)
        summary(cols, rows)
    if a.csv:
        cols, rows = ts[-1]
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        print('wrote', a.csv)
