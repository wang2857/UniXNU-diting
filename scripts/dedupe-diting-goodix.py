#!/usr/bin/env python3
"""Normalize bsd/dev/diting_goodix.c after the overlapping touch patch series.

The Diting touch patches were authored against a moving context.  With the
2026-11-22 upstream merge several of them can insert the Berlin-D config and
THP sections more than once.  Keep the last complete copy of each named
section; the earlier copies are the partial duplicates.
"""
import sys
from pathlib import Path

CONFIG = 'Berlin-D configuration upload'
THP = 'Berlin-D THP host touch processing'

def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: dedupe-diting-goodix.py PATH')
    path = Path(sys.argv[1])
    lines = path.read_text().splitlines()
    cfg = [i for i, line in enumerate(lines) if CONFIG in line]
    thp = [i for i, line in enumerate(lines) if THP in line]
    if len(cfg) <= 1 and len(thp) <= 1:
        return 0
    remove = set()
    if len(cfg) > 1:
        remove.update(range(cfg[1], cfg[-1]))
    if len(thp) > 1:
        remove.update(range(thp[1], thp[-1]))
    out = [line for i, line in enumerate(lines) if i not in remove]
    path.write_text('\n'.join(out) + '\n')
    print(f'dedupe goodix: removed {len(remove)} duplicate lines ({len(cfg)} config, {len(thp)} THP markers)')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
