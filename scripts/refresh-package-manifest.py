#!/usr/bin/env python3
"""Refresh package.manifest.json after the device-specific ADT/DarwinBoot pass.

The official package step verifies the artifact set it produced. Diting then
patches the ADT and repacks the boot image from the final payload. This script
updates the same manifest keys to the files on disk, so a later consumer cannot
mistake the pre-patch boot.img for the delivered one. It never invents a path:
every key must resolve under the work directory.
"""
import hashlib
import json
import sys
from pathlib import Path

OUTPUTS = {
    'boot.img': 'boot/boot.img',
    'diting-xnu-initial.img': 'boot/diting-xnu-initial.img',
    'diting-unixnu-uefi.img': 'boot/diting-unixnu-uefi.img',
    'kernel.macho': 'kernel/kernel.macho',
    'payload.img': 'payload/payload.img',
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: refresh-package-manifest.py <output-config-dir>')
    work = Path(sys.argv[1]).resolve()
    path = work / 'package.manifest.json'
    manifest = json.loads(path.read_text())
    outputs = manifest.setdefault('outputs', {})
    for key, relative in OUTPUTS.items():
        target = (work / relative).resolve()
        if work not in target.parents:
            raise SystemExit(f'{key}: path escapes the output directory: {target}')
        if not target.is_file():
            raise SystemExit(f'{key}: missing {target}')
        outputs[key] = {'bytes': target.stat().st_size, 'sha256': sha256(target)}
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    for key in sorted(outputs):
        print(f'{key}: {outputs[key]["bytes"]} bytes {outputs[key]["sha256"]}')


if __name__ == '__main__':
    main()
