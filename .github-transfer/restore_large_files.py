#!/usr/bin/env python3
"""Restore uploaded large files at their original project paths, with SHA-256 checks."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(8 * 1024**2):
            digest.update(block)
    return digest.hexdigest()


def contained(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'Path escapes project: {relative}')
    return path


def restore(root, records, verify_only=False):
    restored = 0
    for record in records:
        target = contained(root, record['path'])
        if target.exists():
            if target.stat().st_size != record['size'] or sha256(target) != record['sha256']:
                raise ValueError(f'Existing file differs; refusing overwrite: {record["path"]}')
            print(f'OK {record["path"]}')
            continue
        if verify_only:
            raise FileNotFoundError(f'Not restored: {record["path"]}')
        parts = []
        for part in record['parts']:
            source = contained(root, '.github-transfer/objects/' + part['file'])
            if not source.is_file() or source.stat().st_size != part['size'] or sha256(source) != part['sha256']:
                raise ValueError(f'Missing/corrupt part: {source}. Run git lfs pull first.')
            parts.append(source)
        if record['encoding'] not in ('raw', 'gzip'):
            raise ValueError('Unsupported encoding')
        target.parent.mkdir(parents=True, exist_ok=True)
        payload_name = output_name = None
        try:
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.restore-', delete=False) as payload:
                payload_name = Path(payload.name)
                for part in parts:
                    with part.open('rb') as stream:
                        shutil.copyfileobj(stream, payload, 8 * 1024**2)
            output_name = payload_name
            if record['encoding'] == 'gzip':
                with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.restore-', delete=False) as output:
                    output_name = Path(output.name)
                    with gzip.open(payload_name, 'rb') as stream:
                        shutil.copyfileobj(stream, output, 8 * 1024**2)
            if output_name.stat().st_size != record['size'] or sha256(output_name) != record['sha256']:
                raise ValueError(f'Restored checksum mismatch: {record["path"]}')
            os.chmod(output_name, record.get('mode', 0o644))
            # Avoid clobbering a file created concurrently after the initial check.
            os.link(output_name, target)
            restored += 1
            print(f'Restored {record["path"]}')
        finally:
            for temp in {payload_name, output_name} - {None}:
                temp.unlink(missing_ok=True)
    return restored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    manifest = root / '.github-transfer/large-files.json'
    if not manifest.exists():
        raise SystemExit('No uploaded large-file manifest yet; check .github-transfer/pending-files.json.')
    records = json.loads(manifest.read_text())
    count = restore(root, records, args.verify_only)
    print(f'Complete: {len(records)} paths verified, {count} restored.')


if __name__ == '__main__':
    main()
