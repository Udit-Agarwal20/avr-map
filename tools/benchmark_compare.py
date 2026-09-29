#!/usr/bin/env python3
"""Run exactly one policy per fresh process, then verify actual input/routing identity."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

MODES = ('uniform-l0-observed', 'distance-only', 'full-avr')
NAMES = ('UNIFORM_L0_OBSERVED', 'DISTANCE_ONLY', 'FULL_AVR')
COMMON = ('sequence_hash', 'sequence_name', 'data_truth', 'semantic_provenance',
          'source_description', 'license', 'pose_provenance', 'coordinate_frame',
          'grid_origin_m', 'bounds_xy_m', 'frame_count', 'point_count', 'warmup_frames',
          'measured_frame_count', 'config', 'clustering_config', 'git_commit',
          'git_dirty', 'compiler', 'build_type', 'sanitizers', 'operating_system', 'cpu_architecture')
IDENTITY = ('frame_seq', 'timestamp_ns', 'input_hash', 'raw_points', 'accepted_points',
            'static_fusable_points', 'dynamic_excluded_points', 'fused_observations', 'routing_hash')


def verify(output):
    runs = []
    for mode, name in zip(MODES, NAMES):
        root = output / mode
        manifest = json.loads((root / 'run_manifest.json').read_text())
        summary = json.loads((root / 'summary.json').read_text())
        with (root / 'frames.csv').open(newline='') as file:
            rows = list(csv.DictReader(file))
        if manifest['schema_version'] != 1 or summary['schema_version'] != 1:
            raise ValueError('unsupported artifact schema')
        if not manifest['valid'] or not summary['valid'] or manifest['mode'] != name or summary['mode'] != name:
            raise ValueError(f'{mode}: invalid run or incorrect mode')
        if not rows or len(rows) != manifest['frame_count'] or summary['attempted_frames'] != len(rows):
            raise ValueError(f'{mode}: incomplete frame coverage')
        if summary['sequence_hash'] != manifest['sequence_hash']:
            raise ValueError(f'{mode}: sequence hash discrepancy')
        if summary['data_truth'] != manifest['data_truth']:
            raise ValueError('summary data-truth discrepancy')
        cumulative = hashlib.sha256()
        previous = None
        for row in rows:
            seq, timestamp = int(row['frame_seq']), int(row['timestamp_ns'])
            if previous and (seq <= previous[0] or timestamp <= previous[1]):
                raise ValueError('non-monotonic frame identity')
            previous = (seq, timestamp)
            fingerprint = row['routing_hash'].encode('ascii')
            if len(fingerprint) != 64 or any(c not in '0123456789abcdef' for c in row['routing_hash']):
                raise ValueError('malformed routing fingerprint')
            cumulative.update(str(len(fingerprint)).encode() + b':' + fingerprint)
            if any(row[k] != 'true' for k in ('valid', 'routing_valid', 'invariants_valid')):
                raise ValueError(f'{mode}: failed frame')
            if row['data_truth'] != manifest['data_truth']:
                raise ValueError('data-truth label discrepancy')
            if row['status_code'] not in ('0', '1') or row['status_name'] != ('SUCCESS' if row['status_code'] == '0' else 'SUCCESS_REPLAY_PENDING'):
                raise ValueError(f'{mode}: unsuccessful or unverifiable cached routing')
            raw, static, dynamic = (int(row[k]) for k in ('raw_points', 'static_fusable_points', 'dynamic_excluded_points'))
            if raw != static + dynamic or int(row['accepted_points']) != raw or int(row['fused_observations']) != static:
                raise ValueError(f'{mode}: routing/fusion conservation failure')
        if cumulative.hexdigest() != summary['routing_hash']:
            raise ValueError('per-frame/cumulative routing hash discrepancy')
        if sum(int(r['raw_points']) for r in rows) != manifest['point_count']:
            raise ValueError('raw point count discrepancy')
        for key in ('static_fusable_points', 'dynamic_excluded_points'):
            if sum(int(r[key]) for r in rows) != summary[key]:
                raise ValueError('summary routing discrepancy')
        runs.append((manifest, summary, rows))
    first_manifest, first_summary, first_rows = runs[0]
    for manifest, summary, rows in runs[1:]:
        if any(manifest[key] != first_manifest[key] for key in COMMON):
            raise ValueError('comparison INVALID: input, provenance, build or configuration mismatch')
        if summary['routing_hash'] != first_summary['routing_hash']:
            raise ValueError('comparison INVALID: cumulative routing mismatch')
        if [[r[k] for k in IDENTITY] for r in rows] != [[r[k] for k in IDENTITY] for r in first_rows]:
            raise ValueError('comparison INVALID: per-frame input/source/routing mismatch')
    return {'schema_version': 1, 'valid': True, 'data_truth': first_manifest['data_truth'],
            'sequence_hash': first_manifest['sequence_hash'], 'routing_hash': first_summary['routing_hash'],
            'frame_count': len(first_rows), 'modes': list(NAMES),
            'process_isolation': 'Each mode launched as a separate subprocess; verify-existing relies on original run provenance'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner', type=Path)
    parser.add_argument('--sequence', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--warmup-frames', type=int, default=0)
    parser.add_argument('--verify-existing', action='store_true')
    args = parser.parse_args(argv)
    # Never overwrite an existing experiment by default.
    if not args.verify_existing and args.output.exists():
        parser.error('output exists; choose a fresh experiment directory')
    if not args.verify_existing and (not args.runner or not args.sequence):
        parser.error('--runner and --sequence are required')
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        if not args.verify_existing:
            for mode in MODES:
                completed = subprocess.run([str(args.runner.resolve()), '--mode', mode,
                    '--sequence', str(args.sequence.resolve()), '--output', str(args.output / mode),
                    '--warmup-frames', str(args.warmup_frames)], capture_output=True, text=True, check=False)
                # Preserve all subprocess diagnostics, including failed runs.
                (args.output / f'{mode}.log').write_text(completed.stdout + completed.stderr)
                if completed.returncode:
                    raise ValueError(f'{mode} exited {completed.returncode}; see preserved log')
        result = verify(args.output)
    except (ValueError, KeyError, OSError, TypeError) as exc:
        result = {'schema_version': 1, 'valid': False, 'error': str(exc),
                  'data_truth': 'INVALID COMPARISON / NOT PERFORMANCE EVIDENCE'}
    (args.output / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    print(result['data_truth'])
    print('VALID' if result['valid'] else f'INVALID: {result["error"]}')
    return 0 if result['valid'] else 2


if __name__ == '__main__':
    sys.exit(main())
