"""End-to-end artifacts, isolation orchestration, and fail-fast CLI checks."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

runner, fixture, compare_path = map(Path, sys.argv[1:])
spec = importlib.util.spec_from_file_location('compare', compare_path)
compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compare)


def execute(args, success=True):
    result = subprocess.run([str(x) for x in args], text=True, capture_output=True)
    assert (result.returncode == 0) == success, result.stdout + result.stderr
    return result


def sha_field(hasher, value):
    hasher.update(str(len(value)).encode() + b':' + value)


with tempfile.TemporaryDirectory(prefix='avr-artifact-test-') as temp:
    root = Path(temp)
    output = root / 'comparison'
    execute([sys.executable, compare_path, '--runner', runner, '--sequence', fixture,
             '--output', output, '--warmup-frames', 3])
    result = compare.verify(output)
    assert result['valid']
    dataset_hash = hashlib.sha256()
    sha_field(dataset_hash, (fixture / 'sequence.avr').read_bytes())
    sha_field(dataset_hash, (fixture / 'frames.csv').read_bytes())
    with (fixture / 'frames.csv').open() as f:
        for row in csv.DictReader(f):
            sha_field(dataset_hash, (fixture / row['points_file']).read_bytes())
    assert result['sequence_hash'] == dataset_hash.hexdigest()
    full_rows = None
    for mode in compare.MODES:
        run = output / mode
        manifest = json.loads((run / 'run_manifest.json').read_text())
        summary = json.loads((run / 'summary.json').read_text())
        with (run / 'frames.csv').open() as f:
            rows = list(csv.DictReader(f))
        assert manifest['frame_count'] == 20 and manifest['point_count'] == 280
        assert manifest['warmup_frames'] == 3 and manifest['measured_frame_count'] == 17
        assert len(manifest['git_commit']) == 40 and isinstance(manifest['git_dirty'], bool)
        assert manifest['utc_run_timestamp'].endswith('Z')
        assert all(manifest[k] for k in ('compiler', 'build_type', 'operating_system', 'cpu_architecture'))
        assert manifest['semantic_provenance'] == 'SYNTHETIC_VALIDATION_LABELS'
        assert 'NOT REAL PERFORMANCE DATA' in summary['data_truth']
        assert 'not supplied' in manifest['uncertainty_source']
        assert summary['latency_us_all_frames']['count'] == 20
        assert summary['latency_us_post_warmup']['count'] == 17
        values = [float(row['frame_latency_us']) for row in rows]
        assert all(value >= 0 for value in values)
        assert abs(summary['latency_us_all_frames']['mean'] - sum(values) / len(values)) < 1e-8
        assert abs(summary['latency_us_post_warmup']['mean'] - sum(values[3:]) / 17) < 1e-8
        memory = summary['rss_bytes']
        assert memory['baseline'] > 1024 and memory['final'] > 1024
        assert memory['peak'] >= max(memory['baseline'], memory['final'])
        assert memory['peak_delta'] == memory['peak'] - memory['baseline']
        assert memory['final_delta'] == memory['final'] - memory['baseline']
        assert all(v > 0 for v in summary['structure_size_bytes'].values())
        assert summary['modelled_logical_bytes'] is None
        assert summary['dynamic_contamination_count'] is None
        assert summary['invariant_failures'] == 0
        for row in rows:
            assert int(row['static_fusable_points']) == 8 and int(row['dynamic_excluded_points']) == 6
            assert int(row['dynamic_cluster_count']) == 1
            assert sum(int(row[f'l{i}_count']) for i in range(4)) == int(row['active_leaf_count'])
            assert row['data_truth'] == manifest['data_truth']
        if mode == 'full-avr':
            full_rows = rows
    # Repeat in a fresh process; compare deterministic fields only (timing/RSS/UTC vary).
    repeat = root / 'repeat'
    execute([runner, '--mode', 'full-avr', '--sequence', fixture, '--output', repeat])
    with (repeat / 'frames.csv').open() as f:
        again = list(csv.DictReader(f))
    stable = [k for k in full_rows[0] if k not in ('rss_bytes', 'frame_latency_us')]
    assert [[r[k] for k in stable] for r in again] == [[r[k] for k in stable] for r in full_rows]
    # Verify actual mismatch returns nonzero and marks comparison INVALID.
    changed = output / 'distance-only' / 'frames.csv'
    original = changed.read_text()
    rows = list(csv.DictReader(original.splitlines()))
    rows[0]['routing_hash'] = '0' * 64
    with changed.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    execute([sys.executable, compare_path, '--output', output, '--verify-existing'], False)
    assert not json.loads((output / 'comparison.json').read_text())['valid']
    changed.write_text(original)
    execute([sys.executable, compare_path, '--output', output, '--verify-existing'])
    missing = root / 'missing'
    shutil.copytree(fixture, missing)
    points = missing / 'road.csv'
    points.write_text(points.read_text().replace('semantic_class', 'missing_labels'))
    for mode in compare.MODES:
        execute([runner, '--mode', mode, '--sequence', missing, '--output', root / ('bad-' + mode)], False)
    execute([runner, '--mode', 'full-avr', '--sequence', fixture, '--output', repeat], False)
    for warmup in ('-1', '20', '999', 'invalid'):
        execute([runner, '--mode', 'full-avr', '--sequence', fixture, '--output', root / ('warm-' + warmup), '--warmup-frames', warmup], False)
print('Artifact schemas, fresh-process comparison, independent SHA-256, determinism and failure gates passed.')
