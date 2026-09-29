"""Synthetic contract checks only; no real recorded-performance evidence."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recorded_sequence_adapter as adapter

RUNNER = None
if '--runner' in sys.argv:
    position = sys.argv.index('--runner')
    RUNNER = Path(sys.argv[position + 1])
    del sys.argv[position:position + 2]

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
# 90-degree Z rotation followed by translation (2,3,1).
POSE = [0, -1, 0, 2, 1, 0, 0, 3, 0, 0, 1, 1, 0, 0, 0, 1]


class RecordedAdapter(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='avr-recording-contract-')
        self.root = Path(self.temporary.name)
        self.source = self.root / 'source'
        self.source.mkdir()
        self.output = self.root / 'converted'
        self.meta = {
            'schema_version': 1, 'name': 'synthetic-adapter-contract',
            'kind': 'HARNESS_VALIDATION_FIXTURE', 'recording_id': 'synthetic-contract-only',
            'source_uri': 'validation-fixture:no-recorded-data', 'license': 'Repository test fixture',
            'semantic_provenance': 'SYNTHETIC_VALIDATION_LABELS',
            'semantic_label_source': 'Hand-authored contract test labels',
            'motion_label_source': 'Hand-authored contract test motion',
            'pose_provenance': 'Synthetic known rigid transform',
            'coordinate_convention': 'RIGHT_HANDED_Z_UP_METRES',
            'bounds': [-10, 10, -10, 10], 'semantic_class_map': {'road': 2, 'moving_actor': 5},
            'frames': 'frames.jsonl',
        }
        self.frames = [dict(frame_seq=10, timestamp_ns=100, points_file='raw.csv', transform_map_from_sensor=POSE),
                       dict(frame_seq=20, timestamp_ns=200, points_file='raw.csv', transform_map_from_sensor=IDENTITY)]
        self.rows = [[0, 1, 0, .1, 127.5, 'road', .9, 1, .9],
                     [1, 1.05, 0, 3, 64.25, 'moving_actor', .2, 3, .95]]
        self.save()

    def tearDown(self):
        self.temporary.cleanup()

    def save(self):
        (self.source / 'recording.json').write_text(json.dumps(self.meta), encoding='utf-8')
        (self.source / 'frames.jsonl').write_text(''.join(json.dumps(f) + '\n' for f in self.frames), encoding='utf-8')
        with (self.source / 'raw.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(adapter.INPUT_HEADER)
            writer.writerows(self.rows)

    def failure(self):
        self.save()
        with self.assertRaises((ValueError, OSError)):
            adapter.convert_sequence(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.root.glob('.avr-convert-*')))

    def test_transform_preserves_order_time_intensity_labels_and_origin(self):
        result = adapter.convert_sequence(self.source, self.output)
        self.assertEqual(result['frame_count'], 2)
        self.assertEqual(result['point_count'], 4)
        self.assertIn('NOT REAL PERFORMANCE DATA', result['data_truth'])
        with (self.output / 'points-00000000.csv').open() as file:
            points = list(csv.DictReader(file))
        self.assertEqual([p['source_index'] for p in points], ['0', '1'])
        self.assertEqual(tuple(float(points[0][k]) for k in ('x', 'y', 'z')), (2, 4, 1.1))
        self.assertEqual(float(points[1]['intensity']), 64.25)
        self.assertEqual(points[1]['semantic_class'], '5')
        self.assertEqual(points[1]['motion_state'], '3')
        self.assertAlmostEqual(float(points[1]['semantic_confidence']), .2)
        with (self.output / 'frames.csv').open() as file:
            frames = list(csv.DictReader(file))
        self.assertEqual([r['frame_seq'] for r in frames], ['10', '20'])
        self.assertEqual([r['timestamp_ns'] for r in frames], ['100', '200'])
        self.assertEqual(tuple(float(frames[0][k]) for k in ('sensor_x', 'sensor_y', 'sensor_z')), (2, 3, 1))

    def test_byte_reproducible_conversion_and_original_files_unchanged(self):
        originals = {p.name: p.read_bytes() for p in self.source.iterdir()}
        first = adapter.convert_sequence(self.source, self.output)
        other = self.root / 'repeat'
        second = adapter.convert_sequence(self.source, other)
        self.assertEqual(first, second)
        self.assertEqual({p.name: p.read_bytes() for p in self.output.iterdir()},
                         {p.name: p.read_bytes() for p in other.iterdir()})
        self.assertEqual(originals, {p.name: p.read_bytes() for p in self.source.iterdir()})
        digest = hashlib.sha256()
        for content in [originals['recording.json'], originals['frames.jsonl'], originals['raw.csv'], originals['raw.csv']]:
            digest.update(str(len(content)).encode() + b':' + content)
        self.assertEqual(first['source_content_hash'], digest.hexdigest())

    def test_rejects_missing_label_mapping_without_default_unknown(self):
        self.rows[0][5] = 'unmapped'; self.failure()

    def test_rejects_missing_evidence(self):
        self.rows[0][6] = ''; self.failure()

    def test_rejects_reordered_source_ordinals(self):
        self.rows.reverse(); self.failure()

    def test_rejects_duplicate_source_ordinals(self):
        self.rows[1][0] = 0; self.failure()

    def test_rejects_nonmonotonic_sequence(self):
        self.frames[1]['frame_seq'] = 10; self.failure()

    def test_rejects_nonmonotonic_timestamps(self):
        self.frames[1]['timestamp_ns'] = 100; self.failure()

    def test_rejects_scaled_pose(self):
        self.frames[0]['transform_map_from_sensor'] = [v * 2 for v in IDENTITY]; self.failure()

    def test_rejects_reflection(self):
        pose = IDENTITY.copy(); pose[0] = -1
        self.frames[0]['transform_map_from_sensor'] = pose; self.failure()

    def test_rejects_missing_pose(self):
        del self.frames[0]['transform_map_from_sensor']; self.failure()

    def test_rejects_point_outside_bounds_without_clipping(self):
        self.rows[0][1] = 100; self.failure()

    def test_rejects_nonfinite_point(self):
        self.rows[0][1] = 'nan'; self.failure()

    def test_rejects_invalid_confidence_and_motion(self):
        self.rows[0][7] = 4; self.failure()
        self.rows[0][7] = 1; self.rows[0][8] = 1.1; self.failure()

    def test_rejects_missing_provenance_and_synthetic_real_mismatch(self):
        self.meta['license'] = ''; self.failure()
        self.meta['license'] = 'test'; self.meta['kind'] = 'REAL_RECORDED'; self.failure()

    def test_rejects_escaping_symlink(self):
        external = self.root / 'external.csv'; external.write_bytes((self.source / 'raw.csv').read_bytes())
        (self.source / 'escape.csv').symlink_to(external)
        self.frames[0]['points_file'] = 'escape.csv'; self.failure()

    def test_rejects_duplicate_json_keys(self):
        self.assertRaises(ValueError, adapter.json_object, '{"a":1,"a":2}')

    def test_does_not_overwrite_output(self):
        self.output.mkdir(); sentinel = self.output / 'keep'; sentinel.write_text('preserve')
        with self.assertRaises(ValueError):
            adapter.convert_sequence(self.source, self.output)
        self.assertEqual(sentinel.read_text(), 'preserve')

    def test_cli_and_cpp_pipeline_integration(self):
        self.assertIsNotNone(RUNNER, 'CTest must pass the actual C++ runner')
        command = subprocess.run([sys.executable, str(Path(adapter.__file__)), '--source', str(self.source), '--output', str(self.output)], capture_output=True, text=True)
        self.assertEqual(command.returncode, 0, command.stderr)
        comparison = self.root / 'comparison'
        tool = Path(adapter.__file__).parent / 'benchmark_compare.py'
        run = subprocess.run([sys.executable, str(tool), '--runner', str(RUNNER), '--sequence', str(self.output), '--output', str(comparison)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        for mode in ('uniform-l0-observed', 'distance-only', 'full-avr'):
            summary = json.loads((comparison / mode / 'summary.json').read_text())
            # Low semantic confidence must not disable confirmed dynamic routing.
            self.assertEqual(summary['static_fusable_points'], 2)
            self.assertEqual(summary['dynamic_excluded_points'], 2)
            self.assertTrue(summary['valid'])
            self.assertIn('NOT REAL PERFORMANCE DATA', summary['data_truth'])


if __name__ == '__main__':
    unittest.main()
