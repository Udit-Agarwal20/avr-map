#!/usr/bin/env python3
"""Convert an explicitly labeled, posed recording export into AVR_SEQUENCE_V1.

Dataset-specific readers must supply this boundary. No dataset download, inferred
motion, implicit ontology mapping, timestamp synthesis or pose estimation occurs.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile

META_KEYS = {'schema_version', 'name', 'kind', 'recording_id', 'source_uri', 'license',
             'semantic_provenance', 'semantic_label_source', 'motion_label_source',
             'pose_provenance', 'coordinate_convention', 'bounds', 'semantic_class_map', 'frames'}
FRAME_KEYS = {'frame_seq', 'timestamp_ns', 'points_file', 'transform_map_from_sensor'}
INPUT_HEADER = ['source_index', 'x', 'y', 'z', 'intensity', 'label',
                'semantic_confidence', 'motion_state', 'motion_confidence']
OUTPUT_HEADER = ['source_index', 'x', 'y', 'z', 'intensity', 'semantic_class',
                 'semantic_confidence', 'motion_state', 'motion_confidence']
TRUTH = 'HARNESS VALIDATION FIXTURE ONLY / NOT REAL PERFORMANCE DATA'


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def json_object(data):
    value = json.loads(data, object_pairs_hook=unique_object)
    if not isinstance(value, dict):
        raise ValueError('JSON object required')
    return value


def text(value):
    if not isinstance(value, str) or not value.strip() or any(ord(c) < 32 for c in value):
        raise ValueError('nonempty single-line provenance text required')
    return value


def uint(value, maximum=(1 << 64) - 1):
    if isinstance(value, str):
        if not value or any(c not in '0123456789' for c in value):
            raise ValueError('unsigned decimal integer required')
        value = int(value)
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError('unsigned integer outside supported range')
    return value


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError('finite number required')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('finite number required')
    return number


def confidence(value):
    number = finite(value)
    if not 0 <= number <= 1:
        raise ValueError('confidence outside [0,1]')
    return number


def relative_file(root, relative):
    path = Path(text(relative))
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('input path must stay within recording directory')
    resolved = (root / path).resolve(strict=True)
    if not resolved.is_relative_to(root) or not resolved.is_file():
        raise ValueError('input symlink/path escapes recording directory')
    return resolved


def validate_transform(values):
    if not isinstance(values, list) or len(values) != 16:
        raise ValueError('row-major 4x4 sensor-to-map rigid transform required')
    matrix = [finite(v) for v in values]
    if matrix[12:] != [0, 0, 0, 1]:
        raise ValueError('invalid homogeneous transform bottom row')
    rotation = [matrix[i:i + 3] for i in (0, 4, 8)]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[i][k] * rotation[j][k] for k in range(3))
            if not math.isclose(dot, 1.0 if i == j else 0.0, rel_tol=0, abs_tol=1e-6):
                raise ValueError('pose rotation is not orthonormal (no scale/shear allowed)')
    a, b, c = rotation
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1]) -
                   a[1] * (b[0] * c[2] - b[2] * c[0]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0]))
    if not math.isclose(determinant, 1, rel_tol=0, abs_tol=1e-6):
        raise ValueError('pose must be a proper right-handed rotation')
    return matrix


def transform_point(matrix, point):
    return tuple(finite(sum(matrix[r * 4 + c] * point[c] for c in range(3)) + matrix[r * 4 + 3])
                 for r in range(3))


def hash_field(hasher, value):
    hasher.update(str(len(value)).encode('ascii') + b':' + value)


def scalar(value):
    return format(value, '.17g')


def validate_metadata(meta):
    if set(meta) != META_KEYS or type(meta['schema_version']) is not int or meta['schema_version'] != 1:
        raise ValueError('unsupported/malformed recording metadata schema')
    for key in META_KEYS - {'schema_version', 'bounds', 'semantic_class_map'}:
        text(meta[key])
    if meta['kind'] not in ('REAL_RECORDED', 'HARNESS_VALIDATION_FIXTURE'):
        raise ValueError('unknown recording kind')
    if meta['semantic_provenance'] not in ('GROUND_TRUTH_LABELS', 'PRECOMPUTED_LABELS', 'SYNTHETIC_VALIDATION_LABELS'):
        raise ValueError('explicit semantic provenance required')
    if (meta['kind'] == 'HARNESS_VALIDATION_FIXTURE') != (meta['semantic_provenance'] == 'SYNTHETIC_VALIDATION_LABELS'):
        raise ValueError('synthetic/real provenance conflict')
    if meta['coordinate_convention'] != 'RIGHT_HANDED_Z_UP_METRES':
        raise ValueError('normalize input axes/units explicitly before conversion')
    if not isinstance(meta['bounds'], list) or len(meta['bounds']) != 4:
        raise ValueError('four half-open XY bounds required')
    bounds = [finite(v) for v in meta['bounds']]
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('empty/reversed bounds')
    mapping = meta['semantic_class_map']
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError('explicit dataset-label to canonical-class map required')
    for label, code in mapping.items():
        text(label)
        if type(code) is not int:
            raise ValueError('canonical semantic class must be an integer code')
        uint(code, 7)
    return bounds


def convert_sequence(source, output):
    source = Path(source).resolve(strict=True)
    output = Path(output)
    if output.exists():
        raise ValueError('output exists; never overwrite a conversion')
    metadata_bytes = relative_file(source, 'recording.json').read_bytes()
    meta = json_object(metadata_bytes)
    bounds = validate_metadata(meta)
    index_bytes = relative_file(source, meta['frames']).read_bytes()
    source_hash = hashlib.sha256()
    hash_field(source_hash, metadata_bytes); hash_field(source_hash, index_bytes)
    output.parent.mkdir(parents=True, exist_ok=True)
    frame_count = point_count = 0
    previous = None
    # Publish the output only after all frames have validated; failed temporary
    # conversion is removed, and original recording files are never modified.
    with tempfile.TemporaryDirectory(prefix='.avr-convert-', dir=output.parent) as temporary:
        staging = Path(temporary) / 'sequence'
        staging.mkdir()
        with (staging / 'frames.csv').open('w', newline='', encoding='utf-8') as frame_file:
            frame_writer = csv.writer(frame_file, lineterminator='\n')
            frame_writer.writerow(['frame_seq', 'timestamp_ns', 'sensor_x', 'sensor_y', 'sensor_z', 'points_file'])
            for line in io.StringIO(index_bytes.decode('utf-8')):
                frame = json_object(line)
                if set(frame) != FRAME_KEYS:
                    raise ValueError('malformed frame record')
                seq, timestamp = uint(frame['frame_seq']), uint(frame['timestamp_ns'])
                if previous and (seq <= previous[0] or timestamp <= previous[1]):
                    raise ValueError('frame sequence and timestamps must strictly increase')
                previous = (seq, timestamp)
                matrix = validate_transform(frame['transform_map_from_sensor'])
                original = relative_file(source, frame['points_file']).read_bytes()
                hash_field(source_hash, original)
                reader = csv.reader(io.StringIO(original.decode('utf-8')), strict=True)
                if next(reader, None) != INPUT_HEADER:
                    raise ValueError('missing original points, intensity or explicit labels/motion columns')
                point_name = f'points-{frame_count:08d}.csv'
                with (staging / point_name).open('w', newline='', encoding='utf-8') as point_file:
                    writer = csv.writer(point_file, lineterminator='\n')
                    writer.writerow(OUTPUT_HEADER)
                    for ordinal, row in enumerate(reader):
                        if len(row) != len(INPUT_HEADER) or uint(row[0], (1 << 32) - 1) != ordinal:
                            raise ValueError('point source indices must retain original consecutive ordinals')
                        xyz = transform_point(matrix, [finite(v) for v in row[1:4]])
                        if not (bounds[0] <= xyz[0] < bounds[1] and bounds[2] <= xyz[1] < bounds[3]):
                            raise ValueError('transformed point outside bounds; no clipping/filtering permitted')
                        for coordinate in xyz[:2]:
                            grid = coordinate * 20.0
                            if not math.isfinite(grid) or not -(1 << 31) <= math.floor(grid) < (1 << 31):
                                raise ValueError('point outside canonical 5 cm quantization domain')
                        intensity = finite(row[4])
                        if row[5] not in meta['semantic_class_map']:
                            raise ValueError(f'unmapped dataset semantic label: {row[5]}')
                        label = meta['semantic_class_map'][row[5]]
                        semantic_conf = confidence(row[6])
                        motion, motion_conf = uint(row[7], 3), confidence(row[8])
                        writer.writerow([ordinal, *map(scalar, xyz), scalar(intensity), label,
                                         scalar(semantic_conf), motion, scalar(motion_conf)])
                        point_count += 1
                frame_writer.writerow([seq, timestamp, *[scalar(matrix[i]) for i in (3, 7, 11)], point_name])
                frame_count += 1
        if not frame_count:
            raise ValueError('empty recording index')
        digest = source_hash.hexdigest()
        description = f'{meta["recording_id"]}; {meta["source_uri"]}; source SHA256={digest}'
        fields = {
            'name': meta['name'], 'kind': meta['kind'], 'semantic_provenance': meta['semantic_provenance'],
            'source_description': description, 'license': meta['license'],
            'pose_provenance': meta['pose_provenance'], 'coordinate_frame': 'FIXED_MAP_FRAME',
            'bounds': ','.join(map(scalar, bounds)), 'frames': 'frames.csv',
        }
        (staging / 'sequence.avr').write_text('AVR_SEQUENCE_V1\n' + ''.join(f'{k}={v}\n' for k, v in fields.items()), encoding='utf-8')
        provenance = {'schema_version': 1, 'source_content_hash': digest, 'recording': meta,
                      'frame_count': frame_count, 'point_count': point_count,
                      'data_truth': TRUTH if meta['kind'] == 'HARNESS_VALIDATION_FIXTURE' else 'REAL RECORDED DATA / OFFLINE INPUT ADAPTER',
                      'conversion': 'row-major rigid T_map_sensor; source ordinals/intensity/evidence/time preserved; no filtering',
                      'hash_algorithm': 'SHA-256 length-prefixed exact recording.json, index, then input point files in frame order'}
        (staging / 'adapter_provenance.json').write_text(json.dumps(provenance, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if output.exists():
            raise ValueError('output appeared during conversion; refusing to overwrite')
        staging.rename(output)
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = convert_sequence(args.source, args.output)
    except (ValueError, OSError, TypeError, KeyError, csv.Error, OverflowError) as error:
        parser.exit(2, f'INVALID RECORDED INPUT: {error}\n')
    print(result['data_truth'])
    print(f'Converted {result["frame_count"]} frames and {result["point_count"]} original-order points')


if __name__ == '__main__':
    main()
