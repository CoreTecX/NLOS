"""Read-only module/source/release integrity audit. Does not run board hardware."""
from __future__ import annotations
import argparse
import collections
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True


def native(path):
    path = Path(path).resolve()
    return Path('\\\\?\\' + str(path)) if os.name == 'nt' else path


def sha(path):
    digest = hashlib.sha256()
    with native(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    return json.loads(native(path).read_text(encoding='utf-8-sig'))


def contained(root, relative):
    value = Path(relative)
    path = (root / value).resolve()
    if value.is_absolute() or path == root or root not in path.parents:
        raise ValueError('Path escapes repository: ' + str(relative))
    return path


def inspect(repo, catalog=None, overlay=None):
    repo = Path(repo).resolve()
    overlay = Path(overlay).resolve() if overlay else None
    def source(relative):
        real = contained(repo, relative)
        if overlay:
            staged = contained(overlay, relative)
            if native(staged).is_file():
                return staged
        return real
    config = load(source('configs/full_model_m8.json'))
    registry = load(catalog or source('configs/workspace_modules.json'))
    errors, modules, sources, references = [], [], [], []
    if registry.get('schema_version') != 1:
        raise ValueError('Unsupported module registry')
    for field in ('software_revision', 'hardware_release', 'layout_abi'):
        if registry.get(field) != config.get(field):
            errors.append('Identity mismatch: ' + field)
    if registry.get('physical_board_test') is not False or registry.get('candidate_implementation') is not False:
        errors.append('Registry incorrectly promotes board or proposed work')
    rows = registry['modules']
    paths = [row['path'] for row in rows]
    if len(rows) != registry['module_count'] or len(set(paths)) != len(paths):
        errors.append('Missing/duplicate module registration')
    for row in rows:
        missing = []
        if not native(contained(repo, row['path'])).is_dir():
            missing.append(row['path'])
        for relative in [row['readme'], *row['entry_points'], *row['evidence']]:
            if not native(source(relative)).is_file():
                missing.append(relative)
        for dep in row['dependencies']:
            if dep not in paths:
                errors.append('Unknown module dependency: ' + dep)
        if not row['preservation_reason'] or not row['boundary']:
            errors.append('Incomplete module boundary: ' + row['path'])
        if missing:
            errors.append(dict(module=row['path'], missing=missing))
        modules.append(dict(path=row['path'], passed=not missing, classification=row['classification'],
                            evidence_scope='Entry/evidence existence, not a new execution test'))
    packages = {}
    for name in ('performance', 'fallback'):
        root = (repo / registry['release_packages'][name]).resolve()
        manifest = load(root / 'release_manifest.json')
        profile = config['recommended_performance_profile'] if name == 'performance' else config['default_hardware_profile']
        ok = (manifest.get('software_revision') == config['software_revision'] and
              manifest.get('hardware_release') == config['hardware_release'] and
              manifest.get('layout_version') == int(config['layout_abi'], 0) and
              manifest.get('hardware_profile') == profile and
              manifest.get('physical_board_test') is False)
        if not ok:
            errors.append('Package identity mismatch: ' + name)
        packages[name] = root
    destinations = set()
    for row in config['sources']:
        if row['release'] in destinations:
            errors.append('Duplicate release source: ' + row['release'])
        destinations.add(row['release'])
        path = source(row['repository'])
        digest = sha(path)
        matches = {}
        for name, root in packages.items():
            target = contained(root, row['release'])
            matches[name] = native(target).is_file() and sha(target) == digest
        if not all(matches.values()):
            errors.append(dict(source=row['repository'], packages=matches))
        if not any(row['repository'].startswith(p.rstrip('/') + '/') for p in paths):
            errors.append('Source not assigned to a module: ' + row['repository'])
        sources.append(dict(repository=row['repository'], sha256=digest, matches_packages=matches))
    for row in config['python_model_sources']:
        digest = sha(source(row['repository']))
        ok = digest == row['sha256']
        if not ok:
            errors.append('Reference changed: ' + row['repository'])
        references.append(dict(repository=row['repository'], sha256=digest, passed=ok))
    table = source('report/optimization_plan_v3/tasks.csv').read_text('utf-8-sig')
    counts = dict(collections.Counter(r['状态'] for r in csv.DictReader(io.StringIO(table))))
    if counts != registry['v3_counts']:
        errors.append('V3 task counts differ from module catalog')
    return dict(passed=not errors, inspection='module and artifact integrity, not board or numerical acceptance',
                software_revision=config['software_revision'], hardware_release=config['hardware_release'],
                modules=modules, module_count=len(modules), sources=sources, source_count=len(sources),
                references=references, reference_count=len(references), v3_counts=counts,
                physical_board_test=False, candidate_implementation=False, errors=errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--catalog', type=Path, help='Alternative module registry for offline review')
    parser.add_argument('--overlay', type=Path, help='Read staged files over the repository; writes nothing')
    parser.add_argument('--output', type=Path, help='Write a new JSON report; must not already exist')
    args = parser.parse_args()
    try:
        result = inspect(args.repo_root, args.catalog, args.overlay)
    except (ValueError, KeyError, OSError, TypeError) as error:
        result = dict(passed=False, physical_board_test=False, errors=[str(error)])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
    summary = {key: value for key, value in result.items() if key not in ('modules', 'sources', 'references')}
    print(json.dumps(summary, ensure_ascii=True, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
