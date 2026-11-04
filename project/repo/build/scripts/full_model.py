"""Build and verify the complete M8 algorithm from the repository's module sources.

The vendor tools and mingw runtime require an ASCII work directory. A manifest
assembles the original release layout there without maintaining a second source
copy. No command in this entry point accesses or programs a board.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
FLAGS = ['-std=c++17', '-O3', '-ffp-contract=off', '-fno-fast-math',
         '-Wall', '-Wextra', '-Werror', '-Wno-unknown-pragmas', '-Wno-unused-label']
HOST = 'D:/CCC/mingw64/bin/g++.exe'
ARM = 'D:/vivado/2025.2/gnu/aarch64/nt/aarch64-linux/bin/aarch64-linux-gnu-g++.exe'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', 'utf-8')


def contained(root, relative):
    path = (root / relative).resolve()
    if path == root or root not in path.parents:
        raise ValueError('Path leaves its root: ' + str(relative))
    return path


def command(args, cwd, log, timeout=1800):
    result = subprocess.run([str(x) for x in args], cwd=cwd, capture_output=True,
                            encoding='utf-8', errors='replace', timeout=timeout)
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(result.stdout + result.stderr, 'utf-8')
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}); see {log}\n'
                           + (result.stdout + result.stderr)[-3000:])
    return result.stdout


def assemble(repo, stage, config):
    sources = []
    destinations = set()
    for entry in config['sources']:
        src = contained(repo, entry['repository'])
        dst = contained(stage, entry['release'])
        if dst in destinations:
            raise ValueError('Duplicate assembly destination: ' + str(dst))
        destinations.add(dst)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        digest = sha(src)
        if sha(dst) != digest:
            raise RuntimeError('Source copy mismatch: ' + str(src))
        sources.append(dict(repository=entry['repository'], release=entry['release'],
                            sha256=digest,
                            matches_frozen_release=digest == entry['frozen_sha256'] if entry.get('frozen_sha256') else None))
    for name in ('bin', 'reports', 'build'):
        (stage / name).mkdir(exist_ok=True)
    return sources


def build(stage, target, host, arm, profile=False):
    ext = '.exe' if os.name == 'nt' else ''
    jobs = []
    if target in ('host', 'all'):
        for name, extra, inputs in [
            ('ps_full_host', [], ['src/main.cpp', 'src/reference_gemv.cpp']),
            ('ps_full_legacy_host', ['-DPS_DCT_LEGACY'], ['src/main.cpp', 'src/reference_gemv.cpp']),
            ('ps_full_packed_host', ['-DPS_TEST_PACKED_HOST'], ['src/main.cpp', 'src/full_gemv.cpp', 'src/reference_gemv.cpp']),
            ('dct_test', [], ['tests/dct_test.cpp']),
            ('pl_job_tests', [], ['tests/pl_job_tests.cpp']),
            ('numerical_experiments', [], ['tests/numerical_main.cpp','src/reference_gemv.cpp']),
            ('numerical_dct_tests', [], ['tests/numerical_dct_tests.cpp']),
            ('gemv_tb', [], ['tests/gemv_tb.cpp', 'src/full_gemv.cpp', 'src/reference_gemv.cpp']),
            ('packing_boundaries', [], ['tests/packing_boundaries.cpp']),
            ('runtime_tests', [], ['tests/runtime_tests.cpp','src/full_gemv.cpp','src/reference_gemv.cpp']),
        ]:
            jobs.append((name + ext, host, extra, inputs))
    if target in ('arm', 'all'):
        for name, extra, inputs in [
            ('ps_full_aarch64', [], ['src/main.cpp', 'src/reference_gemv.cpp']),
            ('ps_full_legacy_aarch64', ['-DPS_DCT_LEGACY'], ['src/main.cpp', 'src/reference_gemv.cpp']),
            ('dct_test_aarch64', [], ['tests/dct_test.cpp']),
            ('pl_job_tests_aarch64', [], ['tests/pl_job_tests.cpp']),
            ('numerical_experiments_aarch64', [], ['tests/numerical_main.cpp','src/reference_gemv.cpp']),
            ('numerical_dct_tests_aarch64', [], ['tests/numerical_dct_tests.cpp']),
            ('dct_test_portable_aarch64', ['-DPS_DCT_PORTABLE'], ['tests/dct_test.cpp']),
            ('runtime_tests_aarch64', [], ['tests/runtime_tests.cpp','src/full_gemv.cpp','src/reference_gemv.cpp']),
        ]:
            jobs.append((name, arm, ['-static', '-mcpu=cortex-a53', *extra], inputs))
    if profile:
        extra_jobs=[]
        for name, compiler, extra, inputs in jobs:
            if name.startswith('ps_full_') and 'legacy' not in name:
                stem=name[:-4] if name.endswith('.exe') else name
                suffix='.exe' if name.endswith('.exe') else ''
                extra_jobs.append((stem+'_profile'+suffix,compiler,[*extra,'-DPS_ENABLE_PHASE_PROFILE'],inputs))
        jobs.extend(extra_jobs)
        if target in ('host','all'):
            jobs.append(('profile_tests'+ext,host,[],['tests/profile_tests.cpp']))
    compilers = {}
    for _, compiler, _, _ in jobs:
        if compiler in compilers:
            continue
        located = shutil.which(compiler)
        if located is None:
            raise RuntimeError('Compiler unavailable: ' + compiler)
        compilers[compiler] = dict(path=str(Path(located).resolve()), sha256=sha(located),
            version=subprocess.check_output([located, '--version'], encoding='utf-8', errors='replace'))
    records = []
    for name, compiler, extra, inputs in jobs:
        args = [compilers[compiler]['path'], *FLAGS, *extra, *inputs, '-o', 'bin/' + name]
        command(args, stage, stage / 'reports' / ('build_' + name + '.log'))
        binary = stage / 'bin' / name
        if 'aarch64' in name:
            elf = binary.read_bytes()
            if elf[:6] != b'\x7fELF\x02\x01' or struct.unpack_from('<H', elf, 18)[0] != 183:
                raise RuntimeError('Not an AArch64 ELF: ' + name)
            offset = struct.unpack_from('<Q', elf, 32)[0]
            size, count = struct.unpack_from('<HH', elf, 54)
            if {2, 3}.intersection(struct.unpack_from('<I', elf, offset+i*size)[0] for i in range(count)):
                raise RuntimeError('Unexpected dynamic dependency in ' + name)
        records.append(dict(output='bin/' + name, sha256=sha(binary), command=args))
        print('BUILT', name, flush=True)
    return dict(compilers=compilers, jobs=records)


def unit_tests(stage):
    ext = '.exe' if os.name == 'nt' else ''
    results = {}
    for name in ('dct_test', 'gemv_tb', 'packing_boundaries', 'runtime_tests', 'pl_job_tests'):
        text = command([stage / 'bin' / (name + ext)], stage, stage / 'reports' / (name + '.log'))
        results[name] = dict(passed=True, output=text.strip())
        print('PASS', name, flush=True)
    dct = json.loads(results['dct_test']['output'])
    if dct.get('passed') is not True or dct.get('bit_exact') is not True or dct.get('cases') != 3220:
        raise RuntimeError('Unexpected DCT self-test result')
    command([sys.executable, '-B', stage / 'tests/launcher_checks.py'], stage,
            stage / 'reports/launcher_checks.log')
    results['launcher'] = json.loads((stage / 'reports/launcher_checks.json').read_text('utf-8'))
    if results['launcher'].get('passed') is not True:
        raise RuntimeError('Launcher tests did not pass')
    command([sys.executable,'-B',stage/'tests/clock_profile_tests.py'],stage,stage/'reports/clock_profile_tests.log')
    results['clock_profiles']=json.loads((stage/'reports/clock_profile_tests.json').read_text('utf-8'))
    return results


def stage_inputs(stage, release, case):
    data = stage / 'data'
    data.mkdir()
    frozen = json.loads((release / 'release_manifest.json').read_text('utf-8'))
    if frozen.get('release_id') != '20261015-rp2-m8-v1' or frozen.get('layout_version') != 0x32525001:
        raise RuntimeError('Wrong reference release')
    manifest = release / 'data/manifest.json'
    if sha(manifest) != frozen['files']['data/manifest.json']['sha256']:
        raise RuntimeError('Reference data manifest mismatch')
    rows = json.loads(manifest.read_text('utf-8'))
    selected = [r for r in rows if r['key'] == case] if case else rows
    if not selected:
        raise ValueError('Unknown case: ' + str(case))
    shutil.copy2(manifest, data / 'manifest.json')
    copied = set()
    for row in selected:
        for kind in ('model', 'measurement', 'golden'):
            entry = row[kind]
            if entry['file'] in copied:
                continue
            src = contained((release / 'data').resolve(), entry['file'])
            dst = contained(data.resolve(), entry['file'])
            if src.stat().st_size != entry['bytes'] or sha(src) != entry['sha256']:
                raise RuntimeError('Reference input mismatch: ' + str(src))
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            if sha(dst) != entry['sha256']:
                raise RuntimeError('Staged input mismatch: ' + str(dst))
            copied.add(entry['file'])
    return selected


def regression(stage, release, variant, case):
    selected = stage_inputs(stage, release, case)
    data = stage / 'data'
    names = ('native', 'packed') if variant == 'both' else (variant,)
    ext = '.exe' if os.name == 'nt' else ''
    reports = {}
    for name in names:
        out = stage / 'results' / name
        out.mkdir(parents=True)
        exe = 'ps_full_host' if name == 'native' else 'ps_full_packed_host'
        for row in selected:
            command([stage / 'bin' / (exe + ext), data / row['model']['file'],
                     data / row['measurement']['file'], out / row['key'], 'cpu'],
                    stage, out / (row['key'] + '.log'))
            print('SOLVED', name, row['key'], flush=True)
        command([sys.executable, '-B', stage / 'scripts/accept_board.py', out,
                 '--report', out / 'comparison.json', *([] if case else ['--all'])],
                stage, out / 'comparison.log')
        reports[name] = json.loads((out / 'comparison.json').read_text('utf-8'))
        if reports[name].get('passed') is not True or len(reports[name]['cases']) != len(selected):
            raise RuntimeError('Incomplete regression: ' + name)
    return reports


def release_check_paths(repo, config, explicit_release=None, package='all'):
    """Current packages are the default; the immutable M8 reference is opt-in."""
    if explicit_release is not None:
        if package != 'all':
            raise ValueError('--release cannot be combined with --package')
        return [explicit_release.resolve()]
    if package == 'frozen':
        return [(repo / config['frozen_release']).resolve()]
    catalog = json.loads((repo / 'configs/workspace_modules.json').read_text('utf-8'))
    if (catalog.get('software_revision') != config.get('software_revision') or
            catalog.get('hardware_release') != config.get('hardware_release')):
        raise ValueError('Workspace catalog does not match the active software/hardware')
    names = ('performance', 'fallback') if package == 'all' else (package,)
    paths = []
    for name in names:
        path = (repo / catalog['release_packages'][name]).resolve()
        manifest = json.loads((path / 'release_manifest.json').read_text('utf-8'))
        expected_profile = (config['recommended_performance_profile'] if name == 'performance'
                            else config['default_hardware_profile'])
        if (manifest.get('software_revision') != config['software_revision'] or
                manifest.get('hardware_release') != config['hardware_release'] or
                manifest.get('hardware_profile') != expected_profile or
                manifest.get('layout_version') != int(config['layout_abi'], 0)):
            raise ValueError('Wrong current release identity: ' + name)
        paths.append(path)
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['build', 'test', 'regress', 'verify', 'prepare-eda', 'release-check', 'workspace-check'])
    parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--release', type=Path, help='Frozen M8 data for build/regression; explicit package path for release-check')
    parser.add_argument('--package', choices=['all', 'performance', 'fallback', 'frozen'], default='all',
                        help='release-check only: default checks both current P7-P11 packages; frozen selects original M8')
    parser.add_argument('--target', choices=['host', 'arm', 'all'], default='all')
    parser.add_argument('--host', default=HOST if os.name == 'nt' else 'g++')
    parser.add_argument('--arm', default=ARM if os.name == 'nt' else 'aarch64-linux-gnu-g++')
    parser.add_argument('--work-root', type=Path, default=Path('D:/Temp/penumbrasight_m8') if os.name == 'nt' else Path('/tmp/penumbrasight_m8'))
    parser.add_argument('--output', type=Path, help='New result directory; must not exist')
    parser.add_argument('--variant', choices=['native', 'packed', 'both'], default='both')
    parser.add_argument('--profile', action='store_true', help='Also build opt-in phase profiling binaries (PS_PROFILE=1)')
    parser.add_argument('--case', help='One case; omission runs all 15 cases')
    args = parser.parse_args()
    if args.package != 'all' and args.action != 'release-check':
        parser.error('--package applies only to release-check')
    repo = args.repo_root.resolve()
    config = json.loads((repo / 'configs/full_model_m8.json').read_text('utf-8'))
    if args.action == 'workspace-check':
        checker = Path(__file__).with_name('workspace_check.py')
        options = ['--output', str(args.output)] if args.output else []
        return subprocess.call([sys.executable, '-B', str(checker), '--repo-root', str(repo), *options])
    release = (args.release or repo / config['frozen_release']).resolve()
    if args.action == 'release-check':
        for package in release_check_paths(repo, config, args.release, args.package):
            print('CHECK_PACKAGE', package, flush=True)
            code = subprocess.call([sys.executable, '-B', str(package / 'scripts/release_selfcheck.py')])
            if code:
                return code
        return 0
    work_root = args.work_root.resolve()
    if not str(work_root).isascii():
        raise ValueError('--work-root must be an ASCII path for the compiler and runtime')
    # A source tree or frozen release must never become a build output.
    if repo == work_root or repo in work_root.parents or release == work_root or release in work_root.parents:
        raise ValueError('--work-root must be outside the repository and frozen release')
    output = (args.output or repo / 'build/full_model_m8' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
    if output == release or release in output.parents:
        raise ValueError('Cannot write into the frozen release')
    if output.exists():
        raise ValueError('Output directory already exists: ' + str(output))
    output.mkdir(parents=True)
    work_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='run_', dir=work_root)).resolve()
    record = dict(action=args.action, started_at=datetime.now(timezone.utc).isoformat(),
                  repo_root=str(repo), staging=str(stage), frozen_release=str(release),
                  physical_board_test=False,
                  new_optimization=config.get('release_id') != '20261015-rp2-m8-v1',
                  candidate_release_id=config.get('release_id'), passed=False)
    print('STAGING', stage, flush=True)
    try:
        record['sources'] = assemble(repo, stage, config)
        if args.action == 'prepare-eda':
            stage_inputs(stage, release, 'D11_rgb')
            record['eda_command'] = 'powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1'
        else:
            target = 'host' if args.action in ('test', 'regress') else args.target
            record['build'] = build(stage, target, args.host, args.arm, args.profile)
            if args.action in ('test', 'regress', 'verify'):
                if target == 'arm':
                    raise ValueError('Tests require --target host or all')
                record['unit_tests'] = unit_tests(stage)
            if args.action in ('regress', 'verify'):
                record['regression'] = regression(stage, release, args.variant, args.case)
        record['passed'] = True
    except Exception as error:
        record['error'] = str(error)
        raise
    finally:
        # Keep the isolated stage for diagnosis/EDA; never recursively clean user paths.
        for name in ('bin', 'reports', 'results'):
            if (stage / name).is_dir():
                shutil.copytree(stage / name, output / name)
        record['finished_at'] = datetime.now(timezone.utc).isoformat()
        write_json(output / 'verification.json', record)
        print('RESULT', output / 'verification.json', flush=True)
    print('M8_REPOSITORY_PASS', args.action, flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print('FAILED:', error, file=sys.stderr)
        raise SystemExit(1)
