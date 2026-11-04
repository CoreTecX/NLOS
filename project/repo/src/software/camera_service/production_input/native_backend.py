"""Pinned full solver launch plans for Windows and AArch64 Linux (Python 3.7+)."""
from pathlib import Path
import os
import platform
import struct
import sys

from contracts import require, pinned, sha
from inputs import release_file


def within(path, root):
    path, root = Path(path).resolve(), Path(root).resolve()
    return path == root or root in path.parents


def elf_identity(data):
    require(len(data) >= 64 and data[:6] == b'\x7fELF\x02\x01', 'ELF_FORMAT', 'Expected little-endian ELF64')
    machine = struct.unpack_from('<H', data, 18)[0]
    offset = struct.unpack_from('<Q', data, 32)[0]
    size, count = struct.unpack_from('<HH', data, 54)
    require(machine == 183 and size >= 56 and count > 0 and offset+size*count <= len(data),
            'ELF_ARCH', 'Expected bounded AArch64 program headers')
    types = [struct.unpack_from('<I', data, offset+i*size)[0] for i in range(count)]
    require(3 not in types and 2 not in types, 'ELF_DYNAMIC', 'Expected static solver without interpreter/dynamic segment')
    return dict(machine=machine, bits=64, endian='little', static=True)


def platform_target(system=None, machine=None):
    system, machine = system or platform.system(), (machine or platform.machine()).lower()
    if system == 'Windows' and machine in ('amd64', 'x86_64'):
        return 'windows_x86_64'
    if system == 'Linux' and machine in ('aarch64', 'arm64'):
        return 'linux_aarch64'
    require(False, 'RUNTIME_PLATFORM', 'Supported targets: Windows x64 and Linux AArch64')


def validate_runtime(config, system=None, machine=None):
    target = platform_target(system, machine)
    require(config.get('runtime_target', 'auto') in ('auto', target), 'RUNTIME_PLATFORM', 'Config target differs from this machine')
    require(sys.version_info >= (3, 7), 'PYTHON_VERSION', 'Python 3.7 or newer required')
    mode = config.get('backend_mode', 'cpu')
    require(mode in ('cpu', 'pl'), 'BACKEND_MODE', 'Choose cpu or pl')
    require(mode == 'cpu' or target == 'linux_aarch64', 'PL_PLATFORM', 'PL requires AArch64 Linux')
    return target, mode


def pl_readiness(config, workspace, manifest):
    import json
    require(config.get('allow_pl') is True and config.get('pl_readiness') is not None,
            'PL_NOT_AUTHORIZED', 'PL needs an explicit current-boot readiness receipt')
    record = json.loads(pinned(workspace, config['pl_readiness']))
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    require(record.get('boot_id') == boot and record.get('passed') is True and
            record.get('hardware_release') == manifest['hardware_release'] and
            record.get('layout_version') == manifest['layout_version'],
            'PL_READINESS', 'Readiness boot/release/ABI mismatch')
    require(record.get('profile') in ('p56_150_b128', 'p56_100') and
            record.get('profile') == config.get('hardware_profile'), 'PL_PROFILE', 'Unmatched hardware profile')
    return record


def prepare_native(config, workspace, release, manifest, runtime, resources=None):
    target, mode = validate_runtime(config)
    receipt = pl_readiness(config, workspace, manifest) if mode == 'pl' else None
    name = config['host_binary'] if target == 'windows_x86_64' else config.get('arm_binary', 'bin/ps_full_aarch64')
    data = release_file(release, manifest, name) if resources is None else resources.release(release,manifest,name)
    identity = {'format':'PE', 'target':target}
    if target == 'linux_aarch64':
        identity = elf_identity(data)
    else:
        require(data[:2] == b'MZ', 'EXE_FORMAT', 'Expected Windows executable')
    binary = runtime / ('solver.exe' if target == 'windows_x86_64' else 'solver')
    binary.write_bytes(data)
    if target == 'linux_aarch64':
        binary.chmod(0o700)
    libraries = {}
    for record in config.get('host_libraries', []) if target == 'windows_x86_64' else []:
        content = pinned(workspace, record) if resources is None else resources.get(workspace,record)
        filename = record.get('runtime_name', Path(record['path']).name)
        require(filename not in ('solver', 'solver.exe'), 'LIBRARY_NAME', 'Library overlaps executable')
        (runtime/filename).write_bytes(content)
        libraries[filename] = sha(content)
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('PS_'):
            env.pop(key)
    env.update(PS_EXPORT_STATE='1', PS_PL_SCHEDULE='sync',
               PATH=str(runtime)+os.pathsep+env.get('PATH', ''))
    command = [str(binary), 'model.pfm', 'measurement.pfy', 'result', mode, '20000']
    return dict(command=command, env=env, binary=name, libraries=libraries,
                target=target, backend=mode, identity=identity, readiness=receipt,
                hardware_accessed=mode == 'pl')
