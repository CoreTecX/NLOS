"""Strict AArch64 Linux build without BSP/libdrm dependency."""
from pathlib import Path
import argparse
import subprocess


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cc', required=True, help='aarch64-linux-gnu-gcc executable')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--static', action='store_true', help='avoid target glibc version dependencies')
    args = ap.parse_args()
    r = args.repo.resolve()
    incs = ['board', 'src/software/include', 'src/software/fpga_control',
            'src/software/board_app', 'src/software/sd_frame', 'src/software/meas_norm']
    sources = ['src/software/board_app/'+s+'.c' for s in
               ['board_main', 'board_top', 'board_regseq', 'board_fb', 'ps_cache',
                'board_pmem', 'board_pmem_posix', 'frame_pipeline', 'frame_runtime', 'frame_display']]
    sources += ['src/software/fpga_control/'+s+'.c' for s in
                ['fix_core_driver', 'uio_backend', 'uio_backend_posix']]
    sources += ['src/software/meas_norm/meas_norm.c', 'src/software/sd_frame/sd_frame.c',
                'src/software/model_loader/psm_crc32.c', 'board/fix_core.c']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([args.cc, '-std=c11', '-O2', '-D_POSIX_C_SOURCE=200809L',
                    '-DPS_BOARD_APP_MAIN', '-Wall', '-Wextra', '-Werror',
                    *['-I'+str(r/i) for i in incs], *[str(r/s) for s in sources],
                    *(['-static'] if args.static else []), '-lm', '-o', str(args.output)], check=True)
    elf = args.output.read_bytes()[:64]
    if elf[:6] != b'\x7fELF\x02\x01' or int.from_bytes(elf[18:20], 'little') != 183:
        raise SystemExit('ERROR: output is not ELF64 little-endian AArch64')
    print('AARCH64_RELEASE_BUILD_PASS', args.output)


if __name__ == '__main__':
    main()
