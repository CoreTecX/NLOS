"""Independent RAW-to-HLS arithmetic gate; no UIO/shadow substitute."""
from pathlib import Path
import argparse
from dataclasses import replace
import json
import os
import shlex
import subprocess
import sys
import tempfile
import zlib
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--production-model', type=Path)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    r = args.repo.resolve()
    sys.path.insert(0, str(r/'src/model'))
    sys.path.insert(0, str(r/'build/scripts'))
    from penumbrasight_model import psm
    import pack_paper_frame as pack
    cc = shlex.split(os.environ.get('CC', 'gcc'), posix=os.name != 'nt')
    cxx = shlex.split(os.environ.get('CXX', 'g++'), posix=os.name != 'nt')
    with tempfile.TemporaryDirectory(prefix='ps_frame_') as td:
        d = Path(td)
        incs = ['board', 'src/software/include', 'src/software/fpga_control',
                'src/software/board_app', 'src/software/sd_frame',
                'src/software/meas_norm', 'src/fpga/hls']
        flags = ['-O2', '-Wall', '-Wextra', '-Werror', '-D__USE_MINGW_ANSI_STDIO=1']
        flags += ['-I'+str(r/i) for i in incs]
        sources = ['src/software/board_app/'+s+'.c' for s in
                   ['frame_pipeline', 'frame_display', 'board_top', 'board_regseq', 'board_fb', 'ps_cache']]
        sources += ['src/software/fpga_control/'+s+'.c' for s in ['fix_core_driver', 'uio_backend']]
        sources += ['src/software/meas_norm/meas_norm.c', 'src/software/sd_frame/sd_frame.c',
                    'src/software/model_loader/psm_crc32.c', 'board/fix_core.c']
        objects = []
        for i, source in enumerate(sources):
            obj = d/f'{i}.o'
            subprocess.run(cc+['-std=c11', *flags, '-c', str(r/source), '-o', str(obj)], check=True)
            objects.append(str(obj))
        hls = ['ps_fixed_top.cpp', 'projection_engine/projection_engine_fixed.cpp',
               'lr_operator/lr_operator_fixed.cpp', 'pdhg_core/pdhg_core_fixed.cpp']
        exe = d/('frame_test.exe' if os.name == 'nt' else 'frame_test')
        subprocess.run(cxx+['-std=c++17', *flags, '-Wno-unknown-pragmas',
                           str(r/'src/software/board_app/test_frame_pipeline.cpp'), *objects,
                           *[str(r/'src/fpga/hls'/s) for s in hls], '-o', str(exe)], check=True)
        header = psm.PsmHeader(meas_h=95, meas_w=96, hidden_h=18, hidden_w=24,
                               candidates=3, rank_max=1, rank_default=1, loc_rank=1,
                               geometry_hash=0x13572468)
        records = []
        for i in range(3):
            u = np.zeros((1, 9120), dtype=np.int16)
            u[0, np.arange(64)*95+(1, 0, 2)[i]] = 4096  # ||U||=1 exactly
            v = np.zeros((1, 432), dtype=np.int16)
            v[0, 3*18+2] = 32767
            records.append(psm.CandidateRecord(x_mm=i, wall_distance_mm=400, z_mm=200,
                           u_scale=2**-15, s_scale=2**-14, v_scale=2**-15, s_max=1.,
                           U=u, S=np.array([16384], dtype=np.uint16), V=v,
                           observable_mask=np.full(432, 255, dtype=np.uint8)))
        model = d/'model.psm'
        psm.write_model(str(model), header, records)
        raw = np.full((1920, 1920), 100, dtype=np.uint16)
        raw[20:] = 200
        payload = raw.astype('<u2').tobytes()
        hdr = dict(width=1920, height=1920, valid_bits=12, container_bits=16,
                   bayer_phase=0, value_packing=0, flags=1, frame_index=0,
                   payload_bytes=len(payload), scale_num=1, scale_den=1,
                   payload_crc32=zlib.crc32(payload), clipped_pixels=0,
                   payload_sum=int(raw.sum()), payload_sumsq=int((raw.astype(np.int64)**2).sum()),
                   payload_min=100, payload_max=200, median_code=200, p999_code=200,
                   scale_rule_ok=1, source_median=200, source_p999=200)
        frames = []
        for i in range(8):
            hdr['frame_index'] = i
            f = d/f'frame{i}.psf'
            f.write_bytes(pack.build_header(hdr)+payload)
            frames.append(f)
        for count in (1, 4, 8):
            prefix = d/f'result{count}'
            subprocess.run([str(exe), str(model), str(prefix), *map(str, frames[:count])], check=True)
            meta = json.loads(prefix.with_suffix('.json').read_text())
            image = np.fromfile(prefix.with_suffix('.codes.bin'), dtype='<i4').reshape(3, 18, 24)
            physical = np.ldexp(image.astype(float)/2**18, meta['image_scale_exp'])
            # Independent exact least squares for this rank-one operator:
            # measured block difference=10000, U_i=1/8, V=32767/32768.
            expected = count*10000/(0.125*(32767/32768))
            np.testing.assert_allclose(physical[:, 2, 3], expected, rtol=5e-5)
            physical[:, 2, 3] = 0
            assert not physical.any(), 'spurious hidden pixels'
            assert meta['frames'] == count and meta['selected_id'] == 1
        # Non-power-of-two s_max close to the production model exposes tiny
        # Q16 gain rounding; selected-S requantization must preserve amplitude.
        scaled_model = d/'scaled_model.psm'
        smax = float(np.float32(71.88))
        scaled_records = [replace(rec, s_max=smax) for rec in records]
        psm.write_model(str(scaled_model), header, scaled_records)
        prefix = d/'scaled_result'
        subprocess.run([str(exe), str(scaled_model), str(prefix), str(frames[0])], check=True)
        meta = json.loads(prefix.with_suffix('.json').read_text())
        image = np.fromfile(prefix.with_suffix('.codes.bin'), dtype='<i4').reshape(3, 18, 24)
        physical = np.ldexp(image.astype(float)/2**18, meta['image_scale_exp'])
        expected = 10000/(0.125*(32767/32768)*smax)
        np.testing.assert_allclose(physical[:, 2, 3], expected, rtol=5e-5)
        print('NON_POWER_S_GAIN_PASS relative_error=', float(abs(physical[0,2,3]/expected-1)))
        def reject(name, m, fs, expected_code):
            result = subprocess.run([str(exe), str(m), str(d/name), *map(str, fs)],
                                    capture_output=True, text=True)
            assert result.returncode == expected_code, (name, result.returncode, result.stderr)
            assert not (d/name).with_suffix('.json').exists(), name
        corrupt = bytearray(model.read_bytes()); corrupt[-1] ^= 1
        badmodel = d/'badmodel.psm'; badmodel.write_bytes(corrupt)
        reject('bad_model_crc', badmodel, frames[:1], 4)
        corrupt = bytearray(frames[0].read_bytes()); corrupt[-1] ^= 1
        badframe = d/'badframe.psf'; badframe.write_bytes(corrupt)
        reject('bad_frame_crc', model, [badframe], 5)
        reject('bad_sequence', model, [frames[0]]*4, 5)
        reject('bad_count', model, frames[:2], 5)
        hdr['frame_index'] = 3
        hdr['scale_num'] = 2
        badscale = d/'badscale.psf'; badscale.write_bytes(pack.build_header(hdr)+payload)
        reject('mixed_scales', model, frames[:3]+[badscale], 5)
        header = replace(header, geometry_hash=header.geometry_hash ^ 1)
        psm.write_model(str(badmodel), header, records)
        reject('bad_geometry', badmodel, frames[:1], 4)
        if args.production_model:
            prefix = d/'production_result'
            subprocess.run([str(exe), '--production', str(args.production_model), str(frames[0]), str(prefix)], check=True)
            print('PRODUCTION_DIAGNOSTICS', prefix.with_suffix('.json').read_text())
        print('PREBOARD_FRAME_GATE_PASS: analytical 1/4/8 frames, non-power S gain + 6 rejection cases')


if __name__ == '__main__':
    main()

