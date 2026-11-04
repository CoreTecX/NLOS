"""Export and verify a new, generic little-endian model contract. NOT the old board .psm format."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np


def export(model_path,destination):
    model_path=Path(model_path);destination=Path(destination)
    if destination.exists():raise FileExistsError('Use a new export directory to preserve earlier results')
    with np.load(model_path,allow_pickle=False) as p:pack={k:p[k] for k in p.files}
    wall=tuple(int(x) for x in pack['wall_shape']);scene=tuple(int(x) for x in pack['scene_shape'])
    n=int(np.prod(scene));canonical=np.arange(n).reshape(*scene,order='F').ravel()
    if pack['model'].dtype!=np.dtype('int16') or pack['model'].shape!=(int(np.prod(wall)),n):raise ValueError('Requires complete INT16 model')
    arrays={'coefficients_i16.bin':np.ascontiguousarray(pack['model'][:,canonical].T,dtype='<i2'),
        'column_scales_f64.bin':np.asarray(pack['column_scales'][canonical],dtype='<f8')}
    destination.mkdir(parents=True)
    rows=[]
    for name,a in arrays.items():
        path=destination/name;a.tofile(path)
        decoded=np.fromfile(path,dtype=a.dtype).reshape(a.shape)
        if not np.array_equal(decoded,a):raise AssertionError('Export roundtrip failed')
        rows.append(dict(file=name,dtype=a.dtype.str,shape=list(a.shape),order='C',bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    manifest=dict(format='penumbrasight-full-model-raw-v1',old_board_loader_compatible=False,
        source_model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),endianness='little',wall_shape=wall,scene_shape=scene,
        coefficient_axes=['scene_column_in_C_order','wall_pixel_in_MATLAB_F_order'],
        wall_index='row + wall_height * col',scene_index='row * scene_width + col',
        decode='A[wall,scene] = coefficients[scene,wall] * column_scales[scene]',
        model_norm=float(pack['model_norm']),difference='vertical then horizontal forward differences, each / sqrt(2); no boundary edge',
        rgb='three channels R,G,B; model shared by channels, channel scales provided with measurement',
        reconstruction='29x36x3, C order; float64 physical output; no GT and no display normalization',
        files=rows,roundtrip_verified=True,
        arithmetic_note='INT16 storage only; no fixed-point accumulator, RTL or register/DMA protocol implied')
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--model',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    m=export(a.model,a.output_dir);print(json.dumps(dict(format=m['format'],roundtrip_verified=m['roundtrip_verified'],files=m['files']),indent=2))
