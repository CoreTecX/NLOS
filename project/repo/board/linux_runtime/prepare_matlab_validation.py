"""Recreate deployed inputs with exact SD CRC and prepare matched MATLAB reference."""
from pathlib import Path
import sys,re,base64,json,struct,zlib,importlib.util,subprocess
import numpy as np
from scipy.io import loadmat,savemat

root=Path(__file__).resolve().parents[2]
ws=root.parent.parent
out=ws/'03_任务产出'/'S8-26_Board_Verified_20261006'
data=ws.parent/'论文数据集'/'Ordinary-Camera-master'/'Data'
sys.path.insert(0,str(root/'src/model'))
from penumbrasight_model.psm import PsmHeader,_unpack_record
spec=importlib.util.spec_from_file_location('packer',ws/'03_任务产出/S8-14_SD卡喂数据链路/scripts/pack_paper_frame.py')
packer=importlib.util.module_from_spec(spec);spec.loader.exec_module(packer)
log=(out/'validation_serial_20261007.log').read_text(encoding='utf-8-sig')
headers={name:base64.b64decode(encoded) for name,encoded in re.findall(r'FRAME_HEADER (\w+\.psf)\s+([A-Za-z0-9+/=]{172})',log)}
assert len(headers)==13,len(headers)
previous={}
for fn in ['remaining_suite.json','board_suite_20261006.json']:
 for row in json.loads((out/fn).read_text(encoding='utf8')):
  previous[row['case'].replace('colbar_repeat','colbar')]=row['result']
model=root/'sim/models/model_b.psm'
with model.open('rb') as f: hdr=PsmHeader.unpack(f.read(64))
inventory=[]
colbar_payload=None
for name,rawheader in headers.items():
 stem=name.rsplit('_f',1)[0];kind=stem.removeprefix('image_test_')
 h=packer.parse_header(rawheader)
 if name.endswith('_f000.psf'):
  matches=[]
  for source in data.rglob(stem+'.mat'):
   image=loadmat(str(source),variable_names=['image'])['image']
   crop,_=packer.center_crop(image,1920)
   codes,_=packer.quantize_codes(crop,h['scale_num'],h['scale_den'],4095)
   if zlib.crc32(codes.astype('<u2').tobytes())&0xffffffff==h['payload_crc32']:
    matches.append(source);raw=codes;break
  assert matches,('No exact source match',name)
  payload=raw.astype('<u2').tobytes()
  (out/name).write_bytes(rawheader+payload)
  if kind=='colbar':colbar_payload=payload
  info=previous[kind]
  with model.open('rb') as f:
   f.seek(64+info['selected_id']*hdr.record_bytes)
   rec=_unpack_record(hdr,f.read(hdr.record_bytes))
  k=hdr.rank_default
  # Stored factors use Fortran pixel indices; reorder to row-major kernel indices.
  U=rec.U[:k].reshape(k,hdr.meas_w,hdr.meas_h).transpose(0,2,1).reshape(k,-1).T.astype(float)*rec.u_scale
  V=rec.V[:k].reshape(k,hdr.hidden_w,hdr.hidden_h).transpose(0,2,1).reshape(k,-1).T.astype(float)*rec.v_scale
  mult=rec.s_scale*rec.s_max*2.**(info['image_scale_exp']-info['measurement_exp'])
  sexp=int(np.ceil(np.log2(float(rec.S[:k].max())*mult/65535)))
  S=np.rint(rec.S[:k]*mult*2.**(-sexp))*2.**sexp
  norm=np.sqrt(np.abs(U.T@U).sum(axis=1).max())*np.sqrt(np.abs(V.T@V).sum(axis=1).max())*S.max()
  tau=np.floor(.9/(norm**2/2+2)*65536)/65536
  raw_float=crop.astype(float)*h['scale_num']/h['scale_den']
  savemat(str(out/(kind+'_matlab_input.mat')),dict(raw=raw.astype('uint16'),raw_float=raw_float,U=U,V=V,S=S,tau=tau,sigma=.25,lam=66/65536,iterations=64,measurement_exp=info['measurement_exp']),do_compression=True)
  inventory.append(dict(frame=name,source=str(matches[0]),payload_crc32=h['payload_crc32'],selected_id=info['selected_id'],tau=float(tau)))
  print('PREPARED',kind,'CRC_MATCH',h['payload_crc32'],flush=True)
  if len(sys.argv)>1 and sys.argv[1]=='--host':
   subprocess.run([sys.argv[2],str(model),str(out/name),str(out/(kind+'_host'))],check=True)
(out/'matlab_input_inventory.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf8')

for name,header in headers.items():
 if name.startswith('image_test_colbar_'):
  assert colbar_payload is not None
  (out/name).write_bytes(header+colbar_payload)
