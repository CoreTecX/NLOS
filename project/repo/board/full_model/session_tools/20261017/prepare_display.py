"""Build an RGB565 screen from a verified board reconstruction."""
from pathlib import Path
import hashlib,json,struct,zlib
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
out=ROOT/'display';out.mkdir(exist_ok=True)
prefix=ROOT/'downloads/pl_overlap_d11_01/content/results/pl_overlap_d11_01/D11_rgb'
accept=json.loads((ROOT/'pl_overlap_d11_acceptance.json').read_text('utf8'))
if accept['passed'] is not True:raise ValueError('Board reconstruction not accepted')
data=prefix.with_suffix('.pfo').read_bytes();arrays=np.frombuffer(data[64:],dtype='<f8').reshape(3,29,36,3)
physical=arrays[2];v=np.clip(physical/physical.max(),0,1)
v=np.where(v<=.0031308,12.92*v,1.055*v**(1/2.4)-.055)
picture=Image.fromarray(np.rint(v*255).astype(np.uint8),'RGB')
canvas=Image.new('RGB',(1024,768),'#101a2b');d=ImageDraw.Draw(canvas)
fontpath=Path('C:/Windows/Fonts/msyh.ttc')
def font(size):return ImageFont.truetype(str(fontpath),size)
d.rectangle((0,0,1024,96),fill='#1c2c43')
d.text((30,20),'PenumbraSight · 最新算法实板重建',font=font(29),fill='#f2f6fb')
d.text((32,67),'开发板真实输出  |  独立数值与原始状态验收通过',font=font(17),fill='#8fd8c7')
canvas.paste(picture.resize((648,522),Image.Resampling.NEAREST),(28,131))
lines=[('当前案例',18,'#9daec5'),('D11_rgb',27,'#ffffff'),('',14,'#ffffff'),('FPGA 重叠调度',21,'#ffffff'),('150 MHz / burst 128',16,'#b2c3da'),('',12,'#ffffff'),('求解耗时',18,'#9daec5'),('28.86 秒',29,'#80dfc8'),('2900 次迭代 · 已收敛',17,'#b2c3da'),('',12,'#ffffff'),('完整 1044 维 FP64',18,'#ffffff'),('原始输出：36 × 29 RGB',16,'#b2c3da'),('显示：sRGB / 最近邻放大',16,'#b2c3da')]
y=139
for text,size,color in lines:d.text((708,y),text,font=font(size),fill=color);y+=size+12
d.line((28,681,994,681),fill='#344861',width=1)
d.text((30,700),'97 例批量重建正在后台运行；当前展示为已验收结果。',font=font(20),fill='#f2f6fb')
d.text((30,736),'原始 FP64 数据与完整日志已保存。放大显示不增加重建分辨率。',font=font(15),fill='#9daec5')
canvas.save(out/'screen_D11_rgb.png')
pixels=np.asarray(canvas,dtype=np.uint16);packed=((pixels[:,:,0]>>3)<<11)|((pixels[:,:,1]>>2)<<5)|(pixels[:,:,2]>>3)
raw=packed.astype('<u2').tobytes();compressed=zlib.compress(raw,9)
(out/'screen_D11_rgb.rgb565.z').write_bytes(compressed)
manifest=dict(width=1024,height=768,bits_per_pixel=16,red=[11,5,0],green=[5,6,0],blue=[0,5,0],raw_bytes=len(raw),raw_sha256=hashlib.sha256(raw).hexdigest(),compressed_bytes=len(compressed),compressed_sha256=hashlib.sha256(compressed).hexdigest(),case='D11_rgb',source='/home/root/ps_runs_20261017/steps5_12_01/results/pl_overlap_d11_01/D11_rgb.pfo',source_sha256=hashlib.sha256(data).hexdigest(),display='Physical RGB / shared max / sRGB / nearest neighbor',acceptance_passed=True)
(out/'screen_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps(manifest,indent=2))
