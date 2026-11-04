from pathlib import Path
import subprocess,zlib,base64,json,hashlib
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
assets=ROOT/'assets';assets.mkdir(exist_ok=True)
labels={'title':('PenumbraSight  实时采集与重建',36,(12,18,30),(240,246,255)),
 'left':('相机实时画面',29,(12,18,30),(224,235,248)),
 'right':('FPGA 重建图像',29,(12,18,30),(224,235,248)),
 'waiting':('未重建',38,(38,52,72),(178,197,216)),
 'processing':('正在重建…',28,(38,52,72),(255,211,105)),
 'failed':('重建失败',28,(38,52,72),(255,134,134)),
 'complete':('重建完成',28,(38,52,72),(116,231,170)),
 'nonconverged':('未收敛 · 仅诊断图',26,(38,52,72),(255,179,104)),
 'diagnostic':('未标定诊断 · D11 模型与现场不匹配',22,(12,18,30),(255,202,111))}
for key,(label,size,bg,fg) in labels.items():
 font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',size);box=font.getbbox(label)
 im=Image.new('RGB',(box[2]+8,size+18),bg);ImageDraw.Draw(im).text((3,-box[1]+4),label,font=font,fill=fg);im.save(assets/(key+'.png'))
helper=ROOT/'serial_command.ps1'
if not helper.exists():helper.write_bytes(Path('D:/Temp/ps_fb03_20261025/serial_command_baud.ps1').read_bytes())
def send(src,name):
 p=ROOT/(name+'.py');p.write_text(src,encoding='utf-8')
 r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(helper),'-PortName','COM14','-Script',str(p),'-Log',str(ROOT/(name+'.log')),'-TimeoutSeconds','25'],capture_output=True,text=True)
 if r.returncode:raise RuntimeError(r.stdout+r.stderr)
 return r.stdout
if __name__=='__main__':
 for p in [ROOT/'dual_runtime.py',ROOT/'pl_worker.py',ROOT/'start_board.py']+list(assets.glob('*.png')):
  enc=base64.b64encode(zlib.compress(p.read_bytes())).decode();remote='/home/root/ps_dualview_20261030/'+str(p.relative_to(ROOT)).replace('\\','/')
  for i in range(0,len(enc),1500):
   send("from pathlib import Path\np=Path(%r);p.parent.mkdir(exist_ok=True,parents=True)\nwith p.open(%r) as f:f.write(%r)\n"%(remote+'.b64','w' if i==0 else 'a',enc[i:i+1500]),'upload')
  print(send("from pathlib import Path\nimport base64,zlib,hashlib\np=Path(%r);d=zlib.decompress(base64.b64decode(Path(str(p)+'.b64').read_text()));assert hashlib.sha256(d).hexdigest()==%r\np.write_bytes(d)\nprint('VERIFIED',str(p),len(d))"%(remote,hashlib.sha256(p.read_bytes()).hexdigest()),'upload_verify'),flush=True)
