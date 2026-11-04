"""Show every committed batch reconstruction in order, including after restart."""
from bootstrap import REPO
from pathlib import Path
import argparse,io,json,sys,time
sys.path.insert(0,str(REPO/'src/software/full_model/offline_pipeline'))
from batch_store import verify_seal,atomic_json,read_json
from PIL import Image,ImageTk
from app import make_root

class BatchViewer:
 def __init__(self,root,batch,receipt_directory,minimum_dwell_ms=200):
  from tkinter import ttk
  self.root=root;self.batch=Path(batch);self.receipts=Path(receipt_directory);self.receipts.mkdir(parents=True,exist_ok=True)
  self.cursor_path=self.receipts/'viewer_checkpoint.json';self.seen=set();self.submitted=0;self.errors=0;self.closed=False;self.photo=None;self.timer=None
  self.last_displayed=None;self.restored_history=False
  if self.cursor_path.exists():
   record=read_json(self.cursor_path)
   if record['batch']!=str(self.batch.resolve()):raise ValueError('Viewer checkpoint belongs to another batch')
   self.seen=set(record['seen']);self.submitted=len(self.seen)
  self.minimum_dwell_ms=minimum_dwell_ms;root.title('PenumbraSight · 完整重建结果')
  self.caption=ttk.Label(root,text='等待通过验收的完整重建',padding=12);self.caption.pack()
  self.widget=ttk.Label(root,text='尚无图像',padding=12);self.widget.pack()
  self.status=ttk.Label(root,text='历史数据回放 · 不代表相机实时画面',padding=12);self.status.pack()
  root.protocol('WM_DELETE_WINDOW',self.close);self.timer=root.after(50,self.tick)
 def paint(self,row):
  folder=verify_seal(self.batch,row);raw=(folder/'reconstruction_native.png').read_bytes()
  from contracts import sha
  if sha(raw)!=row['seal']['files']['reconstruction_native.png']:raise ValueError('Image changed during display')
  picture=Image.open(io.BytesIO(raw));picture.load();picture=picture.resize((576,464),getattr(Image,'Resampling',Image).NEAREST)
  photo=ImageTk.PhotoImage(picture,master=self.root);self.widget.configure(image=photo,text='');self.photo=photo
  self.caption.configure(text='完整重建 · '+row['id']+' · 第 '+str(row['index']+1)+' 项')
  self.status.configure(text='历史数据 · 数值验收通过 · 非实时相机画面');self.root.update_idletasks();self.last_displayed=row['id']
  return dict(id=row['id'],attempt=row['attempts'],input_measurement_sha256=row['seal']['measurement_sha256'],
              image_sha256=sha(raw),submitted_mono_ns=str(time.monotonic_ns()),kind='HISTORICAL_COMPLETE_RECONSTRUCTION',
              actual_tk_photo_created=True,physical_display_latency_measured=False)
 def tick(self):
  self.timer=None
  if self.closed:return
  try:
   checkpoint=read_json(self.batch/'checkpoint.json')
   if self.photo is None and self.seen:
    previous=[r for r in checkpoint['items'] if r['status']=='COMPLETE' and r['id']+':'+str(r['attempts']) in self.seen]
    if previous:
     self.paint(previous[-1]);self.restored_history=True
     return
   for row in checkpoint['items']:
    key=row['id']+':'+str(row['attempts'])
    if row['status']!='COMPLETE' or key in self.seen:continue
    event=self.paint(row)
    atomic_json(self.receipts/('display_%06d_%03d.json'%(row['index'],row['attempts'])),event)
    seen=self.seen|{key};atomic_json(self.cursor_path,dict(batch=str(self.batch.resolve()),seen=sorted(seen),submitted=len(seen)))
    self.seen=seen;self.submitted=len(seen)
    break
  except FileNotFoundError:self.status.configure(text='等待批处理结果文件…')
  except Exception as error:
   self.errors+=1;self.status.configure(text='显示未完成，将重试：'+str(error))
   atomic_json(self.receipts/'last_error.json',dict(error=str(error),count=self.errors,physical_display_latency_measured=False))
  finally:
   if not self.closed:self.timer=self.root.after(self.minimum_dwell_ms,self.tick)
 def close(self):
  self.closed=True
  if self.timer:self.root.after_cancel(self.timer)
  self.root.destroy()

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True);p.add_argument('--receipts',type=Path,required=True)
 a=p.parse_args();root=make_root();BatchViewer(root,a.batch,a.receipts);root.mainloop()
