"""PC manual command switch. Uses the existing USB serial connection only."""
from pathlib import Path
import os,sys
# Vivado's bundled Python stores Tcl beside the executable, not under lib/.
for name,folder in [('TCL_LIBRARY','tcl8.6'),('TK_LIBRARY','tk8.6')]:
 candidate=Path(sys.base_prefix)/'tcl'/folder
 if candidate.is_dir():os.environ[name]=str(candidate)
import tkinter as tk
from tkinter import ttk,messagebox
import subprocess,threading,queue,json,uuid,re,time
ROOT=Path(__file__).resolve().parent
PYTHON=r'D:\psight\preboard_env\Scripts\pythonw.exe'
REMOTE='/home/root/ps_dualview_20261030'
class App:
 def __init__(self):
  self.root=tk.Tk();self.root.title('PenumbraSight 重建控制开关');self.root.geometry('680x440');self.root.configure(bg='#101827');self.root.protocol('WM_DELETE_WINDOW',self.close)
  self.results=queue.Queue();self.inflight=False;self.state=None;self.polling=False
  style=ttk.Style();style.theme_use('clam');style.configure('TButton',font=('Microsoft YaHei',13),padding=10)
  tk.Label(self.root,text='实时采集 · 手动重建',font=('Microsoft YaHei',23,'bold'),bg='#101827',fg='#edf5ff').pack(pady=(25,15))
  line=tk.Frame(self.root,bg='#101827');line.pack();tk.Label(line,text='串口：',bg='#101827',fg='white').pack(side='left')
  self.port=tk.StringVar(value='COM14');ttk.Entry(line,textvariable=self.port,width=10).pack(side='left',padx=8)
  self.info=tk.StringVar(value='正在自动连接开发板并启动相机预览，请稍候…')
  tk.Label(self.root,textvariable=self.info,wraplength=620,justify='left',font=('Microsoft YaHei',12),bg='#101827',fg='#bcd5eb').pack(pady=22)
  self.btn=ttk.Button(self.root,text='执行一次重建',command=lambda:self.call('reconstruct'),state='disabled');self.btn.pack(fill='x',padx=75)
  row=tk.Frame(self.root,bg='#101827');row.pack(pady=12)
  ttk.Button(row,text='连接 / 检查状态',command=lambda:self.call('status')).pack(side='left',padx=5)
  ttk.Button(row,text='恢复双栏页面',command=lambda:self.call('start')).pack(side='left',padx=5)
  ttk.Button(row,text='清空右侧',command=lambda:self.call('reset')).pack(side='left',padx=5)
  tk.Label(self.root,text='左侧持续预览；每次重建采集命令发出后的16帧新数据。\n实际使用板端 PL；现场模型未标定，结果用于诊断。\n关闭本窗口不会关机、停止风扇或停止相机预览。',font=('Microsoft YaHei',10),bg='#101827',fg='#efc47b').pack(pady=10)
  self.root.after(200,self.consume);self.call('start');self.root.mainloop()
 def close(self):self.root.destroy()
 def call(self,action):
  if self.inflight:return
  port=self.port.get().strip()
  if not re.fullmatch(r'COM\d+',port,re.I):messagebox.showerror('串口','请输入有效串口，例如COM14');return
  self.inflight=True;self.btn.configure(state='disabled');self.info.set('正在通信：'+{'status':'检查状态','start':'恢复双栏页面','reset':'清空右侧','reconstruct':'提交重建命令'}[action])
  def work():
   try:
    source="""from pathlib import Path
import json,time,os,uuid
r=Path(%r);p=Path((r/'current.txt').read_text().strip());s=json.loads((p/'state.json').read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert s['boot_id']==boot and s['state']=='LIVE','Preview not live; restore page first'
os.kill(s['pid'],0)
assert 0<=time.monotonic()-s['last_state_monotonic']<10,'Stale preview state'
c=json.loads((p/'command_state.json').read_text())
"""%REMOTE
    if action in ('reconstruct','reset'):
     source+="""assert c['state'] not in ('CAPTURING','RUNNING'),'Busy: wait for current reconstruction'
assert not (p/'request.json').exists(),'Command already queued'
req=dict(job_id=uuid.uuid4().hex,boot_id=boot,requested_monotonic=time.monotonic(),reset=%r)
t=p/'request.tmp';t.write_text(json.dumps(req));t.replace(p/'request.json')
c.update(state='QUEUED',job_id=req['job_id'])
"""%(action=='reset')
    if action=='start':source="from pathlib import Path\nimport time,json,subprocess\nr=Path(%r)\nf=open(str(r/'startup_ui.log'),'w')\nq=subprocess.Popen(['/usr/bin/python3',str(r/'start_board.py')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);f.close()\nfor _ in range(360):\n try:\n  p=Path((r/'current.txt').read_text().strip());v=json.loads((p/'state.json').read_text())\n  if v.get('state')=='LIVE' and v.get('complete_frames',0)>=2 and v.get('displayed_frames',0)>=1 and v.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and 0<=time.monotonic()-v.get('last_state_monotonic',0)<5:break\n except (OSError,ValueError):pass\n if q.poll() not in (None,0):raise RuntimeError((r/'startup_ui.log').read_text()[-1200:])\n time.sleep(.25)\nelse:raise RuntimeError('Camera startup timeout; inspect startup_ui.log')\n"%REMOTE+source
    source+="print('PS_CONTROL:'+json.dumps(dict(camera=s,command=c)))\n"
    ident=uuid.uuid4().hex;p=ROOT/('cmd_'+ident+'.py');log=ROOT/('cmd_'+ident+'.log');p.write_text(source,encoding='utf8')
    si=subprocess.STARTUPINFO();si.dwFlags|=subprocess.STARTF_USESHOWWINDOW
    result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'serial_command.ps1'),'-PortName',port,'-Script',str(p),'-Log',str(log),'-TimeoutSeconds','110'],capture_output=True,encoding='utf8',errors='replace',startupinfo=si,creationflags=subprocess.CREATE_NO_WINDOW)
    text=log.read_text('utf8') if log.exists() else result.stdout
    if result.returncode:
     if re.search(r'(?m)^[> ]*PS_REMOTE_EXCEPTION\r?$',text):
      if 'Preview not live' in text or 'Stale preview state' in text:raise RuntimeError('串口已连接，但相机预览已停止。请点击“恢复双栏页面”，恢复后再重建。日志：'+str(log))
      raise RuntimeError('开发板已响应，但程序执行失败。日志：'+str(log)+'\n'+text[-600:])
     raise RuntimeError('串口命令未完整返回，请检查COM号或其他串口程序。日志：'+str(log)+'\n'+text[-600:])
    match=re.search(r'PS_CONTROL:(\{[^\r\n]+\})',text)
    if not match:raise RuntimeError('未收到有效状态。日志：'+str(log))
    self.results.put((json.loads(match.group(1)),None))
   except Exception as e:self.results.put((None,str(e)))
  threading.Thread(target=work,daemon=True).start()
 def consume(self):
  try:
   while True:
    result,error=self.results.get_nowait();self.inflight=False
    if error:self.info.set(error);self.polling=False;continue
    c=result['command'];s=result['camera'];state=c['state'];self.state=state
    names={'WAITING':'未重建','QUEUED':'命令已提交','CAPTURING':'采集新图像 %s/16'%c.get('captured',0),'RUNNING':'FPGA 正在重建','COMPLETE':'重建完成','NONCONVERGED':'计算结束：未收敛，仅显示诊断图','ERROR':'重建失败'}
    text=names.get(state,state)+' | 相机 %.1f fps | 完整帧 %s'%(s.get('receive_fps',0),s['complete_frames'])
    if state in ('COMPLETE','NONCONVERGED'):text+='\n本次耗时 %.2f 秒；迭代 %s'%(c.get('seconds',0),c.get('iterations','?'))
    if c.get('input_quality',{}).get('saturated_fraction',0)>.01:text+='\n输入存在明显过曝，请改善曝光/照明后再评价重建质量。'
    if state=='ERROR':text+='\n'+str(c.get('error','查看板端任务日志'))[-260:]
    self.info.set(text);self.btn.configure(state='disabled' if state in ('CAPTURING','RUNNING','QUEUED') else 'normal')
    self.polling=state in ('CAPTURING','RUNNING','QUEUED')
    if self.polling:self.root.after(4000,lambda:self.call('status') if not self.inflight else None)
  except queue.Empty:pass
  self.root.after(200,self.consume)
if __name__=='__main__':App()
