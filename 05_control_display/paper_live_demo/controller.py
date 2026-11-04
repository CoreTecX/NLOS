"""Manual reconstruction switch: live camera left, paper-data full solver right."""
from pathlib import Path
import os,sys,queue,threading
for name,folder in [('TCL_LIBRARY','tcl8.6'),('TK_LIBRARY','tk8.6')]:
    p=Path(sys.base_prefix)/'tcl'/folder
    if p.is_dir():os.environ[name]=str(p)
import tkinter as tk
from tkinter import ttk
from bridge import operate
class App:
    def __init__(self):
        self.root=tk.Tk();self.root.title('非视域智能感知预警系统 · 新版重建控制');self.root.geometry('900x720');self.root.minsize(860,680);self.root.configure(bg='#101827')
        self.results=queue.Queue();self.inflight=False;self.last_output=None;self.connected=False
        style=ttk.Style();style.theme_use('clam');style.configure('TButton',font=('Microsoft YaHei',12),padding=9)
        tk.Label(self.root,text='实时预览 · FPGA 重建',font=('Microsoft YaHei',22,'bold'),bg='#101827',fg='#edf5ff').pack(pady=(25,15))
        row=tk.Frame(self.root,bg='#101827');row.pack();tk.Label(row,text='串口：',bg='#101827',fg='white').pack(side='left')
        self.port=tk.StringVar(value='COM14');self.entry=ttk.Entry(row,textvariable=self.port,width=10);self.entry.pack(side='left',padx=8)
        self.info=tk.StringVar(value='正在连接并恢复双栏显示…')
        tk.Label(self.root,textvariable=self.info,wraplength=700,justify='left',font=('Microsoft YaHei',11),bg='#101827',fg='#bcd5eb').pack(pady=20)
        self.buttons=[]
        for text,action in [('执行重建','reconstruct'),('连接 / 恢复双栏页面','start'),('检查状态','status'),('清空右侧','reset')]:
            b=ttk.Button(self.root,text=text,command=lambda a=action:self.call(a));b.pack(fill='x',padx=105,pady=3);self.buttons.append(b)
        ttk.Button(self.root,text='打开本次重建结果',command=self.open_result).pack(pady=7)
        self.root.after(150,self.consume);self.call('start');self.root.mainloop()
    def open_result(self):
        if self.last_output:os.startfile(self.last_output)
    def call(self,action):
        if self.inflight:return
        if action=='reconstruct' and not self.connected:
            self.info.set('请先点击“连接 / 恢复双栏页面”，连接成功后再执行重建。');return
        self.inflight=True;self.entry.configure(state='disabled')
        for b in self.buttons:b.configure(state='disabled')
        self.info.set('正在重建' if action=='reconstruct' else '正在连接开发板…')
        port=self.port.get().strip()
        def work():
            try:self.results.put((operate(action,port),None))
            except Exception as e:self.results.put((None,str(e)))
        threading.Thread(target=work,daemon=True).start()
    def consume(self):
        try:
            while True:
                result,error=self.results.get_nowait();self.inflight=False;self.entry.configure(state='normal')
                for b in self.buttons:b.configure(state='normal')
                if error:
                    self.connected=False
                    self.info.set('连接或操作失败，请待开发板启动完成后点击“连接 / 恢复双栏页面”重试。\n'+error[-160:]);continue
                c=result['command'];camera=result['camera'];self.connected=camera.get('state')=='LIVE'
                text='相机实时预览：%.2f fps | 右侧状态：%s'%(camera.get('receive_fps',0),c['state'])
                if c.get('job_id'):text+='\n本次任务：'+c['job_id']
                if c['state']=='COMPLETE':
                    text+='\n完整算法：%s次迭代，纯求解%.3f秒；参考及状态核验通过。'%(c['iterations'],c['seconds_solver'])
                    text+='\n数据来源：论文 chair_mushroom；计算后端：电脑CPU；右侧与实时相机独立。'
                    self.last_output=result.get('pc_output') or c.get('pc_output')
                if c['state']=='ERROR':text+='\n'+c.get('error','查看任务日志')
                self.info.set(text)
        except queue.Empty:pass
        self.root.after(150,self.consume)
if __name__=='__main__':App()
