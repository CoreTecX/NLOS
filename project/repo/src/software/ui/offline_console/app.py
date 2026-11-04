"""PenumbraSight offline operation console; no hardware connection is opened."""
from bootstrap import REPO
from pathlib import Path
import argparse,io,json,os,sys,time,uuid
from PIL import Image,ImageTk
from contracts import read_json
from runtime import ConsoleEngine
from presentation import Presenter


def make_root():
    for key,folder,marker in [('TCL_LIBRARY','tcl8.6','init.tcl'),('TK_LIBRARY','tk8.6','tk.tcl')]:
        p=Path(sys.base_prefix)/'tcl'/folder
        if (p/marker).is_file():os.environ.setdefault(key,str(p))
    import tkinter as tk
    return tk.Tk()


class ConsoleWindow:
    SOURCES={'合成动态测量':'dynamic','D11完整算法验证（时间模拟）':'d11','已登记的合成文件序列':'manifest'}
    def __init__(self,root,workspace,output_root,engine_factory=ConsoleEngine):
        import tkinter as tk
        from tkinter import ttk,filedialog
        self.root=root;self.workspace=Path(workspace);self.output_root=Path(output_root);self.engine_factory=engine_factory
        self.engine=None;self.presenter=None;self.closing=False;self.closed=False;self.photos={};self.last_snapshot={};self.history_view=False
        self.display_log=[];self.last_ack='等待开始';self.rendered_keys={}
        root.title('PenumbraSight · 离线运行台');root.geometry('1180x840');root.minsize(980,740)
        root.protocol('WM_DELETE_WINDOW',self.close)
        style=ttk.Style(root);style.configure('Title.TLabel',font=('Microsoft YaHei UI',20,'bold'))
        style.configure('Status.TLabel',font=('Microsoft YaHei UI',13,'bold'))
        outer=ttk.Frame(root,padding=18);outer.pack(fill='both',expand=True);outer.columnconfigure(0,weight=1);outer.rowconfigure(4,weight=1)
        ttk.Label(outer,text='PenumbraSight  离线运行台',style='Title.TLabel').grid(row=0,column=0,sticky='w')
        ttk.Label(outer,text='文件与合成数据 · 快速感知和完整重建 · 当前未连接硬件').grid(row=1,column=0,sticky='w',pady=(2,12))
        sourcebar=ttk.Frame(outer);sourcebar.grid(row=2,column=0,sticky='ew');sourcebar.columnconfigure(1,weight=1)
        self.source_choice=tk.StringVar(value='合成动态测量')
        self.source_box=ttk.Combobox(sourcebar,textvariable=self.source_choice,values=list(self.SOURCES),state='readonly',width=30);self.source_box.grid(row=0,column=0,padx=(0,8))
        self.manifest=tk.StringVar();self.manifest_entry=ttk.Entry(sourcebar,textvariable=self.manifest);self.manifest_entry.grid(row=0,column=1,sticky='ew')
        self.browse=ttk.Button(sourcebar,text='选择序列',command=lambda:self.manifest.set(filedialog.askopenfilename(filetypes=[('JSON序列','*.json')]) or self.manifest.get()))
        self.browse.grid(row=0,column=2,padx=8)
        self.buttons={};self.buttons['开始']=ttk.Button(sourcebar,text='开始',command=self.start);self.buttons['开始'].grid(row=0,column=3)
        controls=ttk.Frame(outer);controls.grid(row=3,column=0,sticky='ew',pady=12)
        for label,callback in [('快速感知',lambda:self.command('MODE',strategy='FAST')),('完整成像',lambda:self.command('MODE',strategy='IMAGE')),
                               ('自动切换',lambda:self.command('AUTO')),('重建当前测量',lambda:self.command('IMAGE')),
                               ('重新定位',lambda:self.command('LOCALIZE')),
                               ('保存快照',lambda:self.command('SAVE')),('停止',self.stop)]:
            button=ttk.Button(controls,text=label,command=callback);button.pack(side='left',padx=(0,6));self.buttons[label]=button
        ttk.Label(controls,text='成像帧数 K').pack(side='left',padx=(10,4));self.k=tk.StringVar(value='1')
        self.k_box=ttk.Combobox(controls,textvariable=self.k,values=['1','2','4','8','自动'],state='readonly',width=5);self.k_box.pack(side='left')
        self.k_box.bind('<<ComboboxSelected>>',lambda event:self.command('K',value='AUTO' if self.k.get()=='自动' else int(self.k.get())))
        panels=ttk.Frame(outer);panels.grid(row=4,column=0,sticky='nsew');panels.columnconfigure((0,1),weight=1);panels.rowconfigure(1,weight=1)
        ttk.Label(panels,text='墙面测量 · 显示预览',style='Status.TLabel').grid(row=0,column=0,sticky='w',pady=6)
        ttk.Label(panels,text='完整算法 · 彩色重建',style='Status.TLabel').grid(row=0,column=1,sticky='w',pady=6)
        self.preview_widget=ttk.Label(panels,text='等待输入',anchor='center',relief='sunken');self.preview_widget.grid(row=1,column=0,sticky='nsew',padx=(0,8))
        self.image_widget=ttk.Label(panels,text='点击“重建当前测量”生成图像',anchor='center',relief='sunken');self.image_widget.grid(row=1,column=1,sticky='nsew',padx=(8,0))
        self.preview_caption=tk.StringVar(value='输入独立显示；不以重建图代替输入')
        self.image_caption=tk.StringVar(value='尚无重建结果')
        ttk.Label(panels,textvariable=self.preview_caption,wraplength=510).grid(row=2,column=0,sticky='w',pady=8)
        ttk.Label(panels,textvariable=self.image_caption,wraplength=510).grid(row=2,column=1,sticky='w',pady=8)
        self.history_button=ttk.Button(panels,text='查看最近历史结果',command=self.show_history);self.history_button.grid(row=3,column=1,sticky='e')
        status=ttk.LabelFrame(outer,text='快速状态与质量',padding=10);status.grid(row=5,column=0,sticky='ew',pady=8)
        self.fast_label=tk.StringVar(value='目标：未知    方向：未知');ttk.Label(status,textvariable=self.fast_label,style='Status.TLabel').pack(anchor='w')
        self.quality_label=tk.StringVar(value='质量：等待输入');ttk.Label(status,textvariable=self.quality_label,wraplength=1050).pack(anchor='w',pady=4)
        self.runtime_label=tk.StringVar(value='就绪');ttk.Label(outer,textvariable=self.runtime_label,wraplength=1080).grid(row=6,column=0,sticky='w',pady=4)
        self.feedback=tk.StringVar(value='等待开始');ttk.Label(outer,textvariable=self.feedback,wraplength=1080).grid(row=7,column=0,sticky='w')
        self.output_label=tk.StringVar(value='输出根目录：'+str(self.output_root));ttk.Label(outer,textvariable=self.output_label,wraplength=1080).grid(row=8,column=0,sticky='w',pady=(8,0))
        root.bind('<Control-s>',lambda e:self.command('SAVE'));root.bind('<Escape>',lambda e:self.stop())
        self._enabled(False);self.timer=root.after(50,self.tick)
    def _enabled(self,running):
        for name,button in self.buttons.items():button.configure(state='normal' if (not running if name=='开始' else running) else 'disabled')
        self.source_box.configure(state='disabled' if running else 'readonly');self.browse.configure(state='disabled' if running else 'normal')
        self.k_box.configure(state='readonly' if running else 'disabled')
    def start(self):
        if self.engine and not self.engine.finished.is_set():return
        mode=self.SOURCES[self.source_choice.get()]
        if mode=='manifest' and not self.manifest.get():self.feedback.set('请先选择已登记的合成序列 JSON。');return
        output=self.output_root/('run_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.engine=self.engine_factory(self.workspace,output,mode,self.manifest.get() or None)
        self.presenter=Presenter(self.engine.run_id);self.history_view=False;self.rendered_keys={};self.photos={}
        self.preview_widget.configure(image='',text='正在校验输入…');self.image_widget.configure(image='',text='等待完整重建')
        self.image_caption.set('尚无重建结果');self.k.set('1');self._enabled(True)
        self.output_label.set('本次输出：'+str(output));self.engine.start();self.feedback.set('正在准备输入和完整模型…')
    def command(self,kind,**args):
        if not self.engine:return
        ack=self.engine.submit(kind,**args);self.feedback.set('命令已排队' if ack['outcome']=='QUEUED' else '当前忙碌，请稍后重试：'+ack['outcome'])
    def stop(self):
        if self.engine and not self.engine.finished.is_set():
            self.engine.submit('STOP');self.feedback.set('正在停止新任务，等待当前重建与保存完成…')
    def close(self):
        if self.closed:return
        self.closing=True;self.stop()
        if self.engine is None or self.engine.finished.is_set():self._destroy()
    def _destroy(self):
        if self.closed:return
        self.closed=True
        if self.timer:self.root.after_cancel(self.timer)
        self.root.destroy()
    def _paint(self,kind,image,key):
        if self.rendered_keys.get(kind)==key:return
        image=image.copy();image.thumbnail((500,350),getattr(Image, "Resampling", Image).NEAREST if kind=='image' else getattr(Image, "Resampling", Image).BILINEAR)
        photo=ImageTk.PhotoImage(image,master=self.root);self.photos[kind]=photo;self.rendered_keys[kind]=key
        (self.image_widget if kind=='image' else self.preview_widget).configure(image=photo,text='')
    def show_history(self):
        if not self.presenter or not self.presenter.history or not self.presenter.history.get('png'):
            self.feedback.set('暂无可查看的历史重建图像');return
        row=self.presenter.history;self.history_view=True
        self._paint('image',Image.open(io.BytesIO(row['png'])),'history:'+row['result']['request_id'])
        self.image_caption.set('历史结果（不代表当前场景） · '+row['result']['freshness'])
    def tick(self):
        self.timer=None
        if self.closed:return
        if self.engine and self.presenter:
            slots,events=self.engine.mailbox.take()
            submitted_fast=None
            for kind in ('state','preview','fast','image'):
                if kind not in slots:continue
                row=slots[kind];accepted=self.presenter.apply(kind,row)
                if accepted and kind=='preview':
                    self._paint('preview',Image.frombytes('RGB',(126,126),row['rgb']),row['frame_uid'])
                    self.preview_caption.set('来源：合成/回放测量 · 序号 '+row['frame_uid'].split(':')[-1])
                elif accepted and kind=='image' and row.get('png'):
                    self.history_view=False;self._paint('image',Image.open(io.BytesIO(row['png'])),row['result']['request_id'])
                if accepted and kind=='fast':
                    submitted_fast=row
            view=self.presenter.view();self.last_snapshot=view;fast=view['fast'];state=view['state']
            names={'present':'存在','absent':'不存在','unknown':'未知','left':'向左','right':'向右'}
            self.fast_label.set('目标：'+names.get(fast.get('presence'),'未知')+'    方向：'+names.get(fast.get('direction'),'未知'))
            q=fast.get('quality',{});b=fast.get('burst',{});qn={'USABLE':'信号充足','LOW_SIGNAL':'信号偏弱','UNOBSERVABLE':'无可辨信号','INVALID':'输入失效','UNKNOWN':'未知'}
            if not fast.get('valid'):quality='未知（输入失效、过期或尚未恢复）'
            else:quality=qn.get(q.get('quality'),'未知')
            snr=q.get('snr_db');text=f'质量：{quality}'+(f' · SNR估计 {snr:.1f} dB' if snr is not None else '')
            text+=f" · FAST K=1 · 成像 K={b.get('effective_k') or '等待'} / 请求 {b.get('requested_k',state.get('requested_k',1))}"
            if b.get('wait_ms') is not None:text+=f" · 观测跨度 {b['wait_ms']:.0f} ms"
            self.quality_label.set(text)
            phase=state.get('phase','INITIALIZING');strategy={'FAST':'快速感知','IMAGE':'完整成像'}.get(state.get('strategy'),'准备中')
            self.runtime_label.set(f"{phase} · {strategy} · {'自动' if state.get('control')=='AUTO' else '手动'} · 已处理 {state.get('processed',0)} 帧 · 图像任务 {state.get('image_state','IDLE')}"+
                (f" · 保存中 {state.get('save',{}).get('active',0)}" if state.get('save',{}).get('active') else ''))
            if view['image'] and not self.history_view:
                im=view['image'];labels={'CURRENT':'当前快照','HISTORICAL_AGE':'历史快照','SCENE_CHANGED':'场景已变化的快照','STALE_CONTEXT':'旧上下文快照'}
                self.image_caption.set(f"{labels.get(im['label'],im['label'])} · {im['age_ms']/1000:.1f}秒前 · K={im['k']} · 数值验收通过\n实景图像质量尚未验收")
            for e in events:
                if e.get('run_id')!=self.engine.run_id:continue
                if e.get('kind')=='SAVE_RESULT':self.feedback.set('已保存：'+e['path'] if e['outcome']=='SAVED' else '保存失败：'+e.get('error',''))
                elif e.get('kind')=='COMMAND_ACK':
                    outcomes={'APPLIED':'设置已应用','RUNNING':'完整重建已开始','QUEUED':'请求已排队','BUSY':'队列已满，请稍后再试',
                        'WAITING_BURST':'正在等待足够的有效帧；快速感知继续','INVALID_CONTEXT':'测量尚未恢复，暂不能重建','EXPIRED_MEASUREMENT':'当前测量已过期','NO_FRAME':'尚无可用帧',
                        'NO_MODEL_BANK':'当前输入未登记可用的定位模型库','LOCALIZATION_RESULT':'定位复核已完成，详情写入运行记录'}
                    self.feedback.set(outcomes.get(e['outcome'],e['outcome']))
                elif e.get('kind')=='ENGINE_ERROR':self.feedback.set('运行失败：'+e['error'])
                elif e.get('kind')=='IMAGE_RESULT' and not e['result']['publish_allowed']:
                    self.feedback.set('重建结果已记录；当前上下文不匹配，可单独查看历史结果' if e['result']['valid_numeric'] else '完整重建未通过数值检查，请查看运行记录')
            if state.get('telemetry',{}).get('audit_complete') is False:self.feedback.set('日志写入失败或溢出，本次审计记录不完整')
            if submitted_fast is not None:
                self.root.update_idletasks()
                row=submitted_fast;self.engine.record_display('fast',row['frame_uid'],row['frame_complete_ns'],row['event'].get('transition','NONE'))
                self.display_log.append(dict(frame_uid=row['frame_uid'],submitted_ns=str(time.monotonic_ns()),transition=row['event'].get('transition')))
                if len(self.display_log)>128:self.display_log.pop(0)
            if self.engine.finished.is_set():
                self._enabled(False)
                if self.closing:self._destroy();return
        self.timer=self.root.after(50,self.tick)


def main():
    config=read_json(REPO/'configs/offline_engineering.json');p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,default=Path(config['workspace_hint']))
    p.add_argument('--output-root',type=Path,default=Path('D:/Temp/penumbrasight_console'))
    args=p.parse_args();root=make_root();ConsoleWindow(root,args.workspace,args.output_root);root.mainloop()

if __name__=='__main__':main()
