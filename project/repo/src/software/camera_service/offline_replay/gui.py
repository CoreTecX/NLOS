"""Tk display adapter. The RAW loader and file writer remain in core.py."""
from pathlib import Path
import os
import sys
import time

from PIL import Image, ImageTk


def make_root():
    # Vivado's bundled Python has Tcl/Tk under base_prefix/tcl, outside its default
    # search path. This changes only this process and respects explicit user paths.
    for key, folder, marker in [('TCL_LIBRARY','tcl8.6','init.tcl'), ('TK_LIBRARY','tk8.6','tk.tcl')]:
        path = Path(sys.base_prefix) / 'tcl' / folder
        if (path / marker).is_file():
            os.environ.setdefault(key, str(path))
    import tkinter as tk
    return tk.Tk()


class ReplayWindow:
    def __init__(self, root, session, interval_ms=200, limit=0, stop_file=None):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.session = root, session
        self.interval_ms, self.limit = interval_ms, limit
        self.stop_file = Path(stop_file) if stop_file else session.recorder.output / 'STOP'
        self.timer, self.running, self.closed, self.cycle = None, False, False, 0
        self.photo = None
        self.display_key = None
        root.title('PenumbraSight — 离线回放 REPLAY')
        root.geometry('960x780')
        root.minsize(760, 650)
        root.protocol('WM_DELETE_WINDOW', self.close)
        outer = ttk.Frame(root, padding=18); outer.pack(fill='both', expand=True)
        outer.columnconfigure(0,weight=1);outer.rowconfigure(4,weight=1)
        ttk.Label(outer, text='历史单帧回放 · REPLAY', font=('Microsoft YaHei UI', 19, 'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(outer, text='循环会重复同一张历史图像；这里显示的是相机预览。').grid(row=1,column=0,sticky='w',pady=(4,12))
        self.choice = tk.StringVar(value=session.frame.sample_id)
        self.selector = ttk.Combobox(outer, textvariable=self.choice, values=list(session.catalog.rows), state='readonly', width=38)
        self.selector.grid(row=2,column=0,sticky='w')
        self.selector.bind('<<ComboboxSelected>>', self.select)
        self.details = tk.StringVar()
        ttk.Label(outer, textvariable=self.details, wraplength=680).grid(row=3,column=0,sticky='w',pady=8)
        self.image = ttk.Label(outer, anchor='center')
        self.image.grid(row=4,column=0,sticky='nsew')
        self.image.bind('<Configure>', self._resize)
        self.status = tk.StringVar(value='就绪')
        ttk.Label(outer, textvariable=self.status, wraplength=680).grid(row=5,column=0,sticky='w',pady=8)
        buttons = ttk.Frame(outer); buttons.grid(row=6,column=0,sticky='ew')
        self.buttons = {}
        for label, command in [('单次显示',self.once),('循环播放',self.play),('停止',self.stop),('保存当前帧',self.save),('退出',self.close)]:
            button = ttk.Button(buttons, text=label, command=command)
            button.pack(side='left', padx=(0,8)); self.buttons[label] = button
        ttk.Label(outer, text='输出：'+str(session.recorder.output), wraplength=680).grid(row=7,column=0,sticky='w',pady=(12,0))
        root.bind('<Escape>', lambda event: self.stop())
        root.bind('<Control-s>', lambda event: self.save())
        self._details()

    def _details(self):
        if self.session.frame is None:
            self.details.set('输入不可用'); return
        identity = self.session.frame.identity; fmt = identity['format']; capture = identity['original_capture']
        self.details.set('%d×%d BayerRG12 | 曝光 %s µs | 原始帧号 %s | 回放间隔 %d ms\n原始相机 ticks：%s（时间频率未知）' %
                         (fmt['width'],fmt['height'],capture['exposure_us'],capture['frame_number'],self.interval_ms,capture['camera_timestamp_ticks']))

    def _paint(self):
        frame = self.session.frame
        available_width,available_height = self.image.winfo_width(),self.image.winfo_height()
        bounds=(min(850,available_width-4) if available_width>4 else 850,
                min(512,available_height-4) if available_height>4 else 512)
        key=(id(frame),bounds)
        if self.photo is not None and self.display_key==key:return
        image = Image.fromarray(frame.preview_rgb)
        image.thumbnail(bounds, getattr(Image, "Resampling", Image).LANCZOS)
        self.photo = ImageTk.PhotoImage(image, master=self.root)
        self.display_key = key
        self.image.configure(image=self.photo, text='')

    def _resize(self,event=None):
        if self.closed or self.photo is None or self.session.frame is None or self.session.state=='ERROR':return
        try:self._paint()
        except Exception as error:self._error(error)

    def _render(self):
        frame = self.session.frame
        self._paint()
        self.session.emit(displayed=True)
        self.status.set('REPLAY #%d · %s · 原始图像保持不变' % (self.session.index,frame.sample_id))
        self.buttons['保存当前帧'].configure(state='normal')

    def _error(self, error):
        self.stop()
        if self.session.state != 'ERROR': self.session.fail(error)
        self.image.configure(image='',text='输入或输出发生错误，已停止')
        self.photo = None
        self.buttons['保存当前帧'].configure(state='disabled')
        self.status.set('错误：'+str(error))

    def once(self):
        if self.closed: return
        self.stop()
        try:
            self._render()
            self.session.stop('single_frame_complete')
        except Exception as error:
            self._error(error)

    def play(self):
        if self.closed or self.running: return
        try:
            self.session.start()
            self.running, self.cycle = True, 0
            self.next_due = time.perf_counter()
            self._tick()
        except Exception as error:
            self._error(error)

    def _tick(self):
        self.timer = None
        if self.closed or not self.running: return
        try:
            if self.stop_file.exists():
                self.stop('stop_file'); return
            current = time.perf_counter()
            if current >= self.next_due:
                self._render(); self.cycle += 1
                if self.limit and self.cycle >= self.limit:
                    self.stop('frame_limit'); return
                # No queued stale frames or catch-up bursts after a slow display/save.
                self.next_due = time.perf_counter() + self.interval_ms / 1000
            delay = max(1,min(50,round((self.next_due-time.perf_counter())*1000)))
            self.timer = self.root.after(delay,self._tick)
        except Exception as error:
            self._error(error)

    def stop(self, reason='user_stop'):
        if self.closed: return
        self.running = False
        if self.timer is not None:
            self.root.after_cancel(self.timer); self.timer = None
        self.session.stop(reason)
        self.status.set('已停止 · REPLAY #%d' % self.session.index)

    def select(self, event=None):
        if self.closed: return
        self.stop('sample_change')
        self.image.configure(image='',text='正在校验样本…')
        self.photo = None
        self.buttons['保存当前帧'].configure(state='disabled')
        try:
            self.session.select(self.choice.get())
            self._details(); self.once()
        except Exception as error:
            self._error(error)

    def save(self):
        if self.closed: return
        try:
            target = self.session.save()
            self.status.set('已保存：'+str(target))
        except Exception as error:
            self._error(error)

    def close(self):
        if self.closed: return
        self.stop('window_close')
        self.session.close('window_close')
        self.closed = True
        self.root.destroy()
