"""Bounded nonblocking JSONL recorder. Overflow and disk failures remain visible."""
from pathlib import Path
import json,queue,threading,time


class BoundedLog:
    def __init__(self,path,capacity=1024,sink=None):
        self.path=Path(path);self.queue=queue.Queue(maxsize=capacity);self.sink=sink;self.closing=False
        self.total=0;self.dropped=0;self.failed=0;self.written=0;self.error=None;self.lock=threading.Lock()
        self.thread=threading.Thread(target=self._run,name='telemetry_writer',daemon=True);self.thread.start()
    def emit(self,row):
        line=json.dumps(dict(at_ns=str(time.monotonic_ns()),**row),ensure_ascii=False,allow_nan=False)+'\n'
        with self.lock:
            self.total+=1
            if self.closing or len(line)>262144:self.dropped+=1;return False
            try:self.queue.put_nowait(line);return True
            except queue.Full:self.dropped+=1;return False
    def _run(self):
        stream=None
        try:
            if self.sink is None:stream=self.path.open('x',encoding='utf8',newline='\n')
        except OSError as e:self.error=str(e)
        while not self.closing or not self.queue.empty():
            try:line=self.queue.get(timeout=.02)
            except queue.Empty:continue
            try:
                if self.error:raise OSError(self.error)
                if self.sink:self.sink(line)
                else:stream.write(line);stream.flush()
                self.written+=1
            except Exception as e:self.failed+=1;self.error=str(e)
            finally:self.queue.task_done()
        if stream:stream.close()
    def close(self,wait=False):
        with self.lock:self.closing=True
        if wait:self.thread.join()
    def status(self):
        return dict(total=self.total,written=self.written,dropped=self.dropped,failed=self.failed,error=self.error,
                    audit_complete=self.dropped==0 and self.failed==0 and self.error is None,
                    drained=self.closing and not self.thread.is_alive())
