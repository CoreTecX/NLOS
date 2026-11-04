"""One running plus one waiting immutable snapshot, with explicit save receipts."""
from dataclasses import dataclass
from pathlib import Path
import hashlib,json,queue,threading,uuid,time


@dataclass(frozen=True)
class SaveSnapshot:
    metadata_json: str
    files: tuple
    def __post_init__(self):
        json.loads(self.metadata_json)
        names=[n for n,_ in self.files]
        if len(names)!=len(set(names)) or any(Path(n).name!=n or '/' in n or '\\' in n or n.startswith('.') for n in names):
            raise ValueError('Snapshot names must be distinct plain file names')
        if any(not isinstance(b,bytes) for _,b in self.files) or sum(len(b) for _,b in self.files)>16*1024*1024:
            raise ValueError('Immutable snapshot exceeds the registered 16 MiB budget')


def write_snapshot(root,request_id,snapshot):
    folder=Path(root)/('save_'+request_id);folder.mkdir(parents=True,exist_ok=False)
    hashes={}
    for name,data in snapshot.files:
        (folder/name).write_bytes(data);hashes[name]=dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    if 'preview.rgb' in hashes:
        from PIL import Image
        pixels=dict(snapshot.files)['preview.rgb'];shape=json.loads(snapshot.metadata_json)['preview_shape']
        Image.frombytes('RGB',(shape[1],shape[0]),pixels).save(folder/'preview.png')
        data=(folder/'preview.png').read_bytes();hashes['preview.png']=dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    record=dict(request_id=request_id,status='SAVED',metadata=json.loads(snapshot.metadata_json),files=hashes)
    (folder/'snapshot.json').write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf8')
    return str(folder)


class SnapshotSaver:
    def __init__(self,root,callback=lambda row:None,writer=write_snapshot):
        self.root=Path(root);self.callback=callback;self.writer=writer
        self.queue=queue.Queue(maxsize=2);self.slots=threading.BoundedSemaphore(2);self.lock=threading.Lock()
        self.closing=False;self.active=0;self.max_active=0;self.accepted=0;self.busy=0;self.failed=0;self.saved=0
        self.thread=threading.Thread(target=self._run,name='snapshot_writer',daemon=True);self.thread.start()
    def submit(self,snapshot):
        with self.lock:
            if self.closing:return dict(outcome='CLOSED')
            if not self.slots.acquire(blocking=False):self.busy+=1;return dict(outcome='BUSY')
            request_id=uuid.uuid4().hex;self.active+=1;self.max_active=max(self.max_active,self.active);self.accepted+=1
            self.queue.put_nowait((request_id,snapshot,time.monotonic_ns()))
            return dict(outcome='QUEUED',request_id=request_id)
    def _run(self):
        while not self.closing or not self.queue.empty():
            try:rid,snapshot,queued_ns=self.queue.get(timeout=.02)
            except queue.Empty:continue
            started_ns=time.monotonic_ns()
            try:
                folder=self.writer(self.root,rid,snapshot);self.saved+=1
                row=dict(kind='SAVE_RESULT',outcome='SAVED',request_id=rid,path=folder)
            except Exception as error:
                self.failed+=1;row=dict(kind='SAVE_RESULT',outcome='FAILED',request_id=rid,error=str(error))
            finally:
                with self.lock:self.active-=1
                self.slots.release();self.queue.task_done()
            row.update(queue_wait_ms=(started_ns-queued_ns)/1e6,write_ms=(time.monotonic_ns()-started_ns)/1e6,
                       total_ms=(time.monotonic_ns()-queued_ns)/1e6)
            self.callback(row)
    def close(self,wait=False):
        with self.lock:self.closing=True
        if wait:self.thread.join()
    @property
    def drained(self):return self.closing and not self.thread.is_alive()
    def status(self):
        with self.lock:return dict(active=self.active,max_active=self.max_active,accepted=self.accepted,busy=self.busy,failed=self.failed,saved=self.saved,drained=self.drained)
