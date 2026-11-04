"""Atomic batch checkpoints, single-writer locks and verified result snapshots."""
from pathlib import Path
import hashlib,json,os,uuid,time
from contracts import InputError,require,pinned,read_json,sha

def file_hash(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()

def atomic_bytes(path,data):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
 try:
  with temporary.open('xb') as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
  # Windows readers can briefly deny delete-sharing; retry a bounded interval.
  # The previous complete checkpoint stays in place until replace succeeds.
  deadline=time.monotonic()+2
  while True:
   try:os.replace(str(temporary),str(path));break
   except PermissionError as error:
    if os.name!='nt' or getattr(error,'winerror',None) not in (5,32,33) or time.monotonic()>=deadline:raise
    time.sleep(.01)
 finally:
  if temporary.exists():temporary.unlink()

def atomic_json(path,value):atomic_bytes(path,(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8'))

class WriterLock:
 def __init__(self,root):self.path=Path(root)/'.batch.lock';self.stream=None
 def __enter__(self):
  self.stream=self.path.open('a+b');self.stream.seek(0,2)
  if not self.stream.tell():self.stream.write(b'0');self.stream.flush()
  self.stream.seek(0)
  try:
   if os.name=='nt':
    import msvcrt
    msvcrt.locking(self.stream.fileno(),msvcrt.LK_NBLCK,1)
   else:
    import fcntl
    fcntl.flock(self.stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
  except OSError:
   self.stream.close();self.stream=None;raise InputError('BATCH_BUSY','Another writer owns this batch')
  return self
 def __exit__(self,*args):
  if self.stream is not None:
   if os.name=='nt':
    import msvcrt
    self.stream.seek(0);msvcrt.locking(self.stream.fileno(),msvcrt.LK_UNLCK,1)
   else:
    import fcntl
    fcntl.flock(self.stream.fileno(),fcntl.LOCK_UN)
   self.stream.close()

def safe_child(root,relative):
 root=Path(root).resolve();path=(root/relative).resolve()
 require(path!=root and root in path.parents,'RESULT_PATH','Result must stay inside its batch')
 return path

def seal_result(output,descriptor):
 output=Path(output);summary=read_json(output/'summary.json')
 require(summary['status']=='COMPLETE' and summary.get('state_acceptance') is True,'BATCH_NUMERIC','Complete state acceptance required')
 require(not descriptor.get('golden') or summary.get('reference_acceptance') is True,'BATCH_REFERENCE','Registered reference acceptance required')
 require(summary['model_sha256']==descriptor['model']['sha256'] and summary['model_id']==descriptor['model_id'],'BATCH_MODEL','Result model identity mismatch')
 if descriptor['kind']=='PFY':require(summary['measurement_sha256']==descriptor['file']['sha256'],'BATCH_INPUT','Result measurement mismatch')
 files=summary['output_files']
 require(all(name in files for name in ('result.pfo','result.pfs','reconstruction_native.png','reconstruction_physical.npy','state_acceptance.json')),
         'BATCH_ARTIFACT','Incomplete numerical artifacts')
 for name,expected in files.items():require(file_hash(safe_child(output,name))==expected,'BATCH_ARTIFACT','Artifact changed: '+name)
 return dict(summary_sha256=file_hash(output/'summary.json'),files=dict(files),measurement_sha256=summary['measurement_sha256'],
             model_sha256=summary['model_sha256'],iterations=summary.get('iterations'),seconds_solver=summary.get('seconds_solver'),
             numeric_scope=summary['numeric_scope'])

def verify_seal(root,row):
 folder=safe_child(root,row['output']);seal=row['seal']
 require(file_hash(folder/'summary.json')==seal['summary_sha256'],'CHECKPOINT_OUTPUT','Result summary changed')
 for name,expected in seal['files'].items():require(file_hash(safe_child(folder,name))==expected,'CHECKPOINT_OUTPUT','Result artifact changed: '+name)
 return folder

def verify_inputs(descriptor,base):
 # Resume is not permission to reuse results after a mutable source has changed.
 for key in ('file','model','golden','calibration','metadata'):
  if descriptor.get(key):pinned(base,descriptor[key])

def append_event(path,row):
 with Path(path).open('a',encoding='utf8',newline='\n') as stream:
  stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
