"""Bounded, content-pinned immutable resource snapshots for one solver owner.

A cache hit deliberately consumes the already verified bytes, never mutable
source-file contents. Changing a model requires its new hash/ID or a new cache.
No measurement, calibration validation or solver result is cached here.
"""
from collections import OrderedDict
from contracts import pinned,blob,MODEL_BYTES,require


class VerifiedResources:
    def __init__(self,max_bytes=64*1024*1024,max_entries=16):
        require(type(max_bytes) is int and 0<=max_bytes<=256*1024*1024 and
                type(max_entries) is int and 1<=max_entries<=64,'CACHE_BUDGET','Invalid resource budget')
        self.max_bytes=max_bytes;self.max_entries=max_entries;self.entries=OrderedDict()
        self.bytes=0;self.peak_bytes=0;self.hits=0;self.misses=0;self.evictions=0;self.source_bytes_read=0

    def get(self,base,record,model_id=None):
        require(type(record['bytes']) is int and 0<record['bytes']<=128*1024*1024 and
                isinstance(record['sha256'],str) and len(record['sha256'])==64 and
                all(c in '0123456789abcdef' for c in record['sha256']), 'CACHE_ID','Pinned content identity required')
        key=(record['sha256'],record['bytes'],model_id)
        if key in self.entries:
            self.hits+=1;self.entries.move_to_end(key);return self.entries[key]
        data=pinned(base,record)
        if model_id is not None:blob(data,1,(29,36),MODEL_BYTES,model_id)
        self.misses+=1;self.source_bytes_read+=len(data)
        if len(data)<=self.max_bytes:
            while self.entries and (self.bytes+len(data)>self.max_bytes or len(self.entries)>=self.max_entries):
                _,old=self.entries.popitem(last=False);self.bytes-=len(old);self.evictions+=1
            self.entries[key]=data;self.bytes+=len(data);self.peak_bytes=max(self.peak_bytes,self.bytes)
        return data

    def model(self,descriptor,base):return self.get(base,descriptor['model'],descriptor['model_id']),descriptor['model_id']
    def release(self,root,manifest,name):return self.get(root,dict(manifest['files'][name],path=name))
    def status(self):
        return dict(bytes=self.bytes,peak_bytes=self.peak_bytes,budget_bytes=self.max_bytes,entries=len(self.entries),
                    hits=self.hits,misses=self.misses,evictions=self.evictions,source_bytes_read=self.source_bytes_read,
                    semantics='VERIFIED_IMMUTABLE_CONTENT_SNAPSHOT',measurement_results_cached=False)
    def clear(self):self.entries.clear();self.bytes=0
