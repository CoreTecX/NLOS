"""GVCP transport from verified board source; no implicit startup or camera image changes."""

from pathlib import Path
import socket,struct,time,threading,subprocess

HOST='192.168.1.10'

CAMERA='192.168.1.64'

REG={'width':0x30360,'height':0x303a0,'pixel_format':0x30610,'super_bayer':0x30614,
 'test_pattern':0x30680,'compression':0x306c8,'acquisition_mode':0x30800,'start':0x30804,'stop':0x30808,
 'frame_rate':0x3081c,'fps_unit':0x10238,'trigger_mode':0x30840,'exposure_us':0x30b04,
 'exposure_auto':0x30b08,'gain':0x31200,'gain_auto':0x31220,'payload':0x31900,
 'chunk':0x33c00,'heartbeat':0x938,'ccp':0xa00,'port':0xd00,'packet_size':0xd04,
 'packet_delay':0xd08,'destination':0xd18,'stream_config':0x960,'binning':0x304a0}

class Control:
 def __init__(self):
  self.s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.s.bind((HOST,0));self.s.settimeout(.35);self.rid=0x100;self.lock=threading.Lock()
 def request(self,cmd,payload):
  with self.lock:
   self.rid=(self.rid+1)%65535 or 1;rid=self.rid
   packet=struct.pack('!BBHHH',0x42,1,cmd,len(payload),rid)+payload
   for attempt in range(3):
    self.s.sendto(packet,(CAMERA,3956));end=time.monotonic()+.4
    while time.monotonic()<end:
     try:data,peer=self.s.recvfrom(4096)
     except socket.timeout:break
     if peer!=(CAMERA,3956) or len(data)<8:continue
     status,ack,n,seq=struct.unpack_from('!HHHH',data)
     if seq!=rid:continue
     if status or ack!=cmd+1 or len(data)!=8+n:raise RuntimeError('GVCP status=%04x ack=%04x cmd=%04x'%(status,ack,cmd))
     return data[8:]
   raise RuntimeError('GVCP timeout cmd=%04x'%cmd)
 def read(self,a):
  b=self.request(0x80,struct.pack('!I',a));assert len(b)==4
  return struct.unpack('!I',b)[0]
 def write(self,a,v):
  b=self.request(0x82,struct.pack('!II',a,v));assert len(b)==4,b.hex()
 def close(self):self.s.close()

def address_probe():
 mac=bytes.fromhex(Path('/sys/class/net/eth0/address').read_text().strip().replace(':',''))
 s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0806));s.bind(('eth0',0));s.settimeout(.1)
 p=b'\xff'*6+mac+struct.pack('!HHHBBH',0x0806,1,0x0800,6,4,1)+mac+b'\0'*4+b'\0'*6+socket.inet_aton(HOST)
 try:
  for _ in range(3):
   s.send(p);end=time.monotonic()+.4
   while time.monotonic()<end:
    try:f=s.recv(4096)
    except socket.timeout:continue
    if len(f)>=42 and f[6:12]!=mac and (f[28:32]==socket.inet_aton(HOST) or (f[28:32]==b'\0'*4 and f[38:42]==socket.inet_aton(HOST))):raise RuntimeError('Camera-side IPv4 conflict')
 finally:s.close()
