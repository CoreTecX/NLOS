"""Read-only FAT32 structural audit after cleanly unmounting an unused boot volume."""
from pathlib import Path
import array,hashlib,json,os,struct,subprocess,sys,time,traceback
S=Path('/home/root/ps_runs_20261018/v5_step02_01');E=S/'evidence/fat_audit';E.mkdir(exist_ok=False)
assert Path('/proc/sys/kernel/random/boot_id').read_text().strip()==json.loads((S/'session.json').read_text())['boot_id']
users=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 for item in [p/'cwd',p/'root']+list((p/'fd').glob('*')):
  try:
   target=os.readlink(str(item))
   if target=='/mnt/boot' or target.startswith('/mnt/boot/'):users.append(dict(pid=p.name,path=target))
  except OSError:pass
assert not users,users
mount_before=Path('/proc/mounts').read_text();assert '/dev/mmcblk1p1 /mnt/boot vfat rw,' in mount_before
os.sync();r=subprocess.run(['umount','/mnt/boot'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True);assert r.returncode==0,r.stdout
report=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),scope='read-only FAT32 structural audit; does not replace dosfstools fsck or validate every application payload',mount_before=mount_before,unmounted_cleanly=True,errors=[],warnings=[],files=[])
try:
 with open('/dev/mmcblk1p1','rb',buffering=0) as f:
  def read(off,n):
   f.seek(off);b=f.read(n);assert len(b)==n;return b
  b=read(0,512);(E/'boot_sector_after_unmount.bin').write_bytes(b)
  bps=struct.unpack_from('<H',b,11)[0];spc=b[13];reserved=struct.unpack_from('<H',b,14)[0];nf=b[16];total=struct.unpack_from('<I',b,32)[0];fsz=struct.unpack_from('<I',b,36)[0];root=struct.unpack_from('<I',b,44)[0];fi,backup=struct.unpack_from('<HH',b,48);assert b[510:512]==b'\x55\xaa' and bps==512 and spc and b[82:90]==b'FAT32   '
  cluster_size=bps*spc;data_sector=reserved+nf*fsz;count=(total-data_sector)//spc;fatraw=[read((reserved+i*fsz)*bps,fsz*bps) for i in range(nf)];fat=array.array('I');fat.frombytes(fatraw[0]);assert sys.byteorder=='little'
  if any(x!=fatraw[0] for x in fatraw[1:]):report['errors'].append('FAT copies differ')
  report['header']=dict(bytes_per_sector=bps,sectors_per_cluster=spc,total_clusters=count,root_cluster=root,fat_copies=nf,fat_sha256=[hashlib.sha256(x).hexdigest() for x in fatraw],dirty_flag=b[65],fat_entry1=hex(fat[1]))
  info=read(fi*bps,bps);(E/'fsinfo.bin').write_bytes(info);free_hint,next_hint=struct.unpack_from('<II',info,488)
  if info[:4]!=b'RRaA' or info[484:488]!=b'rrAa' or info[508:512]!=b'\0\0\x55\xaa':report['errors'].append('FSInfo signatures invalid')
  back=read(backup*bps,bps);(E/'backup_boot_sector.bin').write_bytes(back)
  if b!=back:report['warnings'].append('Primary/backup boot sectors differ (report byte offsets)');report['boot_differing_offsets']=[i for i in range(bps) if b[i]!=back[i]]
  owners={};visited_dirs=set()
  def chain(start,name):
   result=[];seen=set();c=start
   while c:
    if c<2 or c>=count+2:report['errors'].append(name+': out-of-range cluster '+str(c));break
    if c in seen:report['errors'].append(name+': chain cycle');break
    if c in owners:report['errors'].append(name+': crosslink with '+owners[c]);break
    seen.add(c);owners[c]=name;result.append(c);n=fat[c]&0x0fffffff
    if n>=0x0ffffff8:break
    if n==0 or n==0x0ffffff7 or n>=0x0ffffff0:report['errors'].append(name+': invalid chain link '+hex(n));break
    c=n
   return result
  def cluster(c):return read((data_sector+(c-2)*spc)*bps,cluster_size)
  def directory(c,path):
   if c in visited_dirs:report['errors'].append(path+': repeated directory');return
   visited_dirs.add(c);parts=chain(c,path+'/');data=b''.join(cluster(x) for x in parts)
   for off in range(0,len(data),32):
    x=data[off:off+32]
    if not x or x[0]==0:break
    if x[0]==0xe5 or x[11]==0x0f or x[11]&8:continue
    name=x[:8].decode('cp437').strip();ext=x[8:11].decode('cp437').strip()
    if name in ['.','..']:continue
    name=path+'/'+name+('.'+ext if ext else '');first=(struct.unpack_from('<H',x,20)[0]<<16)|struct.unpack_from('<H',x,26)[0];size=struct.unpack_from('<I',x,28)[0]
    if x[11]&0x10:
     if first<2:report['errors'].append(name+': invalid directory cluster')
     else:directory(first,name)
    else:
     parts=chain(first,name) if first else [];needed=(size+cluster_size-1)//cluster_size
     if len(parts)!=needed:report['errors'].append(name+': file size/chain mismatch')
     report['files'].append(dict(short_path=name,bytes=size,clusters=len(parts),first_cluster=first))
  directory(root,'')
  allocated={i for i in range(2,count+2) if fat[i]&0x0fffffff};bad={i for i in allocated if fat[i]&0x0fffffff==0x0ffffff7};lost=sorted(allocated-set(owners)-bad);free=count-len(allocated)
  if lost:report['errors'].append('Unreferenced allocated clusters: '+str(len(lost)))
  if bad:report['warnings'].append('Marked bad clusters: '+str(len(bad)))
  if free_hint!=0xffffffff and free_hint!=free:report['warnings'].append('FSInfo free-count mismatch')
  report.update(directory_count=len(visited_dirs),files_count=len(report['files']),allocated_clusters=len(allocated),referenced_clusters=len(owners),orphan_cluster_count=len(lost),orphan_sample=lost[:100],bad_cluster_count=len(bad),free_clusters=free,fsinfo_free_hint=free_hint,fsinfo_next_hint=next_hint,structural_pass=not report['errors'])
except BaseException as exc:report.update(structural_pass=False,exception=str(exc),traceback=traceback.format_exc())
finally:
 r=subprocess.run(['mount','-t','vfat','-o','ro','/dev/mmcblk1p1','/mnt/boot'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True);report['remount_readonly']=dict(exit_code=r.returncode,output=r.stdout);report['mount_after']=Path('/proc/mounts').read_text();report['uptime']=Path('/proc/uptime').read_text();(E/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
