import glob,io,zipfile,base64
b=io.BytesIO()
with zipfile.ZipFile(b,'w',zipfile.ZIP_DEFLATED) as z:
 for p in glob.glob('/home/root/ps_board/verified_*')+glob.glob('/home/root/ps_board/extended_validation.*')+glob.glob('/home/root/ps_board/validation_telemetry.json'):
  if p.endswith(('.json','.bin','.log')):
   info=zipfile.ZipInfo(p.split('/')[-1],(2026,10,7,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED
   z.writestr(info,open(p,'rb').read())
s=base64.b64encode(b.getvalue()).decode()
print('EXPORT_BEGIN',flush=True)
for i in range(0,len(s),256):print(s[i:i+256],flush=True)
print('EXPORT_END',flush=True)
