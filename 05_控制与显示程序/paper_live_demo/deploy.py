from pathlib import Path
import base64,gzip,subprocess,sys
p=Path(__file__).resolve().parent
names=['runtime.py','start_board.py','camera_transport.py','raw_contract.py']+[str(f.relative_to(p)).replace('\\','/') for f in (p/'assets').glob('*.png')]
files={n:base64.b64encode(gzip.compress((p/n).read_bytes())).decode() for n in names}
source="from pathlib import Path\nimport sys,gzip,base64\nr=Path('/home/root/ps_paper_live_20261103');r.mkdir(exist_ok=True)\nfor n,b in "+repr(files)+".items():\n f=r/n;f.parent.mkdir(parents=True,exist_ok=True);f.write_bytes(gzip.decompress(base64.b64decode(b)))\nsys.path.insert(0,str(r))\nfrom start_board import start\nprint(start())\n"
work=p/'runtime_output';work.mkdir(exist_ok=True);request=work/'deploy.py';request.write_text(source,encoding='utf8')
subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(p/'serial_console.ps1'),'-PortName',sys.argv[1] if len(sys.argv)>1 else 'COM14','-Script',str(request),'-Log',str(work/'deploy.log'),'-TimeoutSeconds','100'],check=True)
