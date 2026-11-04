"""Mark displayed results accepted only after the independent host audit passes."""
import hashlib,json,subprocess,sys
from jtag_transfer import ROOT,ENV
report=json.loads((ROOT/'dataset97_acceptance.json').read_text('utf8'))
if report['passed'] is not True or len(report['cases'])!=97 or not all(r['passed'] for r in report['cases']):raise RuntimeError('Full independent audit not passed')
results=ROOT/'downloads/dataset97_pl_01/content/results/dataset97_pl_01'
accepted={r['key']:hashlib.sha256((results/(r['key']+'.pfo')).read_bytes()).hexdigest() for r in report['cases']}
p=ROOT/'display/accepted_cases.json';p.write_text(json.dumps(accepted,sort_keys=True),encoding='utf8')
subprocess.run([sys.executable,'-B',str(ROOT/'upload_serial_file.py'),str(p),'display/accepted_cases.json'],env=ENV,check=True)
print('DISPLAY_ACCEPTANCE_PUBLISHED',len(accepted),flush=True)
