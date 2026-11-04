"""Run one matched dataset case on CPU without camera or FPGA access."""
from pathlib import Path
import argparse,json,os,subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1];project=root/'project'
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--case',default='Chair_mushroom');p.add_argument('--output',required=True)
a=p.parse_args();index=json.loads((project/'data/batch_manifest.json').read_text('utf8'))
rows=[r for r in index['items'] if r['id']==a.case]
if len(rows)!=1:p.error('Unknown case; see project/data/batch_manifest.json')
if os.name!='nt' and __import__('platform').machine().lower() not in ('aarch64','arm64'):
 p.error('Included native binaries support Windows x64 and Linux AArch64')
config=project/'configs'/('windows_cpu.json' if os.name=='nt' else 'linux_cpu.json')
v=json.loads(config.read_text('utf8'));v['ascii_runtime_root']=str(Path(tempfile.gettempdir())/'ps_engineering_solver')
with tempfile.TemporaryDirectory(prefix='ps_config_') as d:
 c=Path(d)/'config.json';c.write_text(json.dumps(v),encoding='utf8')
 subprocess.run([sys.executable,'-B',str(project/'repo/src/software/full_model/offline_pipeline/run.py'),
 '--config',str(c),'--workspace',str(project),'--input',str(project/'data'/rows[0]['input']['path']),
 '--output',str(Path(a.output).resolve())],check=True)
