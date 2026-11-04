from pathlib import Path
import argparse,json,os,subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1];project=root/'project'
p=argparse.ArgumentParser();p.add_argument('--case',choices=['D11_rgb','Chair_mushroom'],default='D11_rgb');p.add_argument('--output',required=True);a=p.parse_args()
config=project/'configs'/('windows_cpu.json' if os.name=='nt' else 'linux_cpu.json')
v=json.loads(config.read_text('utf8'));v['ascii_runtime_root']=str(Path(tempfile.gettempdir())/'ps_submission_solver')
with tempfile.TemporaryDirectory(prefix='ps_config_') as d:
 c=Path(d)/'config.json';c.write_text(json.dumps(v),encoding='utf8')
 subprocess.run([sys.executable,'-B',str(project/'repo/src/software/full_model/offline_pipeline/run.py'),'--config',str(c),'--workspace',str(project),'--case',a.case,'--output',str(Path(a.output).resolve())],check=True)
