"""Audit downloaded original cases against immutable references and raw solver states."""
from pathlib import Path
import argparse,importlib.util,json
RELEASE=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\P7_P11_Final_20261017\release\performance')

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    p=argparse.ArgumentParser();p.add_argument('results',type=Path);p.add_argument('--report',type=Path,required=True);p.add_argument('--case',action='append');a=p.parse_args()
    numeric=load('original_numeric',RELEASE/'scripts/accept_board.py');state=load('original_state',RELEASE/'scripts/accept_state.py')
    rows=json.loads((RELEASE/'data/manifest.json').read_text('utf8'))
    if a.case:
        rows=[r for r in rows if r['key'] in a.case]
        if len(rows)!=len(a.case):raise ValueError('Unknown or duplicated case selection')
    results=[]
    for r in rows:
        try:
            prefix=a.results/r['key'];n=numeric.evaluate(r,prefix.with_suffix('.pfo'))
            s=state.evaluate(RELEASE/'data'/r['model']['file'],RELEASE/'data'/r['measurement']['file'],prefix)
            row=dict(key=r['key'],passed=bool(n['passed'] and s['passed']),numeric=n,state=s)
        except Exception as e:row=dict(key=r['key'],passed=False,error=str(e))
        results.append(row);print(r['key'],row['passed'],flush=True)
    report=dict(passed=bool(results) and all(r['passed'] for r in results),cases=results,hardware_origin='Established separately by board job, boot identity, and verified transport')
    with a.report.open('x',encoding='utf8') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(dict(passed=report['passed'],cases=len(results))))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
