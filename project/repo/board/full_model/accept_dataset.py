"""Offline numerical and independent state acceptance of downloaded dataset results."""
import argparse,importlib.util,json,sys
from pathlib import Path
from dataset_contract import validate,validate_release,select_cases

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--release',type=Path,required=True)
    p.add_argument('--results',type=Path,required=True);p.add_argument('--case',action='append');p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    root=a.dataset.resolve();release=a.release.resolve();manifest,lock=validate(root);validate_release(release,lock)
    selected=select_cases(manifest,a.case,not a.case)
    numeric=load('dataset_numeric_accept',release/'scripts/accept_board.py');numeric.ROOT=root
    state=load('dataset_state_accept',release/'scripts/accept_state.py')
    results=[]
    for row in selected:
        try:
            # Existing evaluator prefixes ROOT/data to golden.file.
            patched=dict(row,golden=dict(row['golden'],file=str(Path(row['golden']['file']).relative_to('data'))))
            prefix=a.results/row['key'];n=numeric.evaluate(patched,prefix.with_suffix('.pfo'))
            s=state.evaluate(root/row['model']['file'],root/row['measurement']['file'],prefix)
            results.append(dict(key=row['key'],passed=bool(n['passed'] and s['passed']),numeric=n,state=s))
        except Exception as e:results.append(dict(key=row['key'],passed=False,error=str(e)))
    report=dict(passed=bool(results) and all(r['passed'] for r in results),cases=results,
                paper_blocked=len(manifest.get('blocked_paper',[])),hardware_origin='Not inferred from numerical acceptance')
    with a.report.open('x',encoding='utf8') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(dict(passed=report['passed'],cases=len(results),failed=[r['key'] for r in results if not r['passed']]),indent=2))
    return 0 if report['passed'] else 1
if __name__=='__main__':sys.exit(main())
