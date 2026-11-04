"""Read-only package validation; never imports the board launcher."""
import argparse,json,sys
from pathlib import Path
from dataset_contract import validate,READY

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path,nargs='?',default=Path(__file__).resolve().parents[1]);a=p.parse_args()
    m,lock=validate(a.dataset)
    print(json.dumps(dict(passed=True,cases=len(m['cases']),ready=sum(r['reference_status'] in READY for r in m['cases']),
                          paper_status_counts=m['paper_status_counts'],reference_failed=[r['key'] for r in m['cases'] if r['reference_status'] not in READY],board_access=False),indent=2))
    return 0
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('REJECTED:',e,file=sys.stderr);sys.exit(2)
