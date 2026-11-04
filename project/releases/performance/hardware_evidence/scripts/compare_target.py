"""Evaluate downloaded board outputs against frozen numerical references."""
from pathlib import Path
import argparse,json,struct,zlib,math
ROOT=Path(__file__).resolve().parents[1]
def read(path,kind,mid):
    b=path.read_bytes()
    if len(b)!=64+1044*3*3*8 or b[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',b,8)!=(1,kind,29,36):raise ValueError('Invalid output header '+str(path))
    if struct.unpack_from('<Q',b,24)[0]!=len(b)-64 or b[36:40]!=b'\0'*4 or b[56:64]!=b'\0'*8:raise ValueError('Invalid output length/reserved fields '+str(path))
    if b[40:56].hex()!=mid or zlib.crc32(b[64:])!=struct.unpack_from('<I',b,32)[0]:raise ValueError('Output identity/CRC failed '+str(path))
    a=struct.unpack('<9396d',b[64:])
    if not all(math.isfinite(x) for x in a):raise ValueError('Nonfinite output')
    return a
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',type=Path);p.add_argument('--all',action='store_true');a=p.parse_args()
    rows=json.loads((ROOT/'data'/'manifest.json').read_text());results=[];missing=[]
    for r in rows:
        file=a.results/(r['key']+'.pfo')
        if not file.exists():missing.append(r['key']);continue
        actual=read(file,4,r['model_id']);gold=read(ROOT/'data'/r['golden']['file'],3,r['model_id'])
        rel=[]
        for c in range(3):
            aa=actual[c*3132:(c+1)*3132];gg=gold[c*3132:(c+1)*3132]
            rel.append(math.sqrt(sum((x-y)**2 for x,y in zip(aa,gg))/max(sum(y*y for y in gg),1e-300)))
        peak=max(abs(x-y) for x,y in zip(actual[3132:6264],gold[3132:6264]))/max(max(abs(x) for x in gold[3132:6264]),1e-30)
        m=json.loads(file.with_suffix('.json').read_text())
        residuals=[m[k] for k in ('primal','dual','primal_limit','dual_limit')]
        strict=(m['converged'] is True and type(m['iterations']) is int and 0<m['iterations']<=20000
                and all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in residuals)
                and m['primal_limit']>0 and m['dual_limit']>0
                and m['primal']<=m['primal_limit'] and m['dual']<=m['dual_limit'])
        passed=strict and rel[0]<1e-11 and max(rel[1:])<1e-6 and peak<1e-6
        results.append(dict(key=r['key'],passed=passed,relative_h=rel[0],relative_normalized=rel[1],relative_physical=rel[2],relative_peak_error=peak,iterations=m['iterations']))
    report=dict(passed=bool(results) and all(r['passed'] for r in results) and (not a.all or not missing),cases=results,missing=missing,
        note='Numerical agreement with the frozen reference; not calibrated physical accuracy or confidence')
    (a.results/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
