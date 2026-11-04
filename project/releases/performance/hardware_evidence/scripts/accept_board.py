"""Strict offline evaluation of downloaded CPU/PL board results; no hardware access."""
from pathlib import Path
import argparse,json,math,struct,zlib

ROOT=Path(__file__).resolve().parents[1]
N=1044*3

def read(path,kind,model_id):
    b=path.read_bytes()
    if len(b)!=64+3*N*8 or b[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',b,8)!=(1,kind,29,36):
        raise ValueError('Invalid result header: '+str(path))
    if struct.unpack_from('<Q',b,24)[0]!=len(b)-64 or b[36:40]!=bytes(4) or b[56:64]!=bytes(8):
        raise ValueError('Invalid payload length/reserved fields')
    if b[40:56].hex()!=model_id or zlib.crc32(b[64:])!=struct.unpack_from('<I',b,32)[0]:
        raise ValueError('Result identity/CRC mismatch')
    a=struct.unpack('<'+str(3*N)+'d',b[64:])
    if not all(math.isfinite(x) for x in a):raise ValueError('Nonfinite result')
    return a

def norm(a):return math.sqrt(math.fsum(x*x for x in a))

def evaluate(row,path):
    actual=read(path,4,row['model_id']);gold=read(ROOT/'data'/row['golden']['file'],3,row['model_id'])
    rel=[]
    for i in range(3):
        a=actual[i*N:(i+1)*N];g=gold[i*N:(i+1)*N]
        rel.append(norm([x-y for x,y in zip(a,g)])/max(norm(g),1e-30))
    peak=max(abs(x-y) for x,y in zip(actual[N:2*N],gold[N:2*N]))/max(max(abs(x) for x in gold[N:2*N]),1e-30)
    if not all(math.isfinite(x) for x in rel+[peak]):raise ValueError('Result exceeds numerical evaluation range')
    m=json.loads(path.with_suffix('.json').read_text('utf-8'))
    numbers=[m[k] for k in ('primal','dual','primal_limit','dual_limit','rho')]
    finite=all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in numbers)
    expected_primal=1e-7*max(1.,norm(actual[N:2*N]));expected_dual=1e-7*max(1.,norm(actual[:N]))
    last=m['trace'][-1] if isinstance(m.get('trace'),list) and m['trace'] else []
    trace_ok=last==[m['iterations'],m['primal'],m['dual'],m['rho'],m['primal_limit'],m['dual_limit']]
    strict=(m['converged'] is True and type(m['iterations']) is int and 0<m['iterations']<=20000 and finite
            and m['rho']>0 and m['primal_limit']>0 and m['dual_limit']>0
            and m['primal']<m['primal_limit'] and m['dual']<m['dual_limit']
            and math.isclose(m['primal_limit'],expected_primal,rel_tol=1e-12,abs_tol=0.)
            and math.isclose(m['dual_limit'],expected_dual,rel_tol=1e-12,abs_tol=0.) and trace_ok)
    passed=bool(strict and rel[0]<1e-11 and max(rel[1:])<1e-6 and peak<1e-6)
    return dict(key=row['key'],passed=passed,relative_h=rel[0],relative_normalized=rel[1],
                relative_physical=rel[2],relative_peak_error=peak,strict_residuals=bool(strict),
                iterations=m['iterations'] if type(m['iterations']) is int else None)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',type=Path);p.add_argument('--all',action='store_true')
    p.add_argument('--report',type=Path);a=p.parse_args();results=[];missing=[]
    rows=json.loads((ROOT/'data/manifest.json').read_text('utf-8'))
    for row in rows:
        path=a.results/(row['key']+'.pfo')
        if not path.exists():missing.append(row['key']);continue
        try:results.append(evaluate(row,path))
        except (ValueError,KeyError,TypeError,OverflowError,OSError) as e:results.append(dict(key=row['key'],passed=False,error=str(e)))
    report=dict(passed=bool(results) and all(r['passed'] for r in results) and (not a.all or not missing),
                cases=results,missing=missing,evidence='Numerical acceptance only; performance and hardware origin are not inferred')
    text=json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)
    if a.report:
        with a.report.open('x',encoding='utf-8') as f:f.write(text)
    print(text);return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
