"""Match HLS-exported RTL to the exact IP sources imported by Vivado."""
from pathlib import Path
import argparse,hashlib,json,re

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build',type=Path,default=Path(__file__).resolve().parents[1])
    ap.add_argument('--output',type=Path)
    args=ap.parse_args();stage=args.build.resolve()
    ip=stage/'full_gemv_hls/solution1/impl/ip/hdl/verilog'
    generated=stage/'build/vivado_final/full_board.gen/sources_1/bd/full_board/ipshared'
    rows=[]
    for src in sorted(ip.glob('*.v*')):
        matches=list(generated.glob('*/hdl/verilog/'+src.name))
        if len(matches)!=1:raise RuntimeError('Missing or ambiguous implemented RTL: '+src.name)
        dst=matches[0]
        rows.append(dict(file=src.name,sha256=sha(src),matches=sha(src)==sha(dst),implemented_copy=dst.relative_to(stage).as_posix()))
    if not rows or not all(r['matches'] for r in rows):raise RuntimeError('Implemented RTL differs from HLS export')
    rtl='\n'.join((ip/n).read_text('utf-8') for n in ['full_gemv.v','full_gemv_mat_m_axi.v','full_gemv_vec_m_axi.v'])
    locks=re.findall(r'assign\s+(?:[A-Za-z0-9_]*[Aa][Rr][Ll][Oo][Cc][Kk]|[A-Za-z0-9_]*[Aa][Ww][Ll][Oo][Cc][Kk])\s*=\s*([^;]+)',rtl)
    if not locks or not all(v.strip() in ["2'b00","1'b0","2'd0","0"] for v in locks):
        raise RuntimeError('Ignored AXI3 upper lock bits are not constant zero')
    result=dict(passed=True,files=rows,ignored_upper_lock_bits_constant_zero=True,lock_assignments=locks,
                kernel_source_sha256=sha(stage/'src/full_gemv.cpp'),hls_tcl_sha256=sha(stage/'scripts/hls.tcl'),bd_tcl_sha256=sha(stage/'scripts/build_bd.tcl'))
    output=args.output or stage/'reports/physical_source_pairing.json'
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('PHYSICAL_SOURCE_PAIRING_PASS',len(rows))

if __name__=='__main__':main()
