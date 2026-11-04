"""Check that the simulation DUT and floating-point models are the exported IP."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1];out=root/'build'/'rtl_lite'
ip=root/'full_gemv_hls'/'solution1'/'impl'/'ip';reports=root/'reports'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def check(sources,target,report_name,flag):
    rows=[]
    for src in sources:
        copy=target/src.name
        rows.append(dict(file=src.name,**{flag:copy.exists() and digest(src)==digest(copy)},sha256=digest(src)))
    result=dict(passed=bool(rows) and all(r[flag] for r in rows),files=rows)
    (reports/report_name).write_text(json.dumps(result,indent=2),encoding='utf8')
    if not result['passed']:raise RuntimeError('RTL source pairing failed: '+report_name)
    return len(rows)

n=check(sorted((ip/'hdl'/'verilog').glob('*.v*')),out,'rtl_source_pairing.json','equal_to_synthesized_ip')
f=check(sorted((ip/'subcore_prj'/'subcore_prj.gen'/'sources_1'/'ip').glob('*/sim/*.vhd')),out/'ip'/'xil_defaultlib','rtl_float_model_pairing.json','equal_to_exported_ip')
if n==0 or f==0:raise RuntimeError('Missing DUT or floating-point sources')
if digest(root/'tests'/'axi_memory.sv')!=digest(out/'axi_memory.sv'):raise RuntimeError('AXI testbench source mismatch')
provenance={p.relative_to(out).as_posix():digest(p) for p in sorted(out.rglob('*')) if p.is_file() and p.suffix in ('.v','.vh','.vhd','.sv','.hex','.prj','.tcl') and 'xsim.dir' not in p.parts}
(reports/'rtl_lite_sources.json').write_text(json.dumps(provenance,indent=2),encoding='utf8')
print('RTL_SOURCE_PAIRING_PASS',n,'DUT files;',f,'vendor float models')
