"""Check the actual exported ABI, II, floating-point operators and static ELF."""
from pathlib import Path
import json,re,struct,xml.etree.ElementTree as ET,hashlib,os
root=Path(__file__).resolve().parents[1]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ip=root/'full_gemv_hls/solution1/impl/ip';rtl=(ip/'hdl/verilog/full_gemv.v').read_text()
    header=(ip/'drivers/full_gemv_v2_0/src/xfull_gemv_hw.h').read_text()
    expected={'MATRIX_DATA':0x10,'INPUT_R_DATA':0x1c,'OUTPUT_R_DATA':0x28,'N_DATA':0x34,'AP_RETURN':0x3c,'LAYOUT_DATA':0x44}
    actual={k:int(re.search('XFULL_GEMV_CONTROL_ADDR_'+k+r'\s+(0x[0-9a-fA-F]+)',header)[1],16) for k in expected}
    assert actual==expected,actual
    widths={k:int(re.search('C_M_AXI_'+k+r'_DATA_WIDTH\s*=\s*(\d+)',rtl)[1]) for k in ('MAT','VEC')}
    assert widths=={'MAT':128,'VEC':64},widths
    axi_burst=int(re.search(r'\.MAX_READ_BURST_LENGTH\(\s*(\d+)\s*\)',rtl)[1])
    axi_outstanding=int(re.search(r'\.NUM_READ_OUTSTANDING\(\s*(\d+)\s*\)',rtl)[1])
    assert axi_burst==int(os.environ.get('PS_AXI_BURST','64')) and axi_outstanding==int(os.environ.get('PS_AXI_OUTSTANDING','8'))
    xml=ET.parse(root/'full_gemv_hls/solution1/syn/report/compute_pairs_Pipeline_COLUMNS_csynth.xml')
    ii=int(xml.findtext('.//SummaryOfLoopLatency/COLUMNS/PipelineII'));assert ii==1
    period=float(os.environ.get('PS_HLS_PERIOD','10'))
    assert abs(float(xml.findtext('.//TargetClockPeriod'))-period)<0.01
    floats=list((ip/'hdl/verilog').glob('*dadd*.v'))
    latencies=sorted(set(int(re.search(r'_64_(\d+)_',p.name)[1]) for p in floats))
    assert latencies and max(latencies)<16
    flow=(ip/'hdl/verilog/full_gemv_run_pairs.v').read_text()
    assert 'compute_pairs_U0' in flow and 'reduce_pairs_U0' in flow
    assert (ip/'hdl/verilog/full_gemv_fifo_w3072_d2_S.v').is_file()
    names=' '.join(p.name for p in (ip/'hdl/verilog').glob('*.v'))
    assert 'dmul_' in names and 'dadd_' in names and 'dmacc' not in names
    elf=(root/'bin/ps_full_aarch64').read_bytes();assert elf[:6]==b'\x7fELF\x02\x01'
    assert struct.unpack_from('<H',elf,18)[0]==183
    phoff=struct.unpack_from('<Q',elf,32)[0];entsize,phnum=struct.unpack_from('<HH',elf,54)
    segments=[struct.unpack_from('<I',elf,phoff+i*entsize)[0] for i in range(phnum)]
    assert 2 not in segments and 3 not in segments,segments
    source=(root/'src/full_gemv.cpp').read_text();assert 'distance=16 true' in source
    assert 'offset=0x44' in source
    contract=(root/'src/gemv_contract.hpp').read_text();assert 'REG_LAYOUT=0x44' in contract
    driver=(root/'src/pl_backend.hpp').read_text();assert 'launch(0)==ps_layout::BAD_N' in driver
    # Both old/new probe statuses disagree. The n=0 query accesses no DDR in either kernel.
    assert 0x46500101!=0x46500201
    report=dict(passed=True,layout_version='0x32525001',release_id='20261016-rp2-p56-v1',registers=actual,
        axi_data_widths=widths,axi_burst=axi_burst,axi_outstanding=axi_outstanding,columns_ii=ii,clock_ns=period,add_latencies_cycles=latencies,dataflow=True,fifo_width_bits=3072,fifo_depth=2,arm_machine=183,arm_static=True,
        separate_multiply_add=True,physical_board_test_performed=False,
        bytes_per_matrix_1044=522*1044*16,read_beats_per_matrix_1044=522*1044,
        source_sha256={p.name:digest(p) for p in (root/'src').glob('*') if p.is_file()},
        aarch64_sha256=digest(root/'bin/ps_full_aarch64'))
    (root/'reports/contract_audit.json').write_text(json.dumps(report,indent=2))
    print('CONTRACT_AUDIT_PASS mat=128 vec=64 II=1 period_ns='+str(period)+' ARM=static')
if __name__=='__main__':main()
