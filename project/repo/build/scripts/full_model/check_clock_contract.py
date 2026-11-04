"""Join generated PS init, BD clock/width, HLS clock and launcher constants."""
from pathlib import Path
import json,re,sys,hashlib
sys.dont_write_bytecode=True
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
import hardware_profile as hw
def main():
    clock=json.loads((root/'reports/hardware_clock.json').read_text())
    mhz=clock['actual_mhz'];name='p56_'+str(int(mhz))
    hls=json.loads((root/'reports/contract_audit.json').read_text())
    if hls['axi_burst']==128 and mhz==150:name+='_b128'
    assert hls['axi_outstanding']==8
    assert (name=='p56_150_b128') or hls['axi_burst']==64
    p=hw.profile_for(dict(release_id=hw.TOKEN,hardware_release=hw.TOKEN,hardware_profile=name))
    init=root/'build/vivado_final/full_board.gen/sources_1/bd/full_board/ip/full_board_ps_0/psu_init.c'
    s=init.read_text();writes=re.findall(r'PSU_Mask_Write\(CRL_APB_PL0_REF_CTRL_OFFSET,\s*(0x[0-9A-Fa-f]+)U,\s*(0x[0-9A-Fa-f]+)U\)',s)
    assert len(writes)==1,writes
    mask,value=(int(v,16) for v in writes[0]);assert mask==hw.PL0_MASK and value==hw.expected_pl0(p)
    assert re.search(r'PSU_Mask_Write\(CRL_APB_IOPLL_CTRL_OFFSET,\s*0x00717F00U,\s*0x00015A00U\)',s)
    bd=root/'build/vivado_final/full_board.srcs/sources_1/bd/full_board/full_board.bd'
    design=json.loads(bd.read_text());params=design['design']['components']['ps']['parameters']
    assert params['PSU__SAXIGP2__DATA_WIDTH']['value']=='128'
    assert params['PSU__MAXIGP0__DATA_WIDTH']['value']=='32'
    assert params['PSU__CRL_APB__PL0_REF_CTRL__SRCSEL']['value']=='IOPLL'
    assert abs(float(params['PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ']['value'])-mhz)<0.001
    source=(root/'scripts/build_bd.tcl').read_text()
    assert '[get_bd_pins ps/pl_clk0] [get_bd_pins reset/slowest_sync_clk] [get_bd_pins core/ap_clk] [get_bd_pins control/aclk] [get_bd_pins memory/aclk]' in source
    assert 'foreach p [get_bd_pins -quiet ps/*aclk] {connect_bd_net [get_bd_pins ps/pl_clk0] $p}' in source
    hls=json.loads((root/'reports/contract_audit.json').read_text())
    assert abs(hls['clock_ns']-1000/mhz)<0.01
    report=dict(passed=True,physical_board_test=False,profile=name,actual_mhz=mhz,pl0_mask=hex(mask),pl0_value=hex(value),
        parent='IOPLL 1500 MHz nominal; runtime PLL/lock/frac readback required',hp0_width=128,mat_width=hls['axi_data_widths']['MAT'],vec_width=64,
        axi_burst=hls['axi_burst'],axi_outstanding=hls['axi_outstanding'],
        paths=['core/m_axi_mat -> SmartConnect -> HP0','core/m_axi_vec -> same SmartConnect -> same HP0'],shared_clock='PS PL0',
        ideal_hp0_bytes_per_second=16*mhz*1e6,full_matrix_bytes=8719488,ideal_matrix_ms=8719488/(16*mhz*1e3),
        measured_board_bandwidth=None,source_hashes={str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [init,bd,root/'scripts/hardware_profile.py']})
    (root/'reports/clock_contract.json').write_text(json.dumps(report,indent=2))
    print('CLOCK_CONTRACT_PASS',name,hex(value))
if __name__=='__main__':main()
