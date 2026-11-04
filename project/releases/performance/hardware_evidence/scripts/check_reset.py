"""Reproduce old held-reset wiring and validate both release/reload with generated reset IP."""
from pathlib import Path
import subprocess,shutil,json,hashlib,stat,os
root=Path(__file__).resolve().parents[1];out=root/'build/reset_test';out.mkdir(exist_ok=True)
source=root/'build/vivado_final/full_board.gen/sources_1/bd/full_board/ip/full_board_reset_0/sim/full_board_reset_0.vhd'
text=source.read_text();assert "C_EXT_RESET_HIGH => '0'" in text and "C_AUX_RESET_HIGH => '0'" in text
bd=root/'build/vivado_final/full_board.gen/sources_1/bd/full_board/synth/full_board.v'
b=bd.read_text();assert '.aux_reset_in(ps_pl_resetn0)' in b and '.ext_reset_in(ps_pl_resetn0)' in b
design=json.loads((root/'build/vivado_final/full_board.srcs/sources_1/bd/full_board/full_board.bd').read_text())
params=design['design']['components']['ps']['parameters']
settings={k:int(params[k]['value']) for k in ('PSU__MAXIGP0__DATA_WIDTH','PSU__SAXIGP2__DATA_WIDTH','PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ')}
assert list(settings.values())==[32,128,int(os.environ.get('PS_CLOCK_MHZ','100'))],settings
assert params['PSU__CRL_APB__PL0_REF_CTRL__SRCSEL']['value']=='IOPLL'
assert 'wire [127:0]core_m_axi_mat_RDATA;' in b
shutil.copy2(source,out/source.name);shutil.copy2(root/'tests/reset_tb.sv',out/'reset_tb.sv')
if (out/'glbl.v').exists():(out/'glbl.v').chmod((out/'glbl.v').stat().st_mode|stat.S_IWRITE)
shutil.copyfile(Path('D:/vivado/2025.2/Vivado/data/verilog/src/glbl.v'),out/'glbl.v')
(out/'reset.prj').write_text('vhdl xil_defaultlib "full_board_reset_0.vhd"\nsv xil_defaultlib "glbl.v"\nsv xil_defaultlib "reset_tb.sv"\n')
(out/'run.tcl').write_text('run all\nquit\n')
tool=Path('D:/vivado/2025.2/Vivado/bin');reports=root/'reports'
def run(cmd,log):
    with log.open('w') as f:p=subprocess.run([str(x) for x in cmd],cwd=str(out),stdout=f,stderr=subprocess.STDOUT)
    if p.returncode:raise RuntimeError('Reset tool failure '+str(log))
run([tool/'xelab.bat','xil_defaultlib.reset_tb','xil_defaultlib.glbl','-prj','reset.prj','-L','xil_defaultlib','-L','proc_sys_reset_v5_0_17',
     '-L','unisims_ver','-L','unisim','-L','xpm','-relax','-s','reset_test','-log',reports/'reset_elab.log'],reports/'reset_elab_console.log')
run([tool/'xsim.bat','reset_test','-tclbatch','run.tcl','--noieeewarnings','-log',reports/'reset_sim.log'],reports/'reset_sim_console.log')
assert 'RESET_INTEGRATION_PASS' in (reports/'reset_sim.log').read_text()
record=dict(passed=True,previous_wiring_holds_reset=True,corrected_release_and_reload=True,
    integrated_ext_and_aux_source='ps_pl_resetn0',ext_active_low=True,aux_active_low=True,
    generated_wrapper_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    integrated_bd_netlist_sha256=hashlib.sha256(bd.read_bytes()).hexdigest(),ps_settings=settings)
(reports/'reset_integration.json').write_text(json.dumps(record,indent=2));print('RESET_INTEGRATION_PASS')
