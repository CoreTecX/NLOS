"""Supplemental bounded-queue/latency AXI RTL test; no board access."""
from pathlib import Path
import argparse,subprocess,os,re,json,hashlib,shutil,time,sys
sys.dont_write_bytecode=True
root=Path(__file__).resolve().parents[1]
tool=Path('D:/vivado/2025.2/Vivado/bin')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(args,cwd,log,env=None):
    with log.open('w') as f:p=subprocess.run([str(a) for a in args],cwd=cwd,stdout=f,stderr=subprocess.STDOUT,env=env)
    if p.returncode:raise RuntimeError('Tool failed: '+str(log))
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--latency',type=int,default=80);ap.add_argument('--queue',type=int,default=8)
    ap.add_argument('--hold',type=int,default=0);ap.add_argument('--tag',default='supply')
    a=ap.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_]+',a.tag):raise ValueError('Invalid tag')
    out=root/'build'/('rtl_'+a.tag);reports=root/'reports'/a.tag
    if out.exists():raise RuntimeError('Fresh supplemental stage required')
    reports.mkdir(parents=True)
    env=dict(os.environ,PS_RTL_OUTPUT=out.name)
    run([sys.executable,'-B','scripts/make_rtl_lite.py'],root,reports/'generate.log',env)
    shutil.copy2(root/'tests/axi_memory_queued.sv',out/'axi_memory_queued.sv')
    p=out/'rtl_lite.prj';s=p.read_text().replace('sv xil_defaultlib "axi_memory.sv"','sv xil_defaultlib "axi_memory_queued.sv"');p.write_text(s)
    p=out/'rtl_lite_tb.sv';s=p.read_text().replace('axi_memory #(','axi_memory_queued #(')
    start=s.index('  run_case(0,"",');end=s.index('  if((selected_case==0||selected_case==14)')
    s=s[:start]+r'''
  run_case(0,"",32'h46500201,32'h32525001);
  run_case(1044,"",32'h46500202,0);
  $readmemh("matrix_257.hex",mat_memory.mem,0,33152);
  // Poison the padding lane. It must never affect the 257 valid output rows.
  for(integer i=32896;i<33153;i=i+1)mat_memory.mem[i][127:64]=64'h40c34a0000000000;
  run_case(257,"expected_257.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_17.hex",mat_memory.mem,0,152);
  run_case(17,"expected_17.hex",32'h46500200,32'h32525001);
  $display("RTL_SUPPLY_METRICS max_pending=%0d bursts=%0d max_burst=%0d boundary_bursts=%0d read_backpressure=%0d fifo_full=%0d overlap=%0d stage_overlap=%0d ar_stalls=%0d w_stalls=%0d",
      mat_memory.max_pending,mat_memory.bursts,mat_memory.max_burst,mat_memory.boundary_bursts,
      mat_memory.r_backpressure,fifo_full_cycles,overlap_cycles,stage_overlap_cycles,mat_memory.ar_stalls,vec_memory.w_stalls);
  if(mat_memory.boundary_bursts==0 || stage_overlap_cycles==0)$fatal(1,"Supply coverage missing");
  if(vec_memory.write_hold>0 && (fifo_full_cycles==0||vec_memory.w_stalls<vec_memory.write_hold))$fatal(1,"Bounded FIFO backpressure not exercised");
  if(vec_memory.write_hold>0)begin
    begin: abort_buffered_job
      integer prior_full;
      // Reset both the master and modeled AXI domain; retain only DDR contents.
      @(negedge ap_clk);ap_rst_n=0;repeat(20)@(negedge ap_clk);ap_rst_n=1;repeat(20)@(negedge ap_clk);
      wr('h10,32'h40000000);wr('h14,0);wr('h1c,32'h41200000);wr('h20,0);wr('h28,32'h41300000);wr('h2c,0);
      $readmemh("matrix_257.hex",mat_memory.mem,0,33152);
      wr('h44,32'h32525001);wr('h34,257);prior_full=fifo_full_cycles;wr(0,1);
      wait(fifo_full_cycles>prior_full);
      if(vec_memory.hold_left==0)$fatal(1,"Reset did not interrupt blocked writeback");
      @(negedge ap_clk);ap_rst_n=0;repeat(20)@(negedge ap_clk);
      if(m_axi_mat_ARVALID || m_axi_vec_AWVALID)$fatal(1,"AXI request during buffered reset");
      ap_rst_n=1;repeat(20)@(negedge ap_clk);
      wr('h10,32'h40000000);wr('h14,0);wr('h1c,32'h41200000);wr('h20,0);wr('h28,32'h41300000);wr('h2c,0);
      $readmemh("matrix_17.hex",mat_memory.mem,0,152);
      run_case(17,"expected_17.hex",32'h46500200,32'h32525001);
      run_case(0,"",32'h46500201,32'h32525001);
      $display("RTL_FIFO_FULL_ABORT_RESET_PASS");
    end
  end
  $display("RTL_SUPPLY_PASS");$finish;
end
endmodule
'''
    p.write_text(s)
    run(['D:/CCC/mingw64/bin/g++.exe','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-Wno-unknown-pragmas','tests/rtl_vectors.cpp','src/reference_gemv.cpp','-o','bin/rtl_vectors.exe'],root,reports/'vectors_build.log')
    run([root/'bin/rtl_vectors.exe','data/D11_rgb.pfm','data/D11_rgb.pfy',out],root,reports/'vectors.log')
    ip=root/'full_gemv_hls/solution1/impl/ip'
    pairs=[]
    for src in sorted((ip/'hdl/verilog').glob('*.v*')):
        dst=out/src.name;pairs.append(dict(file=src.name,sha256=sha(src),matched=sha(src)==sha(dst)))
    for src in sorted((ip/'subcore_prj/subcore_prj.gen/sources_1/ip').glob('*/sim/*.vhd')):
        dst=out/'ip/xil_defaultlib'/src.name;pairs.append(dict(file=src.name,sha256=sha(src),matched=sha(src)==sha(dst)))
    if not pairs or not all(p['matched'] for p in pairs):raise RuntimeError('Vendor/DUT source mismatch')
    provenance={p.relative_to(out).as_posix():sha(p) for p in out.rglob('*') if p.is_file()}
    (reports/'sources.json').write_text(json.dumps(dict(pairing=pairs,files=provenance),indent=2))
    run([tool/'xelab.bat','xil_defaultlib.rtl_lite_tb','xil_defaultlib.glbl','-prj','rtl_lite.prj','-L','xil_defaultlib','-L','unisims_ver','-L','xpm','-L','floating_point_v7_1_21','-L','floating_point_v7_0_26','-relax','-s','rtl_supply','-log',reports/'elab.log'],out,reports/'elab_console.log')
    tic=time.monotonic()
    (out/'options.txt').write_text('rtl_supply\n-tclbatch run.tcl\n--noieeewarnings\n-testplusarg QUEUE='+str(a.queue)+'\n-testplusarg LATENCY='+str(a.latency)+'\n-testplusarg WRITE_HOLD='+str(a.hold)+'\n-log '+(reports/'sim.log').as_posix()+'\n')
    run([tool/'xsim.bat','-f','options.txt'],out,reports/'sim_console.log')
    log=(reports/'sim.log').read_text();passed='RTL_SUPPLY_PASS' in log and 'Fatal:' not in log
    if a.hold>0:passed=passed and 'RTL_FIFO_FULL_ABORT_RESET_PASS' in log
    result=dict(fifo_full_abort_reset=(a.hold>0 and 'RTL_FIFO_FULL_ABORT_RESET_PASS' in log),passed=passed,seconds=time.monotonic()-tic,queue=a.queue,latency=a.latency,write_hold=a.hold,board_timing_model=False)
    result['cases']=[dict(n=int(n),cycles=int(c),matrix_beats=int(b)) for n,c,b in re.findall(r'RTL_LITE_CASE_PASS n=(\d+) cycles=(\d+) stalls=\d+ matrix_beats=(\d+)',log)]
    m=re.search(r'RTL_SUPPLY_METRICS (.*)',log)
    if m:result['coverage']={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',m[1])}
    (reports/'result.json').write_text(json.dumps(result,indent=2))
    if not passed:raise RuntimeError('Supplemental RTL failed')
    print('RTL_SUPPLY_PASS',json.dumps(result),flush=True)
if __name__=='__main__':main()
