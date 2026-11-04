"""Generate a small AXI test harness around the unchanged HLS RTL."""
from pathlib import Path
import re,shutil,json,hashlib,stat,os
root=Path(__file__).resolve().parents[1];out=root/'build'/os.environ.get('PS_RTL_OUTPUT','rtl_lite');out.mkdir(parents=True,exist_ok=True)
def copy(src,target):
    if target.exists():
        assert out.resolve() in target.resolve().parents
        target.chmod(target.stat().st_mode|stat.S_IWRITE)
    shutil.copy2(src,target)
ip=root/'full_gemv_hls'/'solution1'/'impl'/'ip';sim=ip/'hdl'/'verilog';dut=(sim/'full_gemv.v').read_text()
ports=re.findall(r'^(input|output)\s+(?:\[([^]]+)\]\s+)?(\w+)\s*;',dut,re.M)
params=dict(re.findall(r'parameter\s+(C_\w+)\s*=\s*([^;]+);',dut))
def width(expr):
    if not expr:return ''
    for k in sorted(params,key=len,reverse=True):expr=re.sub(r'\b'+k+r'\b',params[k],expr)
    a,b=expr.split(':');return f'[{int(eval(a,{"__builtins__":{}},{}))}:{int(eval(b,{"__builtins__":{}},{}))}] '
lines=['`timescale 1ns/1ps','module rtl_lite_tb;','reg ap_clk=0;always #5 ap_clk=~ap_clk;reg ap_rst_n=0;reg stalls=1;']
for direction,w,name in ports:
    if name in ('ap_clk','ap_rst_n'):continue
    is_control_input=direction=='input' and name.startswith('s_axi_control')
    lines.append(('reg ' if is_control_input else 'wire ')+width(w)+name+('=0;' if is_control_input else ';'))
lines.append('full_gemv dut('+','.join('.'+name+'('+name+')' for _,_,name in ports)+');')
for bundle,base,words,ro,bits in [('mat',0x40000000,522*1044,1,128),('vec',0x41200000,262144,0,64)]:
    lines.append(f'integer {bundle}_reads,{bundle}_writes;')
    fields=['arvalid','arready','araddr','arlen','arsize','arburst','arid','rvalid','rready','rdata','rlast','rid','rresp','awvalid','awready','awaddr','awlen','awsize','awburst','awid','wvalid','wready','wdata','wstrb','wlast','bvalid','bready','bid','bresp']
    lines.append(f'axi_memory #(.BASE(64\'h{base:x}),.WORDS({words}),.READONLY({ro}),.DATA_WIDTH({bits})) {bundle}_memory(.clk(ap_clk),.resetn(ap_rst_n),.stalls(stalls),.reads({bundle}_reads),.writes({bundle}_writes),'+','.join('.'+f+'(m_axi_'+bundle+'_'+f.upper()+')' for f in fields)+');')
    for signal in ['RUSER','BUSER']:lines.append(f'assign m_axi_{bundle}_{signal}=0;')
run_instance=re.search(r'full_gemv_run_pairs\s+(\w+)\s*\(',dut)
if not run_instance:raise RuntimeError('P5 dataflow module absent')
flow='dut.'+run_instance.group(1)
lines += [
    'integer stage_overlap_cycles=0, overlap_cycles=0, fifo_full_cycles=0, tokens_written=0,tokens_read=0;',
    'always @(posedge ap_clk) if(ap_rst_n) begin',
    f'  if(!{flow}.compute_pairs_U0_ap_idle && !{flow}.reduce_pairs_U0_ap_idle) stage_overlap_cycles<=stage_overlap_cycles+1;',
    f'  if(m_axi_mat_RVALID && m_axi_mat_RREADY && m_axi_vec_WVALID && m_axi_vec_WREADY) overlap_cycles<=overlap_cycles+1;',
    f'  if(!{flow}.completed_full_n) fifo_full_cycles<=fifo_full_cycles+1;',
    f'  if({flow}.compute_pairs_U0_completed_0_write && {flow}.completed_full_n) tokens_written<=tokens_written+1;',
    f'  if({flow}.reduce_pairs_U0_completed_0_read && {flow}.completed_empty_n) tokens_read<=tokens_read+1;',
    'end'
]
lines.append(r'''
integer cycles=0,selected_case=0,case_number=0;
always @(posedge ap_clk)begin
  cycles<=cycles+1;
  if(cycles%100000==0)$display("RTL_LITE_PROGRESS cycles=%0d mat_reads=%0d vec_writes=%0d",cycles,mat_reads,vec_writes);
  if(cycles>12000000)$fatal(1,"RTL simulation watchdog expired");
end
task automatic wr(input [6:0] address,input [31:0] value);
  @(negedge ap_clk);s_axi_control_AWADDR=address;s_axi_control_AWVALID=1;
  do @(posedge ap_clk);while(!s_axi_control_AWREADY);
  @(negedge ap_clk);s_axi_control_AWVALID=0;s_axi_control_WDATA=value;s_axi_control_WSTRB=15;s_axi_control_WVALID=1;s_axi_control_BREADY=1;
  do @(posedge ap_clk);while(!s_axi_control_WREADY);
  @(negedge ap_clk);s_axi_control_WVALID=0;
  do @(posedge ap_clk);while(!s_axi_control_BVALID);
  if(s_axi_control_BRESP!=0)$fatal(1,"AXI-Lite write failure");
  @(negedge ap_clk);s_axi_control_BREADY=0;
endtask
task automatic rd(input [6:0] address,output [31:0] value);
  @(negedge ap_clk);s_axi_control_ARADDR=address;s_axi_control_ARVALID=1;s_axi_control_RREADY=1;
  do @(posedge ap_clk);while(!s_axi_control_ARREADY);
  @(negedge ap_clk);s_axi_control_ARVALID=0;
  do @(posedge ap_clk);while(!s_axi_control_RVALID);
  value=s_axi_control_RDATA;if(s_axi_control_RRESP!=0)$fatal(1,"AXI-Lite read failure");
  @(negedge ap_clk);s_axi_control_RREADY=0;
endtask
reg [63:0] expected[0:3131];
task automatic run_case(input integer n,input string golden,input [31:0] status,input [31:0] layout);
  integer start_cycle,mr,vw,vr,ow,sw,tw,tr;reg [31:0] control,ret;real actual,reference,diff,maxerr,maxref;
  begin: selected_body
  case_number=case_number+1;
  if(selected_case==-1 && case_number>9 && case_number<15)disable selected_body;
  if(selected_case>0 && selected_case!=case_number)disable selected_body;
  wr('h44,layout);start_cycle=cycles;mr=mat_reads;vw=vec_writes;vr=vec_reads;ow=overlap_cycles;sw=stage_overlap_cycles;tw=tokens_written;tr=tokens_read;
  for(integer i=0;i<3148;i=i+1)vec_memory.mem[131072+i]=64'hdeadbeefcafef00d;
  wr('h34,n);wr(0,1);control=0;
  while(!control[1])begin repeat(20)@(posedge ap_clk);rd(0,control);end
  rd('h3c,ret);if(ret!==status)$fatal(1,"Wrong return status %h expected %h",ret,status);
  if(status!=32'h46500200)begin
    if(mat_reads!=mr||vec_writes!=vw||vec_reads!=vr)$fatal(1,"Invalid n touched DDR");
    for(integer i=0;i<3148;i=i+1)if(vec_memory.mem[131072+i]!==64'hdeadbeefcafef00d)$fatal(1,"Invalid n modified output");
  end else begin
    if(mat_reads-mr!=((n+1)/2)*n||vec_reads-vr!=3*n||vec_writes-vw!=3*n)$fatal(1,"Unexpected transfer sizes");
    $readmemh(golden,expected,0,3*n-1);maxerr=0;maxref=0;
    for(integer i=0;i<3*n;i=i+1)begin
      if(^vec_memory.mem[131072+i]===1'bx||vec_memory.mem[131072+i][62:52]==11'h7ff)$fatal(1,"Nonfinite/unknown RTL output");
      if(vec_memory.mem[131072+i]!==expected[i])$fatal(1,"FP64 bit mismatch n=%0d index=%0d actual=%h expected=%h",n,i,vec_memory.mem[131072+i],expected[i]);
      actual=$bitstoreal(vec_memory.mem[131072+i]);reference=$bitstoreal(expected[i]);diff=actual-reference;
      if(diff<0)diff=-diff;if(diff>maxerr)maxerr=diff;if(reference<0)reference=-reference;if(reference>maxref)maxref=reference;
    end
    if(maxerr>1e-11*(maxref>1?maxref:1))$fatal(1,"RTL numerical mismatch %g",maxerr);
    for(integer i=3*n;i<3148;i=i+1)if(vec_memory.mem[131072+i]!==64'hdeadbeefcafef00d)$fatal(1,"Output overrun");
    if(tokens_written-tw!=(n+1)/2 || tokens_read-tr!=(n+1)/2)$fatal(1,"Missing/extra row-pair token");
    if(n>=64 && stage_overlap_cycles==sw)$fatal(1,"No measured producer/consumer activity overlap");
    $display("RTL_DATAFLOW n=%0d overlap_cycles=%0d tokens=%0d fifo_full_total=%0d stage_overlap_cycles=%0d",n,overlap_cycles-ow,tokens_read-tr,fifo_full_cycles,stage_overlap_cycles-sw);
    $display("RTL_LITE_NUMERIC n=%0d max_abs=%g",n,maxerr);
  end
  $display("RTL_LITE_CASE_PASS n=%0d cycles=%0d stalls=%0d matrix_beats=%0d status=%h",n,cycles-start_cycle,stalls,mat_reads-mr,ret);
  end
endtask
initial begin
  if(!$value$plusargs("CASE=%d",selected_case))selected_case=0;
  $readmemh("matrix_kt.hex",mat_memory.mem);
  $readmemh("input.hex",vec_memory.mem,0,3131);
  repeat(20)@(negedge ap_clk);ap_rst_n=1;repeat(10)@(negedge ap_clk);
  wr('h10,32'h40000000);wr('h14,0);wr('h1c,32'h41200000);wr('h20,0);wr('h28,32'h41300000);wr('h2c,0);

  run_case(0,"",32'h46500201,32'h32525001);
  run_case(1045,"",32'h46500201,32'h32525001);
  run_case(-1,"",32'h46500201,32'h32525001);
  run_case(1044,"",32'h46500202,0);
  run_case(0,"",32'h46500202,0);
  $readmemh("matrix_1.hex",mat_memory.mem,0,0);run_case(1,"expected_1.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_2.hex",mat_memory.mem,0,1);run_case(2,"expected_2.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_17.hex",mat_memory.mem,0,152);run_case(17,"expected_17.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_64.hex",mat_memory.mem,0,2047);run_case(64,"expected_64.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_1043.hex",mat_memory.mem,0,544445);run_case(1043,"expected_1043.hex",32'h46500200,32'h32525001);
  stalls=0;
  $readmemh("matrix_kt.hex",mat_memory.mem);run_case(1044,"expected_1044.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_k.hex",mat_memory.mem);$readmemh("input_forward.hex",vec_memory.mem,0,3131);
  run_case(1044,"expected_forward.hex",32'h46500200,32'h32525001);
  stalls=1;
  $readmemh("matrix_kt.hex",mat_memory.mem);$readmemh("input.hex",vec_memory.mem,0,3131);
  run_case(1044,"expected_1044.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_k.hex",mat_memory.mem);$readmemh("input_forward.hex",vec_memory.mem,0,3131);
  run_case(1044,"expected_forward.hex",32'h46500200,32'h32525001);
  // Reuse both bounded FIFOs after changing dimensions and source buffers.
  $readmemh("input.hex",vec_memory.mem,0,3131);
  $readmemh("matrix_17.hex",mat_memory.mem,0,152);run_case(17,"expected_17.hex",32'h46500200,32'h32525001);
  run_case(0,"",32'h46500201,32'h32525001);
  $readmemh("matrix_2.hex",mat_memory.mem,0,1);run_case(2,"expected_2.hex",32'h46500200,32'h32525001);
  if(selected_case==0 || selected_case==-1)begin
    // Abort while the core has accepted matrix data; reset the entire AXI domain.
    $readmemh("matrix_kt.hex",mat_memory.mem);
    wr('h44,32'h32525001);wr('h34,1044);
    begin: abort_job
      integer before_reads;
      before_reads=mat_reads;wr(0,1);
      wait(mat_reads-before_reads>=128);
      @(negedge ap_clk);ap_rst_n=0;repeat(20)@(negedge ap_clk);
      if(m_axi_mat_ARVALID || m_axi_vec_AWVALID)$fatal(1,"AXI request during held reset");
      ap_rst_n=1;repeat(20)@(negedge ap_clk);
      wr('h10,32'h40000000);wr('h14,0);wr('h1c,32'h41200000);wr('h20,0);wr('h28,32'h41300000);wr('h2c,0);
      $display("RTL_ABORT_RESET_PASS");
    end
  end
  $readmemh("matrix_64.hex",mat_memory.mem,0,2047);run_case(64,"expected_64.hex",32'h46500200,32'h32525001);
  $readmemh("matrix_1.hex",mat_memory.mem,0,0);run_case(1,"expected_1.hex",32'h46500200,32'h32525001);
  if((selected_case==0||selected_case==14)&&(mat_memory.ar_stalls==0||mat_memory.r_gaps==0||vec_memory.w_stalls==0))$fatal(1,"Stall coverage missing");
  $display("RTL_STALL_COVERAGE mat_ar=%0d mat_r_gaps=%0d vec_aw=%0d vec_w=%0d",mat_memory.ar_stalls,mat_memory.r_gaps,vec_memory.aw_stalls,vec_memory.w_stalls);
  if(selected_case==0)$display("RTL_LITE_ALL_PASS cases=19 real_model=d11_rgb Kt_then_K bit_exact=1 full_stalls=1");
  else $display("RTL_SHARD_PASS case=%0d",selected_case);
  $finish;
end
endmodule
''')
(out/'rtl_lite_tb.sv').write_text('\n'.join(lines))
copy(root/'tests'/'axi_memory.sv',out/'axi_memory.sv')
prj=[]
for src in sorted((ip/'subcore_prj'/'subcore_prj.gen'/'sources_1'/'ip').glob('*/sim/*.vhd')):
    target=out/'ip'/'xil_defaultlib'/src.name;target.parent.mkdir(parents=True,exist_ok=True);copy(src,target)
    prj.append('vhdl xil_defaultlib "'+target.relative_to(out).as_posix()+'"')
copy(Path('D:/vivado/2025.2/Vivado/data/verilog/src/glbl.v'),out/'glbl.v');prj.append('sv xil_defaultlib "glbl.v"')
for src in sorted(sim.glob('*.v')):
    copy(src,out/src.name);prj.append('sv xil_defaultlib "'+src.name+'"')
for src in sorted(sim.glob('*.vh')):copy(src,out/src.name)
prj+=['sv xil_defaultlib "axi_memory.sv"','sv xil_defaultlib "rtl_lite_tb.sv"']
(out/'rtl_lite.prj').write_text('\n'.join(prj)+'\n');(out/'run.tcl').write_text('run all\nquit\n')
provenance={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file() and p.suffix in ('.v','.vh','.vhd','.sv')}
(root/'reports'/(out.name+'_sources.json')).write_text(json.dumps(provenance,indent=2))
print('RTL_LITE_HARNESS_READY')
