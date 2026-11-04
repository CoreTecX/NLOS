`timescale 1ns/1ps
module rtl_lite_tb;
reg ap_clk=0;always #5 ap_clk=~ap_clk;reg ap_rst_n=0;reg stalls=1;
wire m_axi_mat_AWVALID;
wire m_axi_mat_AWREADY;
wire [63:0] m_axi_mat_AWADDR;
wire [0:0] m_axi_mat_AWID;
wire [7:0] m_axi_mat_AWLEN;
wire [2:0] m_axi_mat_AWSIZE;
wire [1:0] m_axi_mat_AWBURST;
wire [1:0] m_axi_mat_AWLOCK;
wire [3:0] m_axi_mat_AWCACHE;
wire [2:0] m_axi_mat_AWPROT;
wire [3:0] m_axi_mat_AWQOS;
wire [3:0] m_axi_mat_AWREGION;
wire [0:0] m_axi_mat_AWUSER;
wire m_axi_mat_WVALID;
wire m_axi_mat_WREADY;
wire [127:0] m_axi_mat_WDATA;
wire [15:0] m_axi_mat_WSTRB;
wire m_axi_mat_WLAST;
wire [0:0] m_axi_mat_WID;
wire [0:0] m_axi_mat_WUSER;
wire m_axi_mat_ARVALID;
wire m_axi_mat_ARREADY;
wire [63:0] m_axi_mat_ARADDR;
wire [0:0] m_axi_mat_ARID;
wire [7:0] m_axi_mat_ARLEN;
wire [2:0] m_axi_mat_ARSIZE;
wire [1:0] m_axi_mat_ARBURST;
wire [1:0] m_axi_mat_ARLOCK;
wire [3:0] m_axi_mat_ARCACHE;
wire [2:0] m_axi_mat_ARPROT;
wire [3:0] m_axi_mat_ARQOS;
wire [3:0] m_axi_mat_ARREGION;
wire [0:0] m_axi_mat_ARUSER;
wire m_axi_mat_RVALID;
wire m_axi_mat_RREADY;
wire [127:0] m_axi_mat_RDATA;
wire m_axi_mat_RLAST;
wire [0:0] m_axi_mat_RID;
wire [0:0] m_axi_mat_RUSER;
wire [1:0] m_axi_mat_RRESP;
wire m_axi_mat_BVALID;
wire m_axi_mat_BREADY;
wire [1:0] m_axi_mat_BRESP;
wire [0:0] m_axi_mat_BID;
wire [0:0] m_axi_mat_BUSER;
wire m_axi_vec_AWVALID;
wire m_axi_vec_AWREADY;
wire [63:0] m_axi_vec_AWADDR;
wire [0:0] m_axi_vec_AWID;
wire [7:0] m_axi_vec_AWLEN;
wire [2:0] m_axi_vec_AWSIZE;
wire [1:0] m_axi_vec_AWBURST;
wire [1:0] m_axi_vec_AWLOCK;
wire [3:0] m_axi_vec_AWCACHE;
wire [2:0] m_axi_vec_AWPROT;
wire [3:0] m_axi_vec_AWQOS;
wire [3:0] m_axi_vec_AWREGION;
wire [0:0] m_axi_vec_AWUSER;
wire m_axi_vec_WVALID;
wire m_axi_vec_WREADY;
wire [63:0] m_axi_vec_WDATA;
wire [7:0] m_axi_vec_WSTRB;
wire m_axi_vec_WLAST;
wire [0:0] m_axi_vec_WID;
wire [0:0] m_axi_vec_WUSER;
wire m_axi_vec_ARVALID;
wire m_axi_vec_ARREADY;
wire [63:0] m_axi_vec_ARADDR;
wire [0:0] m_axi_vec_ARID;
wire [7:0] m_axi_vec_ARLEN;
wire [2:0] m_axi_vec_ARSIZE;
wire [1:0] m_axi_vec_ARBURST;
wire [1:0] m_axi_vec_ARLOCK;
wire [3:0] m_axi_vec_ARCACHE;
wire [2:0] m_axi_vec_ARPROT;
wire [3:0] m_axi_vec_ARQOS;
wire [3:0] m_axi_vec_ARREGION;
wire [0:0] m_axi_vec_ARUSER;
wire m_axi_vec_RVALID;
wire m_axi_vec_RREADY;
wire [63:0] m_axi_vec_RDATA;
wire m_axi_vec_RLAST;
wire [0:0] m_axi_vec_RID;
wire [0:0] m_axi_vec_RUSER;
wire [1:0] m_axi_vec_RRESP;
wire m_axi_vec_BVALID;
wire m_axi_vec_BREADY;
wire [1:0] m_axi_vec_BRESP;
wire [0:0] m_axi_vec_BID;
wire [0:0] m_axi_vec_BUSER;
reg s_axi_control_AWVALID=0;
wire s_axi_control_AWREADY;
reg [6:0] s_axi_control_AWADDR=0;
reg s_axi_control_WVALID=0;
wire s_axi_control_WREADY;
reg [31:0] s_axi_control_WDATA=0;
reg [3:0] s_axi_control_WSTRB=0;
reg s_axi_control_ARVALID=0;
wire s_axi_control_ARREADY;
reg [6:0] s_axi_control_ARADDR=0;
wire s_axi_control_RVALID;
reg s_axi_control_RREADY=0;
wire [31:0] s_axi_control_RDATA;
wire [1:0] s_axi_control_RRESP;
wire s_axi_control_BVALID;
reg s_axi_control_BREADY=0;
wire [1:0] s_axi_control_BRESP;
wire interrupt;
full_gemv dut(.ap_clk(ap_clk),.ap_rst_n(ap_rst_n),.m_axi_mat_AWVALID(m_axi_mat_AWVALID),.m_axi_mat_AWREADY(m_axi_mat_AWREADY),.m_axi_mat_AWADDR(m_axi_mat_AWADDR),.m_axi_mat_AWID(m_axi_mat_AWID),.m_axi_mat_AWLEN(m_axi_mat_AWLEN),.m_axi_mat_AWSIZE(m_axi_mat_AWSIZE),.m_axi_mat_AWBURST(m_axi_mat_AWBURST),.m_axi_mat_AWLOCK(m_axi_mat_AWLOCK),.m_axi_mat_AWCACHE(m_axi_mat_AWCACHE),.m_axi_mat_AWPROT(m_axi_mat_AWPROT),.m_axi_mat_AWQOS(m_axi_mat_AWQOS),.m_axi_mat_AWREGION(m_axi_mat_AWREGION),.m_axi_mat_AWUSER(m_axi_mat_AWUSER),.m_axi_mat_WVALID(m_axi_mat_WVALID),.m_axi_mat_WREADY(m_axi_mat_WREADY),.m_axi_mat_WDATA(m_axi_mat_WDATA),.m_axi_mat_WSTRB(m_axi_mat_WSTRB),.m_axi_mat_WLAST(m_axi_mat_WLAST),.m_axi_mat_WID(m_axi_mat_WID),.m_axi_mat_WUSER(m_axi_mat_WUSER),.m_axi_mat_ARVALID(m_axi_mat_ARVALID),.m_axi_mat_ARREADY(m_axi_mat_ARREADY),.m_axi_mat_ARADDR(m_axi_mat_ARADDR),.m_axi_mat_ARID(m_axi_mat_ARID),.m_axi_mat_ARLEN(m_axi_mat_ARLEN),.m_axi_mat_ARSIZE(m_axi_mat_ARSIZE),.m_axi_mat_ARBURST(m_axi_mat_ARBURST),.m_axi_mat_ARLOCK(m_axi_mat_ARLOCK),.m_axi_mat_ARCACHE(m_axi_mat_ARCACHE),.m_axi_mat_ARPROT(m_axi_mat_ARPROT),.m_axi_mat_ARQOS(m_axi_mat_ARQOS),.m_axi_mat_ARREGION(m_axi_mat_ARREGION),.m_axi_mat_ARUSER(m_axi_mat_ARUSER),.m_axi_mat_RVALID(m_axi_mat_RVALID),.m_axi_mat_RREADY(m_axi_mat_RREADY),.m_axi_mat_RDATA(m_axi_mat_RDATA),.m_axi_mat_RLAST(m_axi_mat_RLAST),.m_axi_mat_RID(m_axi_mat_RID),.m_axi_mat_RUSER(m_axi_mat_RUSER),.m_axi_mat_RRESP(m_axi_mat_RRESP),.m_axi_mat_BVALID(m_axi_mat_BVALID),.m_axi_mat_BREADY(m_axi_mat_BREADY),.m_axi_mat_BRESP(m_axi_mat_BRESP),.m_axi_mat_BID(m_axi_mat_BID),.m_axi_mat_BUSER(m_axi_mat_BUSER),.m_axi_vec_AWVALID(m_axi_vec_AWVALID),.m_axi_vec_AWREADY(m_axi_vec_AWREADY),.m_axi_vec_AWADDR(m_axi_vec_AWADDR),.m_axi_vec_AWID(m_axi_vec_AWID),.m_axi_vec_AWLEN(m_axi_vec_AWLEN),.m_axi_vec_AWSIZE(m_axi_vec_AWSIZE),.m_axi_vec_AWBURST(m_axi_vec_AWBURST),.m_axi_vec_AWLOCK(m_axi_vec_AWLOCK),.m_axi_vec_AWCACHE(m_axi_vec_AWCACHE),.m_axi_vec_AWPROT(m_axi_vec_AWPROT),.m_axi_vec_AWQOS(m_axi_vec_AWQOS),.m_axi_vec_AWREGION(m_axi_vec_AWREGION),.m_axi_vec_AWUSER(m_axi_vec_AWUSER),.m_axi_vec_WVALID(m_axi_vec_WVALID),.m_axi_vec_WREADY(m_axi_vec_WREADY),.m_axi_vec_WDATA(m_axi_vec_WDATA),.m_axi_vec_WSTRB(m_axi_vec_WSTRB),.m_axi_vec_WLAST(m_axi_vec_WLAST),.m_axi_vec_WID(m_axi_vec_WID),.m_axi_vec_WUSER(m_axi_vec_WUSER),.m_axi_vec_ARVALID(m_axi_vec_ARVALID),.m_axi_vec_ARREADY(m_axi_vec_ARREADY),.m_axi_vec_ARADDR(m_axi_vec_ARADDR),.m_axi_vec_ARID(m_axi_vec_ARID),.m_axi_vec_ARLEN(m_axi_vec_ARLEN),.m_axi_vec_ARSIZE(m_axi_vec_ARSIZE),.m_axi_vec_ARBURST(m_axi_vec_ARBURST),.m_axi_vec_ARLOCK(m_axi_vec_ARLOCK),.m_axi_vec_ARCACHE(m_axi_vec_ARCACHE),.m_axi_vec_ARPROT(m_axi_vec_ARPROT),.m_axi_vec_ARQOS(m_axi_vec_ARQOS),.m_axi_vec_ARREGION(m_axi_vec_ARREGION),.m_axi_vec_ARUSER(m_axi_vec_ARUSER),.m_axi_vec_RVALID(m_axi_vec_RVALID),.m_axi_vec_RREADY(m_axi_vec_RREADY),.m_axi_vec_RDATA(m_axi_vec_RDATA),.m_axi_vec_RLAST(m_axi_vec_RLAST),.m_axi_vec_RID(m_axi_vec_RID),.m_axi_vec_RUSER(m_axi_vec_RUSER),.m_axi_vec_RRESP(m_axi_vec_RRESP),.m_axi_vec_BVALID(m_axi_vec_BVALID),.m_axi_vec_BREADY(m_axi_vec_BREADY),.m_axi_vec_BRESP(m_axi_vec_BRESP),.m_axi_vec_BID(m_axi_vec_BID),.m_axi_vec_BUSER(m_axi_vec_BUSER),.s_axi_control_AWVALID(s_axi_control_AWVALID),.s_axi_control_AWREADY(s_axi_control_AWREADY),.s_axi_control_AWADDR(s_axi_control_AWADDR),.s_axi_control_WVALID(s_axi_control_WVALID),.s_axi_control_WREADY(s_axi_control_WREADY),.s_axi_control_WDATA(s_axi_control_WDATA),.s_axi_control_WSTRB(s_axi_control_WSTRB),.s_axi_control_ARVALID(s_axi_control_ARVALID),.s_axi_control_ARREADY(s_axi_control_ARREADY),.s_axi_control_ARADDR(s_axi_control_ARADDR),.s_axi_control_RVALID(s_axi_control_RVALID),.s_axi_control_RREADY(s_axi_control_RREADY),.s_axi_control_RDATA(s_axi_control_RDATA),.s_axi_control_RRESP(s_axi_control_RRESP),.s_axi_control_BVALID(s_axi_control_BVALID),.s_axi_control_BREADY(s_axi_control_BREADY),.s_axi_control_BRESP(s_axi_control_BRESP),.interrupt(interrupt));
integer mat_reads,mat_writes;
axi_memory #(.BASE(64'h40000000),.WORDS(544968),.READONLY(1),.DATA_WIDTH(128)) mat_memory(.clk(ap_clk),.resetn(ap_rst_n),.stalls(stalls),.reads(mat_reads),.writes(mat_writes),.arvalid(m_axi_mat_ARVALID),.arready(m_axi_mat_ARREADY),.araddr(m_axi_mat_ARADDR),.arlen(m_axi_mat_ARLEN),.arsize(m_axi_mat_ARSIZE),.arburst(m_axi_mat_ARBURST),.arid(m_axi_mat_ARID),.rvalid(m_axi_mat_RVALID),.rready(m_axi_mat_RREADY),.rdata(m_axi_mat_RDATA),.rlast(m_axi_mat_RLAST),.rid(m_axi_mat_RID),.rresp(m_axi_mat_RRESP),.awvalid(m_axi_mat_AWVALID),.awready(m_axi_mat_AWREADY),.awaddr(m_axi_mat_AWADDR),.awlen(m_axi_mat_AWLEN),.awsize(m_axi_mat_AWSIZE),.awburst(m_axi_mat_AWBURST),.awid(m_axi_mat_AWID),.wvalid(m_axi_mat_WVALID),.wready(m_axi_mat_WREADY),.wdata(m_axi_mat_WDATA),.wstrb(m_axi_mat_WSTRB),.wlast(m_axi_mat_WLAST),.bvalid(m_axi_mat_BVALID),.bready(m_axi_mat_BREADY),.bid(m_axi_mat_BID),.bresp(m_axi_mat_BRESP));
assign m_axi_mat_RUSER=0;
assign m_axi_mat_BUSER=0;
integer vec_reads,vec_writes;
axi_memory #(.BASE(64'h41200000),.WORDS(262144),.READONLY(0),.DATA_WIDTH(64)) vec_memory(.clk(ap_clk),.resetn(ap_rst_n),.stalls(stalls),.reads(vec_reads),.writes(vec_writes),.arvalid(m_axi_vec_ARVALID),.arready(m_axi_vec_ARREADY),.araddr(m_axi_vec_ARADDR),.arlen(m_axi_vec_ARLEN),.arsize(m_axi_vec_ARSIZE),.arburst(m_axi_vec_ARBURST),.arid(m_axi_vec_ARID),.rvalid(m_axi_vec_RVALID),.rready(m_axi_vec_RREADY),.rdata(m_axi_vec_RDATA),.rlast(m_axi_vec_RLAST),.rid(m_axi_vec_RID),.rresp(m_axi_vec_RRESP),.awvalid(m_axi_vec_AWVALID),.awready(m_axi_vec_AWREADY),.awaddr(m_axi_vec_AWADDR),.awlen(m_axi_vec_AWLEN),.awsize(m_axi_vec_AWSIZE),.awburst(m_axi_vec_AWBURST),.awid(m_axi_vec_AWID),.wvalid(m_axi_vec_WVALID),.wready(m_axi_vec_WREADY),.wdata(m_axi_vec_WDATA),.wstrb(m_axi_vec_WSTRB),.wlast(m_axi_vec_WLAST),.bvalid(m_axi_vec_BVALID),.bready(m_axi_vec_BREADY),.bid(m_axi_vec_BID),.bresp(m_axi_vec_BRESP));
assign m_axi_vec_RUSER=0;
assign m_axi_vec_BUSER=0;

integer cycles=0,selected_case=0,case_number=0;

// P1 observation counters. No DUT inputs or memory behavior are driven here.
integer p1_total=0,p1_mat_beats=0,p1_mat_backpressure=0,p1_ar_stalls=0;
integer p1_vec_reads=0,p1_vec_writes=0,p1_vec_wstall=0,p1_rgap=0;
integer p1_load=0,p1_compute=0,p1_reduce_write=0,p1_control=0;
integer p1_first_mat=-1,p1_last_mat=-1,p1_first_write=-1,p1_last_write=-1;
always @(posedge ap_clk) begin
 if(ap_rst_n && !dut.ap_idle)begin
  p1_total=p1_total+1;
  if(dut.ap_CS_fsm[9] || dut.ap_CS_fsm[10])p1_load=p1_load+1;
  else if(dut.ap_CS_fsm[20] || dut.ap_CS_fsm[21])p1_compute=p1_compute+1;
  else if(dut.ap_CS_fsm[22] || dut.ap_CS_fsm[23])p1_reduce_write=p1_reduce_write+1;
  else p1_control=p1_control+1;
  if(m_axi_mat_RVALID && m_axi_mat_RREADY)begin
   p1_mat_beats=p1_mat_beats+1;
   if(p1_first_mat<0)p1_first_mat=cycles;p1_last_mat=cycles;
  end
  if(m_axi_mat_RVALID && !m_axi_mat_RREADY)p1_mat_backpressure=p1_mat_backpressure+1;
  if(m_axi_mat_ARVALID && !m_axi_mat_ARREADY)p1_ar_stalls=p1_ar_stalls+1;
  if(mat_memory.rbusy && !m_axi_mat_RVALID)p1_rgap=p1_rgap+1;
  if(m_axi_vec_RVALID && m_axi_vec_RREADY)p1_vec_reads=p1_vec_reads+1;
  if(m_axi_vec_WVALID && m_axi_vec_WREADY)begin
   p1_vec_writes=p1_vec_writes+1;
   if(p1_first_write<0)p1_first_write=cycles;p1_last_write=cycles;
  end
  if(m_axi_vec_WVALID && !m_axi_vec_WREADY)p1_vec_wstall=p1_vec_wstall+1;
 end
end

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
  integer start_cycle,mr,vw,vr;reg [31:0] control,ret;real actual,reference,diff,maxerr,maxref;
  begin: selected_body
  case_number=case_number+1;
  if(selected_case==-1 && case_number>9)disable selected_body;
  if(selected_case>0 && selected_case!=case_number)disable selected_body;
  wr('h44,layout);start_cycle=cycles;mr=mat_reads;vw=vec_writes;vr=vec_reads;
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
    $display("RTL_LITE_NUMERIC n=%0d max_abs=%g",n,maxerr);
  end
  $display("P1_RTL_COUNTERS total=%0d load=%0d compute=%0d reduction_write=%0d control=%0d mat_beats=%0d mat_backpressure=%0d ar_stalls=%0d read_gaps=%0d vec_reads=%0d vec_writes=%0d write_stalls=%0d first_mat=%0d last_mat=%0d first_write=%0d last_write=%0d",p1_total,p1_load,p1_compute,p1_reduce_write,p1_control,p1_mat_beats,p1_mat_backpressure,p1_ar_stalls,p1_rgap,p1_vec_reads,p1_vec_writes,p1_vec_wstall,p1_first_mat,p1_last_mat,p1_first_write,p1_last_write);
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
  if($test$plusargs("P1_CONTINUOUS"))stalls=0;
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
  if((selected_case==0||selected_case==14)&&(mat_memory.ar_stalls==0||mat_memory.r_gaps==0||vec_memory.w_stalls==0))$fatal(1,"Stall coverage missing");
  $display("RTL_STALL_COVERAGE mat_ar=%0d mat_r_gaps=%0d vec_aw=%0d vec_w=%0d",mat_memory.ar_stalls,mat_memory.r_gaps,vec_memory.aw_stalls,vec_memory.w_stalls);
  if(selected_case==0)$display("RTL_LITE_ALL_PASS cases=14 real_model=d11_rgb Kt_then_K bit_exact=1 full_stalls=1");
  else $display("RTL_SHARD_PASS case=%0d",selected_case);
  $finish;
end
endmodule
