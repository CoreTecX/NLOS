`timescale 1ns/1ps
// Deliberately stalled, single-outstanding AXI4 slave. Address and burst checks
// catch accesses outside the allocated buffers. No UVM object allocation.
module axi_memory #(parameter longint BASE=0, parameter int WORDS=1044*1044, parameter bit READONLY=0, parameter int DATA_WIDTH=64)(
    input clk,resetn,stalls,
    input arvalid,output arready,input [63:0] araddr,input [7:0] arlen,input [2:0] arsize,input [1:0] arburst,input arid,
    output reg rvalid,input rready,output reg [DATA_WIDTH-1:0] rdata,output reg rlast,output reg rid,output [1:0] rresp,
    input awvalid,output awready,input [63:0] awaddr,input [7:0] awlen,input [2:0] awsize,input [1:0] awburst,input awid,
    input wvalid,output wready,input [DATA_WIDTH-1:0] wdata,input [DATA_WIDTH/8-1:0] wstrb,input wlast,
    output reg bvalid,input bready,output reg bid,output [1:0] bresp,
    output integer reads,output integer writes
);
    localparam int BEAT_BYTES=DATA_WIDTH/8;
    localparam int BEAT_SIZE=$clog2(BEAT_BYTES);
    reg [DATA_WIDTH-1:0] mem[0:WORDS-1];
    integer tick=0,rleft=0,wleft=0;longint ri=0,wi=0;reg rbusy=0,wbusy=0;
    assign arready=resetn&&!rbusy&&(!stalls||tick%5!=0);
    assign awready=resetn&&!wbusy&&!bvalid&&(!stalls||tick%11!=0);
    assign wready=resetn&&wbusy&&(!stalls||tick%3!=0);
    assign rresp=0;assign bresp=0;
    always @(posedge clk) begin
        if(!resetn)begin rvalid<=0;bvalid<=0;rbusy<=0;wbusy<=0;reads<=0;writes<=0;tick<=0;rlast<=0;rid<=0;bid<=0;rdata<=0;end
        else begin
            tick<=tick+1;
            if(arvalid&&arready)begin
                if(arsize!=BEAT_SIZE||arburst!=1||araddr<BASE||araddr%BEAT_BYTES!=0||araddr+BEAT_BYTES*(arlen+1)>BASE+WORDS*BEAT_BYTES||araddr[11:0]+BEAT_BYTES*(arlen+1)>4096)$fatal(1,"Illegal AXI read %h len=%d",araddr,arlen);
                ri<=(araddr-BASE)/BEAT_BYTES;rleft<=arlen+1;rbusy<=1;rid<=arid;
            end
            if(rvalid&&rready)begin
                reads<=reads+1;ri<=ri+1;rleft<=rleft-1;
                if(rleft==1)begin rbusy<=0;rvalid<=0;end
                else if(stalls&&tick%7==0)rvalid<=0;
                else begin rvalid<=1;rdata<=mem[ri+1];rlast<=(rleft==2);end
            end
            else if(rbusy&&!rvalid&&(!stalls||tick%7!=0))begin rvalid<=1;rdata<=mem[ri];rlast<=(rleft==1);end
            if(awvalid&&awready)begin
                if(READONLY||awsize!=BEAT_SIZE||awburst!=1||awaddr<BASE||awaddr%BEAT_BYTES!=0||awaddr+BEAT_BYTES*(awlen+1)>BASE+WORDS*BEAT_BYTES||awaddr[11:0]+BEAT_BYTES*(awlen+1)>4096)$fatal(1,"Illegal AXI write %h len=%d",awaddr,awlen);
                wi<=(awaddr-BASE)/BEAT_BYTES;wleft<=awlen+1;wbusy<=1;bid<=awid;
            end
            if(wvalid&&wready)begin
                if(wstrb!={BEAT_BYTES{1'b1}}||wlast!=(wleft==1))$fatal(1,"Bad AXI strobe / WLAST");
                mem[wi]<=wdata;wi<=wi+1;wleft<=wleft-1;writes<=writes+1;
                if(wleft==1)begin wbusy<=0;bvalid<=1;end
            end
            if(bvalid&&bready)bvalid<=0;
        end
    end

    // AXI master must retain VALID and payload throughout slave backpressure.
    reg arblocked=0,awblocked=0,wblocked=0;
    reg [77:0] prev_ar,prev_aw;
    reg [DATA_WIDTH+DATA_WIDTH/8:0] prev_w;
    integer ar_stalls=0,aw_stalls=0,w_stalls=0,r_gaps=0;
    always @(posedge clk)begin
        if(!resetn)begin arblocked<=0;awblocked<=0;wblocked<=0;end
        else begin
            if(arblocked&&(!arvalid||{araddr,arlen,arsize,arburst,arid}!==prev_ar))$fatal(1,"AR changed while blocked");
            if(awblocked&&(!awvalid||{awaddr,awlen,awsize,awburst,awid}!==prev_aw))$fatal(1,"AW changed while blocked");
            if(wblocked&&(!wvalid||{wdata,wstrb,wlast}!==prev_w))$fatal(1,"W changed while blocked");
            arblocked<=arvalid&&!arready;awblocked<=awvalid&&!awready;wblocked<=wvalid&&!wready;
            prev_ar<={araddr,arlen,arsize,arburst,arid};prev_aw<={awaddr,awlen,awsize,awburst,awid};prev_w<={wdata,wstrb,wlast};
            if(arvalid&&!arready)ar_stalls<=ar_stalls+1;
            if(awvalid&&!awready)aw_stalls<=aw_stalls+1;
            if(wvalid&&!wready)w_stalls<=w_stalls+1;
            if(rbusy&&!rvalid)r_gaps<=r_gaps+1;
        end
    end
endmodule
