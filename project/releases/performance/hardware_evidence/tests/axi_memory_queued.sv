`timescale 1ns/1ps
// Ordered AXI4 memory with a bounded request queue and configurable response
// latency. It models supply pressure, not the board's DDR controller timing.
module axi_memory_queued #(parameter longint BASE=0, parameter int WORDS=1044*1044,
    parameter bit READONLY=0, parameter int DATA_WIDTH=64)(
    input clk,resetn,stalls,
    input arvalid,output arready,input [63:0] araddr,input [7:0] arlen,input [2:0] arsize,input [1:0] arburst,input arid,
    output reg rvalid,input rready,output reg [DATA_WIDTH-1:0] rdata,output reg rlast,output reg rid,output [1:0] rresp,
    input awvalid,output awready,input [63:0] awaddr,input [7:0] awlen,input [2:0] awsize,input [1:0] awburst,input awid,
    input wvalid,output wready,input [DATA_WIDTH-1:0] wdata,input [DATA_WIDTH/8-1:0] wstrb,input wlast,
    output reg bvalid,input bready,output reg bid,output [1:0] bresp,
    output integer reads,output integer writes
);
    localparam integer BYTES=DATA_WIDTH/8, SIZE=$clog2(BYTES), CAPACITY=16;
    reg [DATA_WIDTH-1:0] mem[0:WORDS-1];
    longint qbase[0:CAPACITY-1],qready[0:CAPACITY-1];
    integer qlen[0:CAPACITY-1];reg qid[0:CAPACITY-1];
    integer head=0,tail=0,pending=0,pos=0,tick=0;
    integer queue_limit=8,latency=80,write_hold=0,hold_left=0;
    reg hold_used=0;
    integer wleft=0;longint wi=0;reg wbusy=0;
    integer ar_stalls=0,aw_stalls=0,w_stalls=0,r_gaps=0;
    integer max_pending=0,bursts=0,max_burst=0,boundary_bursts=0,r_backpressure=0;
    initial begin
        if($value$plusargs("QUEUE=%d",queue_limit))begin end
        if($value$plusargs("LATENCY=%d",latency))begin end
        if($value$plusargs("WRITE_HOLD=%d",write_hold))begin end
        if(queue_limit<1||queue_limit>CAPACITY||latency<0||write_hold<0)$fatal(1,"Invalid memory-model parameters");
    end
    wire push=arvalid&&arready, pop=rvalid&&rready&&rlast;
    assign arready=resetn&&pending<queue_limit&&(!stalls||tick%5!=0);
    assign awready=resetn&&!wbusy&&!bvalid&&(!stalls||tick%11!=0);
    assign wready=resetn&&wbusy&&hold_left==0&&(!stalls||tick%3!=0);
    assign rresp=0;assign bresp=0;
    always @(posedge clk)begin
        if(!resetn)begin
            head<=0;tail<=0;pending<=0;pos<=0;tick<=0;
            rvalid<=0;rdata<=0;rlast<=0;rid<=0;reads<=0;writes<=0;
            bvalid<=0;bid<=0;wbusy<=0;wleft<=0;wi<=0;hold_left<=0;hold_used<=0;
            ar_stalls<=0;aw_stalls<=0;w_stalls<=0;r_gaps<=0;
            max_pending<=0;bursts<=0;max_burst<=0;boundary_bursts<=0;r_backpressure<=0;
        end else begin
            tick<=tick+1;
            if(hold_left>0)hold_left<=hold_left-1;
            if(pending>max_pending)max_pending<=pending;
            case({push,pop})
                2'b10:pending<=pending+1;
                2'b01:pending<=pending-1;
                default:begin end
            endcase
            if(push)begin
                if(arsize!=SIZE||arburst!=1||araddr<BASE||araddr%BYTES!=0||araddr+BYTES*(arlen+1)>BASE+WORDS*BYTES||araddr[11:0]+BYTES*(arlen+1)>4096)$fatal(1,"Illegal queued AXI read");
                qbase[tail]<=(araddr-BASE)/BYTES;qlen[tail]<=arlen+1;qready[tail]<=tick+latency;qid[tail]<=arid;
                tail<=(tail+1)%CAPACITY;bursts<=bursts+1;
                if(arlen+1>max_burst)max_burst<=arlen+1;
                if(araddr[11:0]+BYTES*(arlen+1)==4096)boundary_bursts<=boundary_bursts+1;
            end
            if(rvalid&&rready)begin
                reads<=reads+1;
                if(rlast)begin head<=(head+1)%CAPACITY;pos<=0;rvalid<=0;end
                else begin
                    pos<=pos+1;
                    if(stalls&&tick%7==0)rvalid<=0;
                    else begin rdata<=mem[qbase[head]+pos+1];rlast<=(pos+2==qlen[head]);end
                end
            end else if(!rvalid&&pending>0&&tick>=qready[head]&&(!stalls||tick%7!=0))begin
                rvalid<=1;rdata<=mem[qbase[head]+pos];rlast<=(pos+1==qlen[head]);rid<=qid[head];
            end
            if(awvalid&&awready)begin
                if(READONLY||awsize!=SIZE||awburst!=1||awaddr<BASE||awaddr%BYTES!=0||awaddr+BYTES*(awlen+1)>BASE+WORDS*BYTES||awaddr[11:0]+BYTES*(awlen+1)>4096)$fatal(1,"Illegal queued AXI write");
                wi<=(awaddr-BASE)/BYTES;wleft<=awlen+1;wbusy<=1;bid<=awid;
                if(!hold_used)begin hold_left<=write_hold;hold_used<=1;end
            end
            if(wvalid&&wready)begin
                if(wstrb!={BYTES{1'b1}}||wlast!=(wleft==1))$fatal(1,"Bad queued AXI strobe/WLAST");
                mem[wi]<=wdata;wi<=wi+1;wleft<=wleft-1;writes<=writes+1;
                if(wleft==1)begin wbusy<=0;bvalid<=1;end
            end
            if(bvalid&&bready)bvalid<=0;
            if(arvalid&&!arready)ar_stalls<=ar_stalls+1;
            if(awvalid&&!awready)aw_stalls<=aw_stalls+1;
            if(wvalid&&!wready)w_stalls<=w_stalls+1;
            if(pending>0&&!rvalid)r_gaps<=r_gaps+1;
            if(rvalid&&!rready)r_backpressure<=r_backpressure+1;
            if(pending<0||pending>queue_limit)$fatal(1,"Queue overflow/underflow");
        end
    end
    reg arblocked=0,awblocked=0,wblocked=0,rblocked=0,bblocked=0;
    reg [77:0] prev_ar,prev_aw;
    reg [DATA_WIDTH+DATA_WIDTH/8:0] prev_w;
    reg [DATA_WIDTH+1:0] prev_r;reg prev_b;
    always @(posedge clk)begin
        if(!resetn)begin arblocked<=0;awblocked<=0;wblocked<=0;rblocked<=0;bblocked<=0;end
        else begin
            if(arblocked&&(!arvalid||{araddr,arlen,arsize,arburst,arid}!==prev_ar))$fatal(1,"AR changed under backpressure");
            if(awblocked&&(!awvalid||{awaddr,awlen,awsize,awburst,awid}!==prev_aw))$fatal(1,"AW changed under backpressure");
            if(wblocked&&(!wvalid||{wdata,wstrb,wlast}!==prev_w))$fatal(1,"W changed under backpressure");
            if(rblocked&&(!rvalid||{rdata,rlast,rid}!==prev_r))$fatal(1,"Memory R changed under backpressure");
            if(bblocked&&(!bvalid||bid!==prev_b))$fatal(1,"Memory B changed under backpressure");
            arblocked<=arvalid&&!arready;awblocked<=awvalid&&!awready;wblocked<=wvalid&&!wready;rblocked<=rvalid&&!rready;bblocked<=bvalid&&!bready;
            prev_ar<={araddr,arlen,arsize,arburst,arid};prev_aw<={awaddr,awlen,awsize,awburst,awid};prev_w<={wdata,wstrb,wlast};prev_r<={rdata,rlast,rid};prev_b<=bid;
        end
    end
endmodule
