`timescale 1ns/1ps
module reset_tb;
reg clk=0;always #5 clk=~clk;
reg ps_resetn=0;
wire fixed_aresetn,old_aresetn;
// Same generated IP parameters as the integrated block design.
full_board_reset_0 corrected(.slowest_sync_clk(clk),.ext_reset_in(ps_resetn),
    .aux_reset_in(ps_resetn),.mb_debug_sys_rst(1'b0),.dcm_locked(1'b1),
    .mb_reset(),.bus_struct_reset(),.peripheral_reset(),.interconnect_aresetn(),.peripheral_aresetn(fixed_aresetn));
full_board_reset_0 previous_wiring(.slowest_sync_clk(clk),.ext_reset_in(ps_resetn),
    .aux_reset_in(1'b0),.mb_debug_sys_rst(1'b0),.dcm_locked(1'b1),
    .mb_reset(),.bus_struct_reset(),.peripheral_reset(),.interconnect_aresetn(),.peripheral_aresetn(old_aresetn));
initial begin
    repeat(200)@(negedge clk);
    if(fixed_aresetn!==0||old_aresetn!==0)$fatal(1,"Initial reset not asserted");
    ps_resetn=1;repeat(200)@(negedge clk);
    if(fixed_aresetn!==1)$fatal(1,"Corrected peripheral reset did not release");
    if(old_aresetn!==0)$fatal(1,"Previous active-low auxiliary tie should reproduce held reset");
    ps_resetn=0;repeat(40)@(negedge clk);
    if(fixed_aresetn!==0)$fatal(1,"Reset reload did not assert");
    ps_resetn=1;repeat(200)@(negedge clk);
    if(fixed_aresetn!==1)$fatal(1,"Reset reload did not release");
    $display("RESET_INTEGRATION_PASS initial_assert=1 release=1 reload=1 previous_wiring_held_reset=1");
    $finish;
end
endmodule
