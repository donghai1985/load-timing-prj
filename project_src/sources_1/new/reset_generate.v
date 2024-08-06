`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/22
// Design Name: 
// Module Name: reset_generate
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module reset_generate(
    input       nrst_i      ,

    input       clk_0_i     ,
    output      nrst_0_o    ,
    input       clk_1_i     ,
    output      nrst_1_o    ,
    input       clk_2_i     ,
    output      nrst_2_o    ,
    input       clk_3_i     ,
    output      nrst_3_o    ,

    output      debug_info  
);

reg nrst_0_d0 = 'd0;
reg nrst_0_d1 = 'd0;
always @(posedge clk_0_i) begin
    nrst_0_d0 <= nrst_i;
    nrst_0_d1 <= nrst_0_d0;
end

reg nrst_1_d0 = 'd0;
reg nrst_1_d1 = 'd0;
always @(posedge clk_1_i) begin
    nrst_1_d0 <= nrst_i;
    nrst_1_d1 <= nrst_1_d0;
end


reg nrst_2_d0;
reg nrst_2_d1;
always @(posedge clk_2_i) begin
    nrst_2_d0 <= nrst_i ;
    nrst_2_d1 <= nrst_2_d0;
end

reg nrst_3_d0;
reg nrst_3_d1;
always @(posedge clk_3_i) begin
    nrst_3_d0 <= nrst_i ;
    nrst_3_d1 <= nrst_3_d0;
end

assign nrst_0_o = nrst_0_d1;
assign nrst_1_o = nrst_1_d1;
assign nrst_2_o = nrst_2_d1;
assign nrst_3_o = nrst_3_d1;

endmodule