`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/23
// Design Name: 
// Module Name: udp_tx_control
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


module udp_tx_control #(
    parameter   [8*20-1:0]  SFPGA_VERSION   = "PCG1_TimingS_v2.2   ",
    parameter               MSG_SOURCE      = 16'h0001  

)(
    // clk & rst
    input    wire           clk_i               ,
    input    wire           rst_i               ,
    input    wire           phy_clk_i           ,
    // cfg and comm info
    input    wire           rd_sfpga_version_i  ,
    input    wire           sfpga_comm_reset_i  ,
    input    wire           cfg_load_loss_i     ,
    input    wire  [8:0]    cfg_load_status_i   ,
    input    wire           comm_ack_i          ,
    // slave tx data,to main PC
    input    wire           rd_data_vld_i       ,
    input    wire  [7:0]    rd_data_i           ,
    // udp tx info
    output   wire           tx_start_en_o       ,
    output   wire  [15:0]   tx_byte_num_o       ,
    input    wire           udp_tx_done_i       ,
    input    wire           tx_req_i            ,
    output   wire  [7:0]    tx_data_o           
);


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
genvar  i;
localparam                  ARBITR_NUM          = 'd5;
// localparam  [8*20-1:0]      SFPGA_VERSION       = "PCG1_TimingS_v2.2   ";

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg    [16-1:0]             udp_data_cnt        = 'd0;
reg    [ARBITR_NUM-1:0]     udp_tx_type         = 'd0;

reg                         r_tx_start_en       = 'd0;
reg    [ARBITR_NUM-1:0]     arbitrate           = 'd1;
reg                         arbitr_result_d0    = 'd0;
reg                         arbitr_result_d1    = 'd0;

reg                         rd_sfpga_version_d0 = 'd0;
reg                         rd_sfpga_version_d1 = 'd0;
reg    [8*20-1:0]           sfpga_version_r     = 'd0;

reg                         load_loss_sync      = 'd0;
reg                         comm_ack_sync       = 'd0;
reg    [ 9-1:0]             load_status_sync    = 'd0;

reg    [16-1:0]             tx_byte_num         = 'd0;
reg                         slave_tx_sync       = 'd0;
reg    [16-1:0]             slave_tx_cnt        = 'd0;
reg    [16-1:0]             slave_tx_cnt_latch  = 'd0;

(*mark_debug = "true"*)reg                         sfpga_comm_clear        = 'd0;
reg                         sfpga_comm_clear_wait   = 'd0;
reg                         sfpga_comm_reset_d0     = 'd0;
reg                         sfpga_comm_reset_d1     = 'd0;

reg                         slave_tx_rd_en      = 'd0;
reg                         tx_instruct_rd      = 'd0;
reg                         tx_arbitr_wait      = 'd0;
reg                         tx_arbitr_wait_d    = 'd0;
reg                         tx_arbitr_sync      = 'd0;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire   [ARBITR_NUM-1:0]     udp_type_sync       ;
wire                        tx_start_en_delay   ;
(*mark_debug = "true"*)wire   [ARBITR_NUM-1:0]     arbitr_result       ;
wire   [ 8-1:0]             slave_tx_dout       ;
wire                        slave_tx_full       ;
(*mark_debug = "true"*)wire                        slave_tx_empty      ;
wire                        sfpga_version_nege  ;
wire                        sfpga_comm_reset_nege  ;
wire                        sfpga_comm_reset_pose  ;
(*mark_debug = "true"*)wire                        rd_data_vld_noclear ;

wire   [16-1:0]             tx_instruct_dout ;
wire                        tx_instruct_full ;
(*mark_debug = "true"*)wire                        tx_instruct_empty;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
udp_tx_fifo udp_tx_fifo_inst(
    .clk                        ( phy_clk_i         ),
    .srst                       ( 'd0               ),
    .din                        ( rd_data_i         ),
    .wr_en                      ( rd_data_vld_noclear),
    .rd_en                      ( slave_tx_rd_en || sfpga_comm_clear ),
    .dout                       ( slave_tx_dout     ),
    .full                       ( slave_tx_full     ),
    .empty                      ( slave_tx_empty    )
);

tx_instruct_fifo tx_instruct_fifo_inst(
    .clk                        ( phy_clk_i         ),
    .srst                       ( 'd0               ),
    .din                        ( slave_tx_cnt      ),
    .wr_en                      ( slave_tx_sync     ),
    .rd_en                      ( tx_instruct_rd || sfpga_comm_clear ),
    .dout                       ( tx_instruct_dout  ),
    .full                       ( tx_instruct_full  ),
    .empty                      ( tx_instruct_empty )
);

// debug code
reg [11-1:0] tx_fifo_cnt = 'd0;
always @(posedge phy_clk_i) begin
    if({rd_data_vld_noclear,slave_tx_rd_en} == 2'b10 && ~slave_tx_full)begin
        tx_fifo_cnt <= tx_fifo_cnt + 1;
    end
    else if({rd_data_vld_noclear,slave_tx_rd_en} == 2'b01 && ~slave_tx_empty)begin
        tx_fifo_cnt <= tx_fifo_cnt - 1;
    end
end
// reg_delay #(
//     .DATA_WIDTH                 ( 1                 ),
//     .DELAY_NUM                  ( 5                 )
// )start_en_delay_inst(
//     // clk & rst
//     .clk_i                      ( phy_clk_i         ),
//     .src_data_i                 ( |udp_type_sync    ),
//     .delay_data_o               ( tx_start_en_delay )     
// );
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// slave fpga to main PC
always @(posedge phy_clk_i) begin
    if(rd_data_vld_noclear)
        slave_tx_cnt <= slave_tx_cnt + 1;
    else if(slave_tx_sync)
        slave_tx_cnt <= 'd0;
end

reg rd_data_vld_d0 ;
always @(posedge phy_clk_i) begin
    rd_data_vld_d0 <= rd_data_vld_noclear;
    slave_tx_sync  <= ~rd_data_vld_noclear && rd_data_vld_d0;
end

// instruct fifo read
always @(posedge phy_clk_i) begin
    if(sfpga_comm_clear)
        tx_arbitr_wait <= 'd0;
    else if(~tx_instruct_empty && (~tx_arbitr_wait) && (~sfpga_comm_clear_wait))
        tx_arbitr_wait <= 'd1;
    else if(udp_tx_done_i && arbitr_result[3])
        tx_arbitr_wait <= 'd0;
end

always @(posedge phy_clk_i) tx_instruct_rd      <= ~tx_instruct_empty && (~tx_arbitr_wait) && (~sfpga_comm_clear_wait);
always @(posedge phy_clk_i) tx_arbitr_wait_d    <= tx_arbitr_wait;
always @(posedge phy_clk_i) begin
    if(sfpga_comm_clear)
        tx_arbitr_sync      <= 'd0;
    else 
        tx_arbitr_sync      <= ~tx_arbitr_wait_d && tx_arbitr_wait;
end 

always @(posedge phy_clk_i) begin
    if(tx_arbitr_sync)
        slave_tx_cnt_latch <= tx_instruct_dout;
end

always @(posedge phy_clk_i) begin
    load_loss_sync    <= cfg_load_loss_i    ;         
    comm_ack_sync     <= comm_ack_i         ;
    if(cfg_load_status_i[8])begin
        load_status_sync[8:0]  <= cfg_load_status_i[8:0];
    end
    else begin
        load_status_sync[8] <= 1'b0;
    end
end

assign udp_type_sync     = {sfpga_version_nege,tx_arbitr_sync,load_loss_sync,comm_ack_sync,load_status_sync[8]};
assign arbitr_result     = udp_tx_type & arbitrate;
assign tx_start_en_delay = ~arbitr_result_d1 && arbitr_result_d0;

always @(posedge phy_clk_i) begin
    arbitr_result_d0 <= |arbitr_result  ;
    arbitr_result_d1 <= arbitr_result_d0;
end

// arbitr trigger
generate
    for(i=0;i<ARBITR_NUM;i=i+1)begin
        always @(posedge phy_clk_i) begin
            if(udp_type_sync[i])
                udp_tx_type[i] <= 'd1;
            else if(udp_tx_done_i && arbitr_result[i])
                udp_tx_type[i] <= 'd0;
        end
    end
endgenerate

// check arbitr
always @(posedge phy_clk_i) begin
    if(arbitr_result=='d0)
        arbitrate <= {arbitrate[ARBITR_NUM-2:0],arbitrate[ARBITR_NUM-1]};
    else 
        arbitrate <= arbitrate;
end

// udp tx 
always @(posedge phy_clk_i) begin
    if(|arbitr_result)begin
        if(tx_req_i)
            udp_data_cnt <= udp_data_cnt + 1;
    end
    else begin
        udp_data_cnt <= 'd0;
    end
end

always @(posedge phy_clk_i) begin
    if(arbitr_result[3])
        slave_tx_rd_en <= (udp_data_cnt>'d0) && (udp_data_cnt<=slave_tx_cnt_latch) && tx_req_i;
end

// read sfpga version
always @(posedge phy_clk_i) begin
    rd_sfpga_version_d0 <= rd_sfpga_version_i;
    rd_sfpga_version_d1 <= rd_sfpga_version_d0;
end
assign sfpga_version_nege = ~rd_sfpga_version_d0 && rd_sfpga_version_d1;

always @(posedge phy_clk_i) begin
    if(arbitr_result[4] && tx_req_i)begin
        if(udp_data_cnt > 'd3)
            sfpga_version_r <= {sfpga_version_r[19*8-1:0],8'd0};
    end
    else begin
        sfpga_version_r <= SFPGA_VERSION;
    end
end


// sfpga comm clear
always @(posedge phy_clk_i) begin
    sfpga_comm_reset_d0 <= sfpga_comm_reset_i;
    sfpga_comm_reset_d1 <= sfpga_comm_reset_d0;
end
assign sfpga_comm_reset_nege = ~sfpga_comm_reset_d0 && sfpga_comm_reset_d1;
// assign sfpga_comm_reset_pose = sfpga_comm_reset_d0 && (~sfpga_comm_reset_d1);

always @(posedge phy_clk_i) begin
    if(sfpga_comm_reset_nege)
        sfpga_comm_clear_wait <= 'd1;
    else if(~arbitr_result[3])
        sfpga_comm_clear_wait <= 'd0;
end

reg [12-1:0] sfpga_comm_clear_cnt = 'd0;
always @(posedge phy_clk_i) begin
    if(sfpga_comm_clear_wait && (~arbitr_result[3]))
        sfpga_comm_clear <= 'd1;
    else if(sfpga_comm_clear_cnt[11] && (~rd_data_vld_i))
        sfpga_comm_clear <= 'd0;
end

always @(posedge phy_clk_i) begin
    if(sfpga_comm_clear)
        sfpga_comm_clear_cnt <= sfpga_comm_clear_cnt[11] ? sfpga_comm_clear_cnt : sfpga_comm_clear_cnt + 1;
    else
        sfpga_comm_clear_cnt <= 'd0;
end

assign rd_data_vld_noclear = ~sfpga_comm_clear && rd_data_vld_i;

reg [ 8-1:0] rd_data_r = 'd0;
always @(*) begin
    case(arbitr_result)
        'b00001:begin
            if(udp_data_cnt=='d0)begin
                rd_data_r = MSG_SOURCE[15:8];       // msg source
            end
            else if(udp_data_cnt=='d1)begin
                rd_data_r = MSG_SOURCE[7:0];       // msg source
            end
            else if(udp_data_cnt=='d2)begin
                rd_data_r = 'h00;             // msg type
            end
            else if(udp_data_cnt=='d3)begin
                rd_data_r = 'h02;             // msg type  load_status
            end
            else if(udp_data_cnt=='d4)begin
                rd_data_r = load_status_sync[7:0];   // load_status msg data
            end
            else begin
                rd_data_r = 'd0;
            end
        end
        'b00010:begin
            if(udp_data_cnt=='d0)begin
                rd_data_r = MSG_SOURCE[15:8];       // msg source
            end
            else if(udp_data_cnt=='d1)begin
                rd_data_r = MSG_SOURCE[7:0];       // msg source
            end
            else if(udp_data_cnt=='d2)begin
                rd_data_r = 'h00;             // msg type
            end
            else if(udp_data_cnt=='d3)begin
                rd_data_r = 'h03;             // msg type  comm_ack
            end
            else begin
                rd_data_r = 'd0;              // msg data
            end
        end
        'b00100:begin
            if(udp_data_cnt=='d0)begin
                rd_data_r = MSG_SOURCE[15:8];       // msg source
            end
            else if(udp_data_cnt=='d1)begin
                rd_data_r = MSG_SOURCE[7:0];       // msg source
            end
            else if(udp_data_cnt=='d2)begin
                rd_data_r = 'h00;             // msg type
            end
            else if(udp_data_cnt=='d3)begin
                rd_data_r = 'h01;             // msg type  load_loss 
            end
            else begin
                rd_data_r = 'd0;              // msg data
            end
        end
        'b01000:begin
            if(udp_data_cnt=='d0)begin
                rd_data_r = MSG_SOURCE[15:8];       // msg source
            end
            else if(udp_data_cnt=='d1)begin
                rd_data_r = MSG_SOURCE[7:0];       // msg source
            end
            else begin
                rd_data_r = slave_tx_dout;         // msg data,include msg type
            end
        end
        'b10000:begin
            if(udp_data_cnt=='d0)begin
                rd_data_r = MSG_SOURCE[15:8];       // msg source
            end
            else if(udp_data_cnt=='d1)begin
                rd_data_r = MSG_SOURCE[7:0];       // msg source
            end
            else if(udp_data_cnt=='d2)begin
                rd_data_r = 'h00;             // msg type
            end
            else if(udp_data_cnt=='d3)begin
                rd_data_r = 'h04;             // msg type  sfpga version 
            end
            else begin
                rd_data_r = sfpga_version_r[19*8 +: 8];         // msg data
            end
        end
            default:/*default*/;
    endcase
end

always @(posedge phy_clk_i) begin
    if(arbitr_result[3])
        tx_byte_num <= slave_tx_cnt_latch + 'd2;  // 2 for head byte
    else if(arbitr_result[4])
        tx_byte_num <= 'd24;
    else 
        tx_byte_num <= 'd20;
end

assign tx_data_o     = rd_data_r;
assign tx_start_en_o = tx_start_en_delay;
assign tx_byte_num_o = tx_byte_num;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

endmodule
