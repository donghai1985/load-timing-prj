`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/19
// Design Name: 
// Module Name: message_comm_rx
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


module message_comm_x4_rx #(
    parameter               DATA_WIDTH = 8 

)(
    // clk & rst
    input    wire           clk                 ,
    input    wire           rst_n               ,
    // ethernet interface for message data
    output   wire           msg_rx_data_vld_o   ,
    output   wire    [7:0]  msg_rx_data_o       ,
    // comm info
    input    wire           MSG_CLK             ,
    input    wire           MSG_RX_FSX          ,
    input    wire           MSG_RX0             ,
    input    wire           MSG_RX1             ,
    input    wire           MSG_RX2             ,
    input    wire           MSG_RX3              
);


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam                  ST_IDLE     = 3'd0;
localparam                  ST_WAIT     = 3'd1;
localparam                  ST_TX       = 3'd2;
localparam                  ST_CRC      = 3'd3;
localparam                  ST_FINISH   = 3'd4;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg                         rx_ram_wr_en            = 'd0;
reg             [11-1:0]    msg_rx_ram_raddr        = 'd0;

reg             [ 1-1:0]    comm_rx_cnt             = 'd0;
reg             [ 16-1:0]   comm_rx_data_temp       = 'd0;
reg             [ 8-1:0]    crc_new_data            = 'hff;
reg             [16-1:0]    comm_rx_data_cnt        = 'd0;
reg                         crc_check_result        = 'd0;
reg                         comm_rx_last_ff         = 'd0;
reg             [16-1:0]    comm_rx_data_num_sync   = 'd0;
reg                         st_read_ram             = 'd0;
reg                         comm_rx_data_vld        = 'd0;
reg                         comm_rx_last_sync       = 'd0;
reg                         msg_rx_fsx_d0       ;
reg                         msg_rx_fsx_d1       ;
reg                         comm_rx_last_d      ;
reg                         comm_rx_last_ff_d0  ;
reg                         comm_rx_last_ff_d1  ;
reg                         comm_rx_last_ff_d2  ;
reg                         comm_rx_last_ff_d3  ;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire            [ 8-1:0]    msg_rx_ram_dout ;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
msg_comm_rx_ram msg_comm_rx_ram_inst(
    .a          ( comm_rx_data_cnt[10:0] ),
    .d          ( comm_rx_data_temp[7:0]),
    .dpra       ( msg_rx_ram_raddr      ),
    .clk        ( MSG_CLK               ),
    .we         ( rx_ram_wr_en          ),
    .qdpo_clk   ( clk                   ),
    .qdpo       ( msg_rx_ram_dout       )
);


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
always @(posedge MSG_CLK) begin
    if(MSG_RX_FSX)
        comm_rx_data_temp <= {comm_rx_data_temp[11:0],MSG_RX3,MSG_RX2,MSG_RX1,MSG_RX0};
end

always @(posedge MSG_CLK) begin
    if(MSG_RX_FSX)
        comm_rx_cnt <= comm_rx_cnt + 1;
    else 
        comm_rx_cnt <= 'd0;
end

always @(posedge MSG_CLK) begin
    rx_ram_wr_en <= MSG_RX_FSX && (&comm_rx_cnt);
end 

always @(posedge MSG_CLK) msg_rx_fsx_d0 <= MSG_RX_FSX;
always @(posedge MSG_CLK) msg_rx_fsx_d1 <= msg_rx_fsx_d0;

wire comm_rx_last ;
assign comm_rx_last = msg_rx_fsx_d1 && ~msg_rx_fsx_d0;

always @(posedge MSG_CLK) begin
    if(rx_ram_wr_en)
        comm_rx_data_cnt <= comm_rx_data_cnt + 1;
    else if(comm_rx_last)
        comm_rx_data_cnt <= 'd0;
end

(*mark_debug = "true"*)reg [16-1:0] comm_rx_data_num = 'd0;
always @(posedge MSG_CLK) begin
    if(comm_rx_last)
        comm_rx_data_num <= comm_rx_data_cnt;
end

// crc check
always @(posedge MSG_CLK) begin
    if(comm_rx_last)
        crc_new_data <= 'hff;
    else if(rx_ram_wr_en && |comm_rx_data_cnt)
        crc_new_data <= nextCRC8D8(comm_rx_data_temp[15:8],crc_new_data);
end

always @(posedge MSG_CLK) begin
    if(comm_rx_last)begin
        if(crc_new_data != comm_rx_data_temp[7:0])
            crc_check_result <= 'd1; 
    end
    else 
        crc_check_result <= 'd0; 
end

// cross clock domain
always @(posedge MSG_CLK) comm_rx_last_d = comm_rx_last;

(*mark_debug = "true"*)wire rx_sync_result;
assign rx_sync_result = comm_rx_last_d && (~crc_check_result);

(*mark_debug = "true"*)wire rx_start_sync;
(*mark_debug = "true"*)wire [16-1:0] rx_start_num_sync ;
handshake #(
    .DATA_WIDTH                 ( 16                    )
)handshake_inst(
    // clk & rst
    .src_clk_i                  ( MSG_CLK               ),
    .src_rst_i                  ( 0                     ),
    .dest_clk_i                 ( clk                   ),
    .dest_rst_i                 ( 0                     ),
    
    .src_data_i                 ( comm_rx_data_num      ),
    .src_vld_i                  ( rx_sync_result        ),

    .dest_data_o                ( rx_start_num_sync     ),
    .dest_vld_o                 ( rx_start_sync         )
);

// read message data
always @(posedge clk) begin
    if(rx_start_sync)
        st_read_ram <= 'd1;
    else if(msg_rx_ram_raddr==rx_start_num_sync - 2)
        st_read_ram <= 'd0;
end

always @(posedge clk) begin
    if(st_read_ram)
        msg_rx_ram_raddr <= msg_rx_ram_raddr + 1;
    else
        msg_rx_ram_raddr <= 'd0;
end

always @(posedge clk) begin
    comm_rx_data_vld <= st_read_ram;
end


assign msg_rx_data_vld_o = comm_rx_data_vld;
assign msg_rx_data_o     = msg_rx_ram_dout;

// crc function
// polynomial: x^8 + x^2 + x + 1
// data width: 8
// convention: the first serial bit is D[7]
function[7:0]nextCRC8D8;
    input[7:0]Data; 
    input[7:0]crc; 
    reg [7:0] d; 
    reg [7:0]c;
    reg [7:0] newcrc; 
    begin 
        d = Data; 
        c = crc;
        newcrc[0]  = d[7] ^ d[6] ^ d[0] ^ c[0] ^ c[6] ^ c[7];
        newcrc[1]  = d[6] ^ d[1] ^ d[0] ^ c[0] ^ c[1] ^ c[6];
        newcrc[2]  = d[6] ^ d[2] ^ d[1] ^ d[0] ^ c[0] ^ c[1] ^ c[2] ^ c[6]; 
        newcrc[3]  = d[7] ^ d[3] ^ d[2] ^ d[1] ^ c[1] ^ c[2] ^ c[3] ^ c[7]; 
        newcrc[4]  = d[4] ^ d[3] ^ d[2] ^ c[2] ^ c[3] ^ c[4]; 
        newcrc[5]  = d[5] ^ d[4] ^ d[3] ^ c[3] ^ c[4] ^ c[5]; 
        newcrc[6]  = d[6] ^ d[5] ^ d[4] ^ c[4] ^ c[5] ^ c[6]; 
        newcrc[7]  = d[7] ^ d[6] ^ d[5] ^ c[5] ^ c[6] ^ c[7]; 
        nextCRC8D8 = newcrc ;
    end
endfunction 
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

endmodule
