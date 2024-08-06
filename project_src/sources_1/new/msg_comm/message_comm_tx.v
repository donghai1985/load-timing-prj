`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/18
// Design Name: 
// Module Name: message_comm_tx
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


module message_comm_tx #(
    parameter               DATA_WIDTH = 8 

)(
    // clk & rst
    input    wire           phy_rx_clk          ,
    input    wire           clk                 ,
    input    wire           rst_n               ,
    // ethernet interface for message data
    input    wire           rec_pkt_done_i      ,
    input    wire           rec_en_i            ,
    input    wire    [7:0]  rec_data_i          ,
    input    wire           rec_byte_num_en_i   ,
    input    wire    [15:0] rec_byte_num_i      ,
    output   wire           comm_ack_o          ,
    // info
    output   wire           MSG_CLK             ,
    output   wire           MSG_TX_FSX          ,
    output   wire           MSG_TX              
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
reg             [ 3-1:0]    state           = ST_IDLE;
reg             [ 3-1:0]    state_next      = ST_IDLE;

reg                         comm_tx_done    = 'd0;
reg             [16-1:0]    msg_tx_num      = 'd0;
reg             [ 4-1:0]    fifo_rd_cnt     = 'd0;
reg                         fifo_dout_vld   = 'd0;
reg             [ 8-1:0]    tx_data_temp    = 'd0;
reg                         tx_data_en      = 'd0;
reg             [ 8-1:0]    crc_new_data    = 'hff;
reg             [ 3-1:0]    tx_bit_cnt      = 'd0;

reg                         msg_data_rd_last = 'd0;
reg                         rec_byte_num_en_d0  = 'd0;
reg                         rec_byte_num_en_d1  = 'd0;
reg             [16-1:0]    rec_byte_num_d      = 'd0;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                        msg_tx_fifo_rd   ;
wire            [ 8-1:0]    msg_tx_fifo_dout ;
wire                        msg_tx_fifo_full ;
wire                        msg_tx_fifo_empty;
wire                        crc_vld ;

wire                        rec_byte_num_en_pose    ;
wire                        rec_byte_num_en_sync    ;
wire            [16-1:0]    rec_byte_num_sync       ;  
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
msg_comm_tx_fifo msg_comm_tx_fifo_inst(
    .rst       ( 'd0               ),
    .wr_clk    ( phy_rx_clk        ),
    .din       ( rec_data_i        ),
    .wr_en     ( rec_en_i          ),
    .rd_clk    ( clk               ),
    .rd_en     ( msg_tx_fifo_rd    ),
    .dout      ( msg_tx_fifo_dout  ),
    .full      ( msg_tx_fifo_full  ),
    .empty     ( msg_tx_fifo_empty )
);


always @(posedge phy_rx_clk) begin
    rec_byte_num_en_d0 <= rec_byte_num_en_i;
    rec_byte_num_en_d1 <= rec_byte_num_en_d0;
end
assign rec_byte_num_en_pose = rec_byte_num_en_d0 && ~rec_byte_num_en_d1;

always @(posedge phy_rx_clk) rec_byte_num_d <= rec_byte_num_i; 

handshake #(
    .DATA_WIDTH                 ( 16                    )
)handshake_inst(
    // clk & rst
    .src_clk_i                  ( phy_rx_clk            ),
    .src_rst_i                  ( 0                     ),
    .dest_clk_i                 ( clk                   ),
    .dest_rst_i                 ( ~rst_n                ),
    
    .src_data_i                 ( rec_byte_num_d        ),
    .src_vld_i                  ( rec_byte_num_en_pose  ),

    .dest_data_o                ( rec_byte_num_sync     ),
    .dest_vld_o                 ( rec_byte_num_en_sync  )
);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
assign msg_tx_fifo_rd   = (state==ST_TX) && (fifo_rd_cnt=='d0) && (|msg_tx_num); 
assign crc_vld          = state==ST_TX && msg_data_rd_last;


always @(posedge clk) begin
    if(~rst_n)
        state <= ST_IDLE;
    else 
        state <= state_next;
end

always @(*) begin
    state_next = state;
    case(state)
        ST_IDLE : 
            if(rec_byte_num_en_sync)
                state_next = ST_WAIT;
        ST_WAIT : 
            if(~msg_tx_fifo_empty)
                state_next = ST_TX;
        ST_TX :
            if(msg_data_rd_last)
                state_next = ST_CRC;
        ST_CRC : 
            if(comm_tx_done)
                state_next = ST_FINISH;
        ST_FINISH :
                state_next = ST_IDLE;
        default:
                state_next = ST_IDLE;
    endcase
end

always @(posedge clk) begin
    if(state==ST_IDLE && rec_byte_num_en_sync) 
        msg_tx_num <= rec_byte_num_sync;
    else if(state==ST_TX && msg_tx_fifo_rd)
        msg_tx_num <= msg_tx_num - 1;
end

always @(posedge clk) begin
    if(state == ST_TX)begin
        if(fifo_rd_cnt == 'd7)
            fifo_rd_cnt <= 'd0; 
        else 
            fifo_rd_cnt <= fifo_rd_cnt + 1;
    end
    else begin
        fifo_rd_cnt <= 'd0;
    end
end

always @(posedge clk) begin
    fifo_dout_vld <= msg_tx_fifo_rd;
end

always @(posedge clk) begin
    msg_data_rd_last <= (msg_tx_num=='d0) && (tx_bit_cnt=='d0) && (state==ST_TX);
end

always @(posedge clk) begin
    if(state==ST_IDLE)
        crc_new_data <= 'hff;
    else if(fifo_dout_vld)
        crc_new_data <= nextCRC8D8(msg_tx_fifo_dout,crc_new_data);
end


always @(posedge clk) begin
    comm_tx_done <= (state==ST_CRC) && (tx_bit_cnt=='d0);
end

always @(negedge clk) begin
    if(fifo_dout_vld || crc_vld)begin
        tx_bit_cnt   <= 'd7;
    end
    else begin
        tx_bit_cnt   <= tx_bit_cnt - 1;
    end
end

always @(negedge clk) begin
    if(fifo_dout_vld)
        tx_data_en <= 'd1;
    else if(comm_tx_done)
        tx_data_en <= 'd0;
end

always @(negedge clk) begin
    if(fifo_dout_vld)begin
        tx_data_temp <= msg_tx_fifo_dout;
    end
    else if(crc_vld)begin
        tx_data_temp <= crc_new_data;
    end
end

assign comm_ack_o   = rec_byte_num_en_pose;
assign MSG_CLK      = clk;
assign MSG_TX_FSX   = tx_data_en;
assign MSG_TX       = tx_data_temp[tx_bit_cnt];


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
