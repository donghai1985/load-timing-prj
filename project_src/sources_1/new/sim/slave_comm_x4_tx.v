`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: zas
// Engineer: syx
// 
// Create Date: 2024/06/3
// Design Name: pcg
// Module Name: slave_comm_x4_tx
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


module slave_comm_x4_tx #(
    parameter               DATA_WIDTH = 8 

)(
    // clk & rst
    input    wire           clk_sys_i               ,
    input    wire           rst_i                   ,
    // ethernet interface for message data
    input    wire           slave_tx_en_i           ,
    input    wire    [7:0]  slave_tx_data_i         ,
    input    wire           slave_tx_byte_num_en_i  ,
    input    wire    [15:0] slave_tx_byte_num_i     ,
    output   wire           slave_tx_ack_o          ,
    // info
    input    wire           SLAVE_MSG_CLK           ,
    output   wire           SLAVE_MSG_TX_FSX        ,
    output   wire           SLAVE_MSG_TX0           ,
    output   wire           SLAVE_MSG_TX1           ,
    output   wire           SLAVE_MSG_TX2           ,
    output   wire           SLAVE_MSG_TX3           
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
reg             [ 3-1:0]    state                       = ST_IDLE;
reg             [ 3-1:0]    state_next                  = ST_IDLE;

reg                         comm_tx_done                = 'd0;
reg             [16-1:0]    msg_tx_num                  = 'd0;
reg             [ 1-1:0]    fifo_rd_cnt                 = 'd0;
reg                         fifo_dout_vld               = 'd0;
reg             [ 8-1:0]    tx_data_temp                = 'd0;
reg                         tx_data_en                  = 'd0;
reg             [ 8-1:0]    crc_new_data                = 'hff;
reg             [ 1-1:0]    tx_bit_cnt                  = 'd0;

reg                         msg_data_rd_last            = 'd0;
reg                         slave_tx_byte_num_en_sync   = 'd0;
reg             [15:0]      slave_tx_byte_num_sync      = 'd0;  
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                        msg_tx_fifo_rd   ;
wire            [ 8-1:0]    msg_tx_fifo_dout ;
wire                        msg_tx_fifo_full ;
wire                        msg_tx_fifo_empty;
wire                        crc_vld ;

wire                        slave_tx_ack;
wire                        slave_tx_ack_sync;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
msg_comm_tx_fifo msg_comm_tx_fifo_inst(
    .rst       ( 1'b0              ),
    .wr_clk    ( clk_sys_i         ),
    .din       ( slave_tx_data_i   ),
    .wr_en     ( slave_tx_en_i     ),
    .rd_clk    ( SLAVE_MSG_CLK     ),
    .rd_en     ( msg_tx_fifo_rd    ),
    .dout      ( msg_tx_fifo_dout  ),
    .full      ( msg_tx_fifo_full  ),
    .empty     ( msg_tx_fifo_empty )
);

reg handshake_en_sync_d0;
reg handshake_en_sync_d1;
reg handshake_en_sync_d2;
reg handshake_en_sync_d3;

reg [16-1:0] slave_tx_byte_num_ff;
always @(posedge clk_sys_i) begin
    if(slave_tx_byte_num_en_i)
        slave_tx_byte_num_ff <= slave_tx_byte_num_i;
end

reg handshake_en = 'd0;
always @(posedge clk_sys_i ) begin
    if(slave_tx_byte_num_en_i)
        handshake_en <= 'd1;
    else if(handshake_en_sync_d2 && ~handshake_en_sync_d3)
        handshake_en <= 'd0;
end

always @(posedge SLAVE_MSG_CLK) handshake_en_sync_d0 <= handshake_en;
always @(posedge SLAVE_MSG_CLK) handshake_en_sync_d1 <= handshake_en_sync_d0;
always @(posedge clk_sys_i)     handshake_en_sync_d2 <= handshake_en_sync_d1;
always @(posedge clk_sys_i)     handshake_en_sync_d3 <= handshake_en_sync_d2;

always @(posedge SLAVE_MSG_CLK ) begin
    if(handshake_en_sync_d0 && ~handshake_en_sync_d1)
        slave_tx_byte_num_sync <= slave_tx_byte_num_ff;
end

always @(posedge SLAVE_MSG_CLK ) slave_tx_byte_num_en_sync <= handshake_en_sync_d0 && ~handshake_en_sync_d1;


xpm_cdc_pulse #(
   .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
   .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
   .REG_OUTPUT(1),     // DECIMAL; 0=disable registered output, 1=enable registered output
   .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
   .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_inst (
   .dest_pulse(slave_tx_ack_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                            // transfer is correctly initiated on src_pulse input. This output is
                            // combinatorial unless REG_OUTPUT is set to 1.

   .dest_clk(clk_sys_i),     // 1-bit input: Destination clock.
   .dest_rst('d0),     // 1-bit input: optional; required when RST_USED = 1
   .src_clk(SLAVE_MSG_CLK),       // 1-bit input: Source clock.
   .src_pulse(slave_tx_ack),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
                            // destination clock domain. The minimum gap between each pulse transfer must be
                            // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
                            // between the falling edge of a src_pulse to the rising edge of the next
                            // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
                            // will generate a pulse the size of one dest_clk period in the destination
                            // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
                            // src_rst and/or dest_rst are asserted.

   .src_rst('d0)        // 1-bit input: optional; required when RST_USED = 1
);
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
assign msg_tx_fifo_rd   = (state==ST_TX) && (fifo_rd_cnt=='d0) && (|msg_tx_num); 
assign crc_vld          = state==ST_TX && msg_data_rd_last;
assign slave_tx_ack     = state==ST_FINISH;

always @(posedge SLAVE_MSG_CLK) begin
    if(rst_i)
        state <= ST_IDLE;
    else 
        state <= state_next;
end

always @(*) begin
    state_next = state;
    case(state)
        ST_IDLE : 
            if(slave_tx_byte_num_en_sync)
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

always @(posedge SLAVE_MSG_CLK) begin
    if(state==ST_IDLE && slave_tx_byte_num_en_sync) 
        msg_tx_num <= slave_tx_byte_num_sync;
    else if(state==ST_TX && msg_tx_fifo_rd)
        msg_tx_num <= msg_tx_num - 1;
end

always @(posedge SLAVE_MSG_CLK) begin
    if(state == ST_TX)begin
        if(fifo_rd_cnt == 'd1)
            fifo_rd_cnt <= 'd0; 
        else 
            fifo_rd_cnt <= fifo_rd_cnt + 1;
    end
    else begin
        fifo_rd_cnt <= 'd0;
    end
end

always @(posedge SLAVE_MSG_CLK) begin
    fifo_dout_vld <= msg_tx_fifo_rd;
end

always @(posedge SLAVE_MSG_CLK) begin
    msg_data_rd_last <= (msg_tx_num=='d0) && (tx_bit_cnt=='d0) && (state==ST_TX);
end

always @(posedge SLAVE_MSG_CLK) begin
    if(state==ST_IDLE)
        crc_new_data <= 'hff;
    else if(fifo_dout_vld)
        crc_new_data <= nextCRC8D8(msg_tx_fifo_dout,crc_new_data);
end


always @(posedge SLAVE_MSG_CLK) begin
    comm_tx_done <= (state==ST_CRC) && (tx_bit_cnt=='d0);
end

always @(negedge SLAVE_MSG_CLK) begin
    if(fifo_dout_vld || crc_vld)begin
        tx_bit_cnt   <= 'd1;
    end
    else begin
        tx_bit_cnt   <= tx_bit_cnt - 1;
    end
end

always @(negedge SLAVE_MSG_CLK) begin
    if(fifo_dout_vld)
        tx_data_en <= 'd1;
    else if(comm_tx_done)
        tx_data_en <= 'd0;
end

always @(negedge SLAVE_MSG_CLK) begin
    if(fifo_dout_vld)begin
        tx_data_temp <= msg_tx_fifo_dout;
    end
    else if(crc_vld)begin
        tx_data_temp <= crc_new_data;
    end
end

assign slave_tx_ack_o   = slave_tx_ack_sync;
assign SLAVE_MSG_TX_FSX = tx_data_en;
assign SLAVE_MSG_TX0    = tx_data_temp[(tx_bit_cnt*4)+0];
assign SLAVE_MSG_TX1    = tx_data_temp[(tx_bit_cnt*4)+1];
assign SLAVE_MSG_TX2    = tx_data_temp[(tx_bit_cnt*4)+2];
assign SLAVE_MSG_TX3    = tx_data_temp[(tx_bit_cnt*4)+3];


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
