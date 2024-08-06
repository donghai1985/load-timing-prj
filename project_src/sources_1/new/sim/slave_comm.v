`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/29
// Design Name: 
// Module Name: slave_comm
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


module slave_comm(
    // clk & rst
    input    wire           clk_sys_i               ,
    input    wire           rst_i                   ,
    // ethernet interface for message data
    input    wire           slave_tx_en_i           ,
    input    wire    [7:0]  slave_tx_data_i         ,
    input    wire           slave_tx_byte_num_en_i  ,
    input    wire    [15:0] slave_tx_byte_num_i     ,
    output   wire           slave_tx_ack_o          ,
    // message rx info
    output   wire           rd_data_vld_o           ,
    output   wire    [7:0]  rd_data_o               ,
    // info
    input    wire           SLAVE_MSG_CLK           ,
    output   wire           SLAVE_MSG_TX_FSX        ,
    output   wire           SLAVE_MSG_TX0           ,
    output   wire           SLAVE_MSG_TX1           ,
    output   wire           SLAVE_MSG_TX2           ,
    output   wire           SLAVE_MSG_TX3           ,
    input    wire           SLAVE_MSG_RX_FSX        ,
    input    wire           SLAVE_MSG_RX            
);

slave_comm_x4_tx slave_comm_tx_inst(
    .clk_sys_i                  ( clk_sys_i                 ),
    .rst_i                      ( rst_i                     ),
    .slave_tx_en_i              ( slave_tx_en_i             ),
    .slave_tx_data_i            ( slave_tx_data_i           ),
    .slave_tx_byte_num_en_i     ( slave_tx_byte_num_en_i    ),
    .slave_tx_byte_num_i        ( slave_tx_byte_num_i       ),
    .slave_tx_ack_o             ( slave_tx_ack_o            ),

    .SLAVE_MSG_CLK              ( SLAVE_MSG_CLK             ),
    .SLAVE_MSG_TX_FSX           ( SLAVE_MSG_TX_FSX          ),
    .SLAVE_MSG_TX0              ( SLAVE_MSG_TX0             ),
    .SLAVE_MSG_TX1              ( SLAVE_MSG_TX1             ),
    .SLAVE_MSG_TX2              ( SLAVE_MSG_TX2             ),
    .SLAVE_MSG_TX3              ( SLAVE_MSG_TX3             )
);

slave_comm_rx slave_comm_rx_inst(
    .clk_sys_i                  ( clk_sys_i                 ),
    .slave_rx_data_vld_o        ( rd_data_vld_o             ),
    .slave_rx_data_o            ( rd_data_o                 ),
    .SLAVE_MSG_CLK              ( SLAVE_MSG_CLK             ),
    .SLAVE_MSG_RX_FSX           ( SLAVE_MSG_RX_FSX          ),
    .SLAVE_MSG_RX               ( SLAVE_MSG_RX              )
);

endmodule
