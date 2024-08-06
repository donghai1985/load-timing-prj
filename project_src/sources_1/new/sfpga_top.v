`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/24 08:15:24
// Design Name: 
// Module Name: sfpga_top
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


module sfpga_top(
    //clk
    input   wire        S_FPGA_CLK              , 
    //ethernet phy
    output  wire        SFPGA_ETH_PWR_EN        , 
    output  wire        SFPGA_RGMII_TX_CLK      , 
    output  wire [3:0]  SFPGA_RGMII_TXD         ,
    output  wire        SFPGA_RGMII_TX_CTL      ,
    input   wire        SFPGA_RGMII_RX_CLK      , 
    input   wire [3:0]  SFPGA_RGMII_RXD         ,
    input   wire        SFPGA_RGMII_RX_CTL      ,
    output  wire        SFPGA_RGMII_MDC         , 
    inout   wire        SFPGA_RGMII_MDIO        , 
    output  wire        SFPGA_ETH_RESET_B       , 
    input   wire        SFPGA_ETH_INTB          , 
    //config fpga
    output  wire [15:0] FPGA_CONFIG_D           , 
    output  wire        FPGA_CONFIG_RDWR_B      , 
    output  wire        FPGA_CONFIG_CSI_B       , 
    input   wire        FPGA_INIT_B_SFPGA       , 
    output  wire        FPGA_PROG_B_SFPGA       , 
    output  wire        FPGA_CCLK_SFPGA         , 
    input   wire        FPGA_DONE_SFPGA         , 
    //dsp
    input   wire        DSP_SYSCLKOUT_SFPGA     , 
    output  wire        DSP_MCBSP1_SLCLK        ,
    input   wire        DSP_MCBSP1_TXCLK        ,
    input   wire        DSP_MCBSP1_FST          ,
    input   wire        DSP_MCBSP1_TX           ,
    output  wire        DSP_MCBSP1_RXCLK        ,
    output  wire        DSP_MCBSP1_FSR          ,
    output  wire        DSP_MCBSP1_RX           ,
    //
    output  wire        FPGA_TO_SFPGA_RESERVE0  ,     //clk
    output  wire        FPGA_TO_SFPGA_RESERVE1  ,     //fsx
    output  wire        FPGA_TO_SFPGA_RESERVE2  ,     //tx
    input   wire        FPGA_TO_SFPGA_RESERVE3  ,     //fsr    
    input   wire        FPGA_TO_SFPGA_RESERVE4  ,     //rx
    input   wire        FPGA_TO_SFPGA_RESERVE5  ,     //reserved
    input   wire        FPGA_TO_SFPGA_RESERVE6  ,     //reserved 
    input   wire        FPGA_TO_SFPGA_RESERVE7  ,     //reserved 
    input   wire        FPGA_TO_SFPGA_RESERVE8  ,     //reserved 
    input   wire        FPGA_TO_SFPGA_RESERVE9  ,     //reserved
    //test io
    output  wire        tp92                    ,
    output  wire        tp93                    ,
    output  wire        tp94                    ,
    output  wire        tp95                    ,
    output  wire        tp96                    ,
    output  wire        tp97                    ,
    output  wire        tp100                   ,
    output  wire        tp101                   ,
    output  wire        tp90                    ,
    output  wire        tp91                    ,
    output  wire        tp98                    ,
    output  wire        tp99                    
);

parameter               SOURCE_PMT1                 = 16'h0001;
parameter               SOURCE_PMT2                 = 16'h0002;
parameter               SOURCE_PMT3                 = 16'h0003;
parameter               SOURCE_TIMING               = 16'h0004;
parameter   [8*20-1:0]  SFPGA_VERSION               = "PCG1_TimingS_v3.2   ";

wire                    locked                      ;
wire                    clk_50m                     ;
wire                    nrst_50m                    ;
wire                    clk_200m                    ;
wire                    nrst_200m                   ;
wire                    clk_100m                    ;
wire                    nrst_100m                   ;
wire                    clk_eth                     ;
wire                    nrst_eth                    ;

wire                    eth_rec_pkt_done            ;
wire                    eth_rec_en                  ;
wire [7:0]              eth_rec_data                ;
wire                    eth_rec_byte_num_en         ;
wire [15:0]             eth_rec_byte_num            ;

wire                    eth_rec_pkt_done_sim        ;
wire                    eth_rec_en_sim              ;
wire [7:0]              eth_rec_data_sim            ;
wire                    eth_rec_byte_num_en_sim     ;
wire [15:0]             eth_rec_byte_num_sim        ;

wire                    eth_rec_cfg_pkg_total_en    ;
wire [15:0]             eth_rec_cfg_pkg_total       ;
wire                    eth_rec_cfg_pkg_num_en      ;
wire [15:0]             eth_rec_cfg_pkg_num         ;
wire                    eth_rec_cfg_done            ;
wire                    eth_rec_cfg_en              ;
wire [7:0]              eth_rec_cfg_data            ;
wire                    eth_rec_cfg_byte_num_en     ;
wire [15:0]             eth_rec_cfg_byte_num        ;

wire                    eth_rec_cfg_pkg_total_en_sim    ;
wire [15:0]             eth_rec_cfg_pkg_total_sim       ;
wire                    eth_rec_cfg_pkg_num_en_sim      ;
wire [15:0]             eth_rec_cfg_pkg_num_sim         ;
wire                    eth_rec_cfg_done_sim            ;
wire                    eth_rec_cfg_en_sim              ;
wire [7:0]              eth_rec_cfg_data_sim            ;
wire                    eth_rec_cfg_byte_num_en_sim     ;
wire [15:0]             eth_rec_cfg_byte_num_sim        ;

wire                    rd_sfpga_version            ;
wire                    sfpga_comm_reset            ;
wire                    comm_ack                    ;
wire                    udp_load_loss               ;
wire [8:0]              udp_load_status             ;
wire                    rd_data_vld                 ;
wire [7:0]              rd_data                     ;


wire                    FPGA_cfg_error              ;

wire                    eth_tx_start_en             ;
wire [15:0]             eth_tx_byte_num             ;
wire                    eth_udp_tx_done             ;
wire                    eth_tx_req                  ;
wire [7:0]              eth_tx_data                 ;


assign                  tp92                =    locked;
assign                  tp93                =    clk_50m;//1'b0;
assign                  tp94                =    FPGA_CONFIG_RDWR_B;
assign                  tp95                =    FPGA_CONFIG_CSI_B ;
assign                  tp96                =    FPGA_INIT_B_SFPGA ;
assign                  tp97                =    FPGA_PROG_B_SFPGA ;
assign                  tp100               =    FPGA_CCLK_SFPGA   ;
assign                  tp101               =    FPGA_DONE_SFPGA   ;
assign                  tp90                =    1'b0;
assign                  tp91                =    1'b0;
assign                  tp98                =    1'b0;
assign                  tp99                =    1'b0;

assign                  SFPGA_RGMII_MDC     =    1'b1;

assign                  SFPGA_ETH_PWR_EN    =    1'b1;

clk_wiz_0 clk_wiz_inst(
    .clk_out1                   ( clk_50m                       ),
    .clk_out2                   ( clk_200m                      ),
    .clk_out3                   ( clk_100m                      ),
    .reset                      ( 1'b0                          ),
    .locked                     ( locked                        ),
    .clk_in1                    ( S_FPGA_CLK                    )
 );

// phy ic reset
reg phy_nrst = 'd0;
reg [24-1:0] phy_rst_cnt = 'd0;
always @(posedge clk_50m)  
begin
    if(~locked) begin
        phy_nrst    <=  1'b0;
        phy_rst_cnt <=  24'd0;
    end
    else if(phy_rst_cnt == 24'd2000000) begin    //40ms
        phy_rst_cnt <=  phy_rst_cnt;
        phy_nrst    <=  1'b1;
    end
    else if(phy_rst_cnt < 24'd500000) begin
        phy_rst_cnt <=  phy_rst_cnt + 24'd1;
        phy_nrst    <=  1'b1;
    end
    else begin
        phy_rst_cnt <=  phy_rst_cnt + 24'd1;
        phy_nrst    <=  1'b0;
    end
end
assign                  SFPGA_ETH_RESET_B   =   phy_nrst;

reset_generate reset_generate_inst(
    .nrst_i                     ( phy_nrst                      ),
    .clk_0_i                    ( clk_50m                       ),
    .nrst_0_o                   ( nrst_50m                      ),
    .clk_1_i                    ( clk_200m                      ),
    .nrst_1_o                   ( nrst_200m                     ),
    .clk_2_i                    ( clk_eth                       ),
    .nrst_2_o                   ( nrst_eth                      ),
    .clk_3_i                    ( clk_100m                      ),
    .nrst_3_o                   ( nrst_100m                     ),
    .debug_info                 (                               )
);

eth_udp_loop eth_udp_loop_inst(
    // .clk                        ( clk_50m                       ),
    .idelay_clk                 ( clk_200m                      ),
    .rst_n                      ( nrst_eth                      ),

    .eth_rxc                    ( SFPGA_RGMII_RX_CLK            ),
    .eth_rx_ctl                 ( SFPGA_RGMII_RX_CTL            ),
    .eth_rxd                    ( SFPGA_RGMII_RXD               ),
    .eth_txc                    ( SFPGA_RGMII_TX_CLK            ),    
    .eth_tx_ctl                 ( SFPGA_RGMII_TX_CTL            ),
    .eth_txd                    ( SFPGA_RGMII_TXD               ),        
    .eth_rst_n                  (                               ),
    .eth_intb                   ( SFPGA_ETH_INTB                ),
    
    .eth_clk                    ( clk_eth                       ),
    
    .rec_pkt_done               ( eth_rec_pkt_done              ),
    .rec_en                     ( eth_rec_en                    ),
    .rec_data                   ( eth_rec_data                  ),
    .rec_byte_num_en            ( eth_rec_byte_num_en           ),
    .rec_byte_num               ( eth_rec_byte_num              ),
    .rd_sfpga_version_o         ( rd_sfpga_version              ),
    .sfpga_comm_reset_o         ( sfpga_comm_reset              ),
    
    .rec_cfg_pkg_total_en       ( eth_rec_cfg_pkg_total_en      ),
    .rec_cfg_pkg_total          ( eth_rec_cfg_pkg_total         ),
    .rec_cfg_pkg_num_en         ( eth_rec_cfg_pkg_num_en        ),
    .rec_cfg_pkg_num            ( eth_rec_cfg_pkg_num           ),
    .rec_cfg_done               ( eth_rec_cfg_done              ),      
    .rec_cfg_en                 ( eth_rec_cfg_en                ),            
    .rec_cfg_data               ( eth_rec_cfg_data              ), 
    .rec_cfg_byte_num_en        ( eth_rec_cfg_byte_num_en       ),
    .rec_cfg_byte_num           ( eth_rec_cfg_byte_num          ),

    .tx_start_en                ( eth_tx_start_en               ),
    .tx_byte_num                ( eth_tx_byte_num               ),
    .udp_tx_done                ( eth_udp_tx_done               ),
    .tx_req                     ( eth_tx_req                    ),
    .tx_data                    ( eth_tx_data                   )
);

// master FPGA load
mfpga_config_ctrl mfpga_config_ctrl_inst(
    .clk                        ( clk_50m                       ),
    .rst_n                      ( nrst_50m                      ),
    
    .eth_clk                    ( clk_eth                       ),
`ifdef SIMULATE
    .eth_rec_cfg_pkg_total_en   ( eth_rec_cfg_pkg_total_en_sim  ),
    .eth_rec_cfg_pkg_total      ( eth_rec_cfg_pkg_total_sim     ),
    .eth_rec_cfg_pkg_num_en     ( eth_rec_cfg_pkg_num_en_sim    ),
    .eth_rec_cfg_pkg_num        ( eth_rec_cfg_pkg_num_sim       ),
    .eth_rec_cfg_en             ( eth_rec_cfg_en_sim            ), 
    .eth_rec_cfg_data           ( eth_rec_cfg_data_sim          ),
`else
    .eth_rec_cfg_pkg_total_en   ( eth_rec_cfg_pkg_total_en      ),
    .eth_rec_cfg_pkg_total      ( eth_rec_cfg_pkg_total         ),
    .eth_rec_cfg_pkg_num_en     ( eth_rec_cfg_pkg_num_en        ),
    .eth_rec_cfg_pkg_num        ( eth_rec_cfg_pkg_num           ),
    .eth_rec_cfg_en             ( eth_rec_cfg_en                ), 
    .eth_rec_cfg_data           ( eth_rec_cfg_data              ),
`endif // SIMULATE
    
    .udp_load_loss_o            ( udp_load_loss                 ),
    .udp_load_status_o          ( udp_load_status               ),
    .FPGA_CONFIG_D              ( FPGA_CONFIG_D                 ),
    .FPGA_CONFIG_RDWR_B         ( FPGA_CONFIG_RDWR_B            ),
    .FPGA_CONFIG_CSI_B          ( FPGA_CONFIG_CSI_B             ),
    .FPGA_INIT_B_SFPGA          ( FPGA_INIT_B_SFPGA             ),
    .FPGA_PROG_B_SFPGA          ( FPGA_PROG_B_SFPGA             ),
    .FPGA_CCLK_SFPGA            ( FPGA_CCLK_SFPGA               ),
// `ifdef SIMULATE
//     .FPGA_DONE_SFPGA            ( cfg_done_sim[255]              )
// `else
    .FPGA_DONE_SFPGA            ( FPGA_DONE_SFPGA               )
// `endif //SIMULATE
);


`ifdef SIMULATE
udp_message_sim  udp_message_sim_inst(
    .phy_clk                    ( clk_eth                       ),
    .rst_n                      ( nrst_eth                      ),
    .rec_pkt_done_o             ( eth_rec_pkt_done_sim          ),
    .rec_en_o                   ( eth_rec_en_sim                ),
    .rec_data_o                 ( eth_rec_data_sim              ),
    .rec_byte_num_en_o          ( eth_rec_byte_num_en_sim       ),
    .rec_byte_num_o             ( eth_rec_byte_num_sim          ),

    .rec_cfg_pkg_total_en_o     ( eth_rec_cfg_pkg_total_en_sim  ),
    .rec_cfg_pkg_total_o        ( eth_rec_cfg_pkg_total_sim     ),
    .rec_cfg_pkg_num_en_o       ( eth_rec_cfg_pkg_num_en_sim    ),
    .rec_cfg_pkg_num_o          ( eth_rec_cfg_pkg_num_sim       ),
    .rec_cfg_en_o               ( eth_rec_cfg_en_sim            ), 
    .rec_cfg_data_o             ( eth_rec_cfg_data_sim          )
);

`endif //SIMULATE



// message response communication
message_comm message_comm_inst(
    // clk & rst
    .phy_rx_clk                 ( clk_eth                       ),
    .clk                        ( clk_50m                       ),
    .rst_n                      ( nrst_50m                      ),
    // ethernet interface for message data
`ifdef SIMULATE
    .rec_pkt_done_i             ( eth_rec_pkt_done_sim          ),
    .rec_en_i                   ( eth_rec_en_sim                ),
    .rec_data_i                 ( eth_rec_data_sim              ),
    .rec_byte_num_en_i          ( eth_rec_byte_num_en_sim       ),
    .rec_byte_num_i             ( eth_rec_byte_num_sim          ),
`else
    .rec_pkt_done_i             ( eth_rec_pkt_done              ),
    .rec_en_i                   ( eth_rec_en                    ),
    .rec_data_i                 ( eth_rec_data                  ),
    .rec_byte_num_en_i          ( eth_rec_byte_num_en           ),
    .rec_byte_num_i             ( eth_rec_byte_num              ),
`endif //SIMULATE
    .comm_ack_o                 ( comm_ack                      ),
    // message rx info
    .rd_data_vld_o              ( rd_data_vld                   ),
    .rd_data_o                  ( rd_data                       ),
    // info
    .MSG_CLK                    ( FPGA_TO_SFPGA_RESERVE0        ),
    .MSG_TX_FSX                 ( FPGA_TO_SFPGA_RESERVE1        ),
    .MSG_TX                     ( FPGA_TO_SFPGA_RESERVE2        ),
    .MSG_RX_FSX                 ( FPGA_TO_SFPGA_RESERVE3        ),
    .MSG_RX0                    ( FPGA_TO_SFPGA_RESERVE4        ),
    .MSG_RX1                    ( FPGA_TO_SFPGA_RESERVE5        ),
    .MSG_RX2                    ( FPGA_TO_SFPGA_RESERVE6        ),
    .MSG_RX3                    ( FPGA_TO_SFPGA_RESERVE7        )
);


// udp to mainPC
udp_tx_control #(
    .SFPGA_VERSION              ( SFPGA_VERSION                 ),
    .MSG_SOURCE                 ( SOURCE_TIMING                 )
)udp_tx_control_inst(
    // clk & rst
    // .clk_i                      ( clk_50m                       ),
    // .rst_i                      ( ~nrst_50m                     ),
    // cfg and comm info
    .rd_sfpga_version_i         ( rd_sfpga_version              ),
    .sfpga_comm_reset_i         ( sfpga_comm_reset              ),
    .cfg_load_loss_i            ( udp_load_loss                 ),
    .cfg_load_status_i          ( udp_load_status               ),
    .comm_ack_i                 ( comm_ack                      ),
    // slave tx data,to main PC
    .phy_clk_i                  ( clk_eth                       ),
    .rd_data_vld_i              ( rd_data_vld                   ),
    .rd_data_i                  ( rd_data                       ),
    // udp tx info
    .tx_start_en_o              ( eth_tx_start_en               ),
    .tx_byte_num_o              ( eth_tx_byte_num               ),
    .udp_tx_done_i              ( eth_udp_tx_done               ),
    .tx_req_i                   ( eth_tx_req                    ),
    .tx_data_o                  ( eth_tx_data                   )
);
endmodule
