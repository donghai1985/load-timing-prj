//~ `New testbench
`timescale  1ns / 1ps

module tb_sfpga_top;

// sfpga_top Parameters
parameter PERIOD  = 20;

wire                rd_mfpga_version                ;
wire    [63:0]      readback_data                   ;
wire                readback_vld                    ;
parameter   [8*20-1:0]      VERSION     = "PCG1_TimingM_v7.1   ";

// sfpga_top Inputs
reg   S_FPGA_CLK                           = 0 ;
reg   SFPGA_RGMII_RX_CLK                   = 1 ;
reg   [3:0]  SFPGA_RGMII_RXD               = 0 ;
reg   SFPGA_RGMII_RX_CTL                   = 0 ;
reg   SFPGA_ETH_INTB                       = 0 ;
reg   FPGA_INIT_B_SFPGA                    = 0 ;
reg   FPGA_DONE_SFPGA                      = 0 ;
reg   FPGA_TO_SFPGA_RESERVE8               = 0 ;
reg   FPGA_TO_SFPGA_RESERVE9               = 0 ;

reg rst_100m = 'd1;
// bpsi_top_if_v2 Inputs
reg   clk_100m                            = 0 ;
reg   clk_300m                              = 0 ;
reg   [1:0] data_acq_en_i                        = 0 ;
reg   bg_data_acq_en_i                     = 0 ;
reg   position_test_en_i                   = 0 ;
reg   [24:0]  position_aim_i               = 0 ;
reg   [22:0]  kp_i                         = 'h10_0000 ;
reg   [1:0]  motor_freq_i                  = 0 ;
reg   motor_data_out_en_i                  = 0 ;
reg   [15:0]  motor_data_out_i             = 0 ;
reg   SSPI_CLK                             = 0 ;
reg   SSPI_MOSI                            = 0 ;


reg motor_data_out_en_sim = 'd0;
reg [16-1:0] motor_data_out_sim = 'd0;
reg [16-1:0] ufeed_cnt = 'd0;

wire                position_cali_en;
wire	[24:0]		position_cali;
wire                bpsi_bg_data_acq_en;
wire    [2:0]       bpsi_data_acq_en;
wire    [24:0]      bpsi_position_aim;
wire    [22:0]      bpsi_kp;
wire    [1:0]       bpsi_motor_freq;
wire				bpsi_position_en;
wire    [15:0]      fbc_bias_voltage;

wire				motor_data_in_en;
wire 	[15:0]		motor_Ufeed_latch;
wire 	[15:0]		motor_data_in;
wire				motor_rd_en;
wire				motor_data_out_en;
wire 	[15:0]		motor_data_out;

wire                bpsi_bg_data_en;
wire	[23:0]		bpsi_bg_data_a;
wire	[23:0]		bpsi_bg_data_b;
wire                bpsi_cali_data_en;
wire    [23:0]      bpsi_cali_data_a;
wire    [23:0]      bpsi_cali_data_b;
wire				BPSi_DATA0;
wire				BPSi_DATA1;
wire				BPSi_DATA2;
wire				BPSi_DATA3;
// sfpga_top Outputs
wire  SFPGA_ETH_PWR_EN                     ;
wire  SFPGA_RGMII_TX_CLK                   ;
wire  [3:0]  SFPGA_RGMII_TXD               ;
wire  SFPGA_RGMII_TX_CTL                   ;
wire  SFPGA_RGMII_MDC                      ;
wire  SFPGA_ETH_RESET_B                    ;
wire  [15:0]  FPGA_CONFIG_D                ;
wire  FPGA_CONFIG_RDWR_B                   ;
wire  FPGA_CONFIG_CSI_B                    ;
wire  FPGA_PROG_B_SFPGA                    ;
wire  FPGA_CCLK_SFPGA                      ;
wire  FPGA_TO_SFPGA_RESERVE0               ;
wire  FPGA_TO_SFPGA_RESERVE1               ;
wire  FPGA_TO_SFPGA_RESERVE2               ;
wire  FPGA_TO_SFPGA_RESERVE3               ;
wire  FPGA_TO_SFPGA_RESERVE4               ;
wire  FPGA_TO_SFPGA_RESERVE5               ;
wire  FPGA_TO_SFPGA_RESERVE6               ;
wire  FPGA_TO_SFPGA_RESERVE7               ;
wire  tp92                                 ;
wire  tp93                                 ;
wire  tp94                                 ;
wire  tp95                                 ;
wire  tp96                                 ;
wire  tp97                                 ;
wire  tp100                                ;
wire  tp101                                ;
wire  tp90                                 ;
wire  tp91                                 ;
wire  tp98                                 ;
wire  tp99                                 ;

wire            bpsi_data_en ;
wire [23:0]     bpsi_data_a  ;
wire [23:0]     bpsi_data_b  ;
// sfpga_top Bidirs
wire  SFPGA_RGMII_MDIO                     ;
wire                slave_tx_ack         ;
wire                slave_tx_byte_en     ;
wire    [ 7:0]      slave_tx_byte        ;
wire                slave_tx_byte_num_en ;
wire    [15:0]      slave_tx_byte_num    ;
wire                slave_rx_data_vld    ;
wire    [ 7:0]      slave_rx_data        ;

initial
begin
    forever #(PERIOD/2)  S_FPGA_CLK=~S_FPGA_CLK;
end
initial
begin
    forever #(3.33/2)  clk_300m=~clk_300m;
end

initial
begin
    forever #(8/2)  SFPGA_RGMII_RX_CLK=~SFPGA_RGMII_RX_CLK;
end

// initial
// begin
//     #(PERIOD*2) rst_n  =  1;
// end

sfpga_top  u_sfpga_top (
    .S_FPGA_CLK              ( S_FPGA_CLK                     ),
    .SFPGA_RGMII_RX_CLK      ( SFPGA_RGMII_RX_CLK             ),
    .SFPGA_RGMII_RXD         ( SFPGA_RGMII_RXD         [3:0]  ),
    .SFPGA_RGMII_RX_CTL      ( SFPGA_RGMII_RX_CTL             ),
    .SFPGA_ETH_INTB          ( SFPGA_ETH_INTB                 ),
    .FPGA_INIT_B_SFPGA       ( FPGA_INIT_B_SFPGA              ),
    .FPGA_DONE_SFPGA         ( FPGA_DONE_SFPGA                ),
    .FPGA_TO_SFPGA_RESERVE3  ( FPGA_TO_SFPGA_RESERVE3         ),
    .FPGA_TO_SFPGA_RESERVE4  ( FPGA_TO_SFPGA_RESERVE4         ),
    .FPGA_TO_SFPGA_RESERVE5  ( FPGA_TO_SFPGA_RESERVE5         ),
    .FPGA_TO_SFPGA_RESERVE6  ( FPGA_TO_SFPGA_RESERVE6         ),
    .FPGA_TO_SFPGA_RESERVE7  ( FPGA_TO_SFPGA_RESERVE7         ),
    .FPGA_TO_SFPGA_RESERVE8  ( FPGA_TO_SFPGA_RESERVE8         ),
    .FPGA_TO_SFPGA_RESERVE9  ( FPGA_TO_SFPGA_RESERVE9         ),

    .FPGA_PROG_B_SFPGA       ( FPGA_PROG_B_SFPGA              ),
    .FPGA_CCLK_SFPGA         ( FPGA_CCLK_SFPGA                ),
    .SFPGA_ETH_PWR_EN        ( SFPGA_ETH_PWR_EN               ),
    .SFPGA_RGMII_TX_CLK      ( SFPGA_RGMII_TX_CLK             ),
    .SFPGA_RGMII_TXD         ( SFPGA_RGMII_TXD         [3:0]  ),
    .SFPGA_RGMII_TX_CTL      ( SFPGA_RGMII_TX_CTL             ),
    .SFPGA_RGMII_MDC         ( SFPGA_RGMII_MDC                ),
    .SFPGA_ETH_RESET_B       ( SFPGA_ETH_RESET_B              ),
   
    .FPGA_TO_SFPGA_RESERVE0  ( FPGA_TO_SFPGA_RESERVE0         ),
    .FPGA_TO_SFPGA_RESERVE1  ( FPGA_TO_SFPGA_RESERVE1         ),
    .FPGA_TO_SFPGA_RESERVE2  ( FPGA_TO_SFPGA_RESERVE2         ),

    .SFPGA_RGMII_MDIO        ( SFPGA_RGMII_MDIO               )
);



always @(posedge FPGA_CCLK_SFPGA ) begin
    if(~FPGA_PROG_B_SFPGA)
        FPGA_INIT_B_SFPGA <= 'd0;
    else 
        FPGA_INIT_B_SFPGA <= 'd1;
end



// mfpga to mainPC message arbitrate 
arbitrate_bpsi #(
    .MFPGA_VERSION                  ( VERSION                       )
) arbitrate_bpsi_inst(
    .clk_i                          ( clk_100m                      ),
    .rst_i                          ( rst_100m                      ),
    
    .readback_data_i                ( readback_data                 ),
    .readback_vld_i                 ( readback_vld                  ),
    

    .rd_mfpga_version_i             ( rd_mfpga_version              ),

    .slave_tx_ack_i                 ( slave_tx_ack                  ),
    .slave_tx_byte_en_o             ( slave_tx_byte_en              ),
    .slave_tx_byte_o                ( slave_tx_byte                 ),
    .slave_tx_byte_num_en_o         ( slave_tx_byte_num_en          ),
    .slave_tx_byte_num_o            ( slave_tx_byte_num             )

);

slave_comm slave_comm_inst(
    // clk & rst
    .clk_sys_i                      ( clk_100m                      ),
    .rst_i                          ( rst_100m                      ),
    // salve tx info
    .slave_tx_en_i                  ( slave_tx_byte_en              ),
    .slave_tx_data_i                ( slave_tx_byte                 ),
    .slave_tx_byte_num_en_i         ( slave_tx_byte_num_en          ),
    .slave_tx_byte_num_i            ( slave_tx_byte_num             ),
    .slave_tx_ack_o                 ( slave_tx_ack                  ),
    // slave rx info
    .rd_data_vld_o                  ( slave_rx_data_vld             ),
    .rd_data_o                      ( slave_rx_data                 ),
    // info
    .SLAVE_MSG_CLK                  ( FPGA_TO_SFPGA_RESERVE0        ),
    .SLAVE_MSG_TX_FSX               ( FPGA_TO_SFPGA_RESERVE3        ),
    .SLAVE_MSG_TX0                  ( FPGA_TO_SFPGA_RESERVE4        ),
    .SLAVE_MSG_TX1                  ( FPGA_TO_SFPGA_RESERVE5        ),
    .SLAVE_MSG_TX2                  ( FPGA_TO_SFPGA_RESERVE6        ),
    .SLAVE_MSG_TX3                  ( FPGA_TO_SFPGA_RESERVE7        ),
    .SLAVE_MSG_RX_FSX               ( FPGA_TO_SFPGA_RESERVE1        ),
    .SLAVE_MSG_RX                   ( FPGA_TO_SFPGA_RESERVE2        )
);

command_map command_map_inst(
    .clk_sys_i                      ( clk_100m                      ),
    .rst_i                          ( rst_100m                      ),
    .slave_rx_data_vld_i            ( slave_rx_data_vld             ),
    .slave_rx_data_i                ( slave_rx_data                 ),
    
    .data_acq_en_o                  ( bpsi_data_acq_en              ),
    .bg_data_acq_en_o               ( bpsi_bg_data_acq_en           ),
    .position_arm_o                 ( bpsi_position_aim             ),
    .kp_o                           ( bpsi_kp                       ),
    .ki_o                           ( bpsi_ki                       ),
    .kd_o                           ( bpsi_kd                       ),
    .motor_freq_o                   ( bpsi_motor_freq               ),
    .bpsi_position_en_o             ( bpsi_position_en              ),
    .fbc_bias_voltage_o             ( fbc_bias_voltage              ),
    .fbc_cali_uop_set_o             ( fbc_cali_uop_set              ),
    .motor_Ufeed_latch_i            ( motor_Ufeed_latch             ),
    .motor_data_in_i                ( motor_data_in                 ), // Uop to motor
    .eds_power_en_o                 ( eds_power_en                  ),
    .eds_frame_en_o                 ( eds_frame_en                  ),
    .eds_test_en_o                  ( eds_test_en                   ),
    .eds_texp_time_o                ( texp_time                     ),
    .eds_frame_to_frame_time_o      ( frame_to_frame_time           ),
    .laser_uart_data_o              ( laser_tx_data                 ),
    .laser_uart_vld_o               ( laser_tx_vld                  ),
    .pmt_master_spi_data_o          ( pmt_master_spi_data           ),
    .pmt_master_spi_vld_o           ( pmt_master_spi_vld            ),
    .pmt_adc_start_data_o           ( pmt_adc_start_data            ),
    .pmt_adc_start_vld_o            ( pmt_adc_start_vld             ),
    .pmt_adc_start_hold_o           ( pmt_adc_start_hold            ),
    .rd_mfpga_version_o             ( rd_mfpga_version              ),
    .FBC_fifo_rst_o                 ( FBC_out_fifo_rst              ),
    .readback_data_o                ( readback_data                 ),
    .readback_vld_o                 ( readback_vld                  ),

    .debug_info                     (                               )
);

initial
begin
    forever #(10/2)  clk_100m=~clk_100m;
end

initial
begin
    rst_100m  =  1;
    #(10*2);
    rst_100m  =  0;
end


endmodule