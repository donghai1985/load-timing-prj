`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/29
// Design Name: 
// Module Name: arbitrate_bpsi
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
// `define FBC_UDP_OFF


module arbitrate_bpsi #(
    parameter                               TCQ                 = 0.1   ,
    parameter        [8*20-1:0]             MFPGA_VERSION       = "PCG1_TimingM_v1.1   "
)(
    // clk & rst
    input    wire                           clk_i                       ,
    input    wire                           rst_i                       ,
    input    wire    [64-1:0]               readback_data_i             ,
    input    wire                           readback_vld_i              ,
    // calibrate voltage. dark current * R
    input    wire                           FBCi_cali_en_i              ,
    input    wire    [23:0]                 FBCi_cali_a_i               ,
    input    wire    [23:0]                 FBCi_cali_b_i               ,
    input    wire                           FBCr1_cali_en_i             ,
    input    wire    [23:0]                 FBCr1_cali_a_i              ,
    input    wire    [23:0]                 FBCr1_cali_b_i              ,
    input    wire                           FBCr2_cali_en_i             ,
    input    wire    [23:0]                 FBCr2_cali_a_i              ,
    input    wire    [23:0]                 FBCr2_cali_b_i              ,
    
    // actual voltage
    input    wire                           FBC_out_fifo_rst_i          ,
    input    wire                           FBCi_out_en_i               ,
    input    wire    [23:0]                 FBCi_out_a_i                ,
    input    wire    [23:0]                 FBCi_out_b_i                ,
    input    wire                           FBCr1_out_en_i              ,
    input    wire    [23:0]                 FBCr1_out_a_i               ,
    input    wire    [23:0]                 FBCr1_out_b_i               ,
    input    wire                           FBCr2_out_en_i              ,
    input    wire    [23:0]                 FBCr2_out_a_i               ,
    input    wire    [23:0]                 FBCr2_out_b_i               ,

    // background voltage. dark current * R
    input    wire                           FBCi_bg_en_i                ,
    input    wire    [23:0]                 FBCi_bg_a_i                 ,
    input    wire    [23:0]                 FBCi_bg_b_i                 ,
    input    wire                           FBCr1_bg_en_i               ,
    input    wire    [23:0]                 FBCr1_bg_a_i                ,
    input    wire    [23:0]                 FBCr1_bg_b_i                ,
    input    wire                           FBCr2_bg_en_i               ,
    input    wire    [23:0]                 FBCr2_bg_a_i                ,
    input    wire    [23:0]                 FBCr2_bg_b_i                ,

    input    wire                           motor_data_in_en_i          , // Uop en
    input    wire    [15:0]                 motor_data_in_i             , // Uop to motor
    input    wire    [15:0]                 motor_data_out_i            , // Ufeed
    
    input    wire    [8-1:0]                laser_rx_data_i             ,
    input    wire                           laser_rx_vld_i              ,
    input    wire                           laser_rx_last_i             ,

    input    wire                           spi_slave0_ack_rst_i        ,
    input    wire                           spi_slave0_ack_vld_i        ,
    input    wire                           spi_slave0_ack_last_i       ,
    input    wire    [32-1:0]               spi_slave0_ack_data_i       ,
    input    wire                           spi_slave1_ack_rst_i        ,
    input    wire                           spi_slave1_ack_vld_i        ,
    input    wire                           spi_slave1_ack_last_i       ,
    input    wire    [32-1:0]               spi_slave1_ack_data_i       ,
    input    wire                           spi_slave2_ack_rst_i        ,
    input    wire                           spi_slave2_ack_vld_i        ,
    input    wire                           spi_slave2_ack_last_i       ,
    input    wire    [32-1:0]               spi_slave2_ack_data_i       ,

    // read mfpga version
    input    wire                           rd_mfpga_version_i          ,

    // slave comm
    input    wire                           slave_tx_ack_i              ,
    output   wire                           slave_tx_byte_num_en_o      ,
    output   wire   [15:0]                  slave_tx_byte_num_o         ,
    output   wire                           slave_tx_byte_en_o          ,
    output   wire   [ 7:0]                  slave_tx_byte_o             

);


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam          [16-1:0]                FBC_ACTUAL_TYPE             = 'h010F;
localparam          [11-1:0]                FBC_ACTUAL_DATA_NUM         = 'd30;
localparam          [16-1:0]                FBC_ACTUAL_BYTE_NUM         = FBC_ACTUAL_DATA_NUM * 18 + 2;

localparam          [16-1:0]                FBC_VOL_TYPE                = 'h010B;
localparam          [11-1:0]                FBC_VOL_DATA_NUM            = 'd10;
localparam          [16-1:0]                FBC_VOL_BYTE_NUM            = FBC_VOL_DATA_NUM * 4 + 2;

localparam          [16-1:0]                FBC_BG_CALI_TYPE            = 'h0115;
localparam          [16-1:0]                FBC_BG_CALI_NUM             = 'd21;

localparam          [16-1:0]                LASER_UART_TYPE             = 'h0122;

localparam          [16-1:0]                SPI_SLAVE_ACK_TYPE          = 'h0123;

localparam          [16-1:0]                MFPGA_VERSION_TYPE          = 'h0100;
localparam          [16-1:0]                MFPGA_VERSION_NUM           = 'd22;

localparam          [16-1:0]                READBACK_TYPE               = 'h0200;
localparam          [16-1:0]                READBACK_NUM                = 'd10;

localparam                                  ARBITRATE_NUM               = 10;    // control arbitrate channel
genvar i;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg                                         FBCi_wr_en_d0               = 'd0;
reg                                         FBCi_wr_en_d1               = 'd0;
reg                                         FBCr1_wr_en_d0              = 'd0;
reg                                         FBCr1_wr_en_d1              = 'd0;
reg                                         FBCr2_wr_en_d0              = 'd0;
reg                                         FBCr2_wr_en_d1              = 'd0;

reg                 [24-1:0]                FBCi_data_b                 = 'd0;
reg                 [24-1:0]                FBCr1_data_b                = 'd0;
reg                 [24-1:0]                FBCr2_data_b                = 'd0;

reg                 [24-1:0]                FBCi_din                    = 'd0;
reg                 [24-1:0]                FBCr1_din                   = 'd0;
reg                 [24-1:0]                FBCr2_din                   = 'd0;

reg                 [ 7-1:0]                FBCi_cnt                    = 'd0;
reg                 [ 7-1:0]                FBCr1_cnt                   = 'd0;
reg                 [ 7-1:0]                FBCr2_cnt                   = 'd0;

reg                                         FBCi_rd_en                  = 'd0;
reg                                         FBCr1_rd_en                 = 'd0;
reg                                         FBCr2_rd_en                 = 'd0;


reg                                         fbc_i_full                  = 'd0;
reg                                         fbc_r1_full                 = 'd0;
reg                                         fbc_r2_full                 = 'd0;

reg                                         fbc_vol_wr_en_d0            = 'd0;
reg                                         fbc_vol_wr_en_d1            = 'd0;
reg                 [16-1:0]                fbc_vol_feed_d              = 'd0;
reg                 [16-1:0]                fbc_vol_din                 = 'd0;
reg                 [ 7-1:0]                fbc_vol_cnt                 = 'd0;
reg                                         fbc_vol_rd_en               = 'd0;

reg                 [10-1:0]                laser_rx_cnt                = 'd0;
reg                 [10-1:0]                laser_rx_num                = 'd0;
reg                                         laser_rx_rd_en              = 'd0;

reg                 [ARBITRATE_NUM-1:0]     arbitrate                   = 'd1;
reg                 [ARBITRATE_NUM-1:0]     bpsi_type                   = 'd0;
reg                                         arbitr_result_d0            = 'd0;
reg                                         arbitr_result_d1            = 'd0;

reg                 [160-1:0]               slave_tx_data               = 'd0;

reg                 [11-1:0]                slave_tx_cnt                = 'd0;
reg                 [ 8-1:0]                slave_tx_byte               = 'd0;
reg                                         slave_tx_byte_en            = 'd0;

reg                 [24-1:0]                FBCi_bg_data_a_sync         = 'd0;
reg                 [24-1:0]                FBCi_bg_data_b_sync         = 'd0;
reg                 [24-1:0]                FBCi_cali_data_a_sync       = 'd0;
reg                 [24-1:0]                FBCi_cali_data_b_sync       = 'd0;
reg                 [24-1:0]                FBCr1_bg_data_a_sync        = 'd0;
reg                 [24-1:0]                FBCr1_bg_data_b_sync        = 'd0;
reg                 [24-1:0]                FBCr1_cali_data_a_sync      = 'd0;
reg                 [24-1:0]                FBCr1_cali_data_b_sync      = 'd0;
reg                 [24-1:0]                FBCr2_bg_data_a_sync        = 'd0;
reg                 [24-1:0]                FBCr2_bg_data_b_sync        = 'd0;
reg                 [24-1:0]                FBCr2_cali_data_a_sync      = 'd0;
reg                 [24-1:0]                FBCr2_cali_data_b_sync      = 'd0;
reg                 [25-1:0]                bpsi_position_actual_sync   = 'd0;  
reg                 [16-1:0]                slave_tx_byte_num           = 'd0;

reg                                         FBCi_bg_ready               = 'd0;
reg                                         FBCr1_bg_ready              = 'd0;
reg                                         FBCr2_bg_ready              = 'd0;
reg                                         FBCi_cali_ready             = 'd0;
reg                                         FBCr1_cali_ready            = 'd0;
reg                                         FBCr2_cali_ready            = 'd0;

reg                 [ 6-1:0]                spi_slave_ack_cnt [2:0] ;
reg                 [ 6-1:0]                spi_slave_ack_num [2:0] ;
reg                                         spi_ack_rd_en     [2:0] ;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                [ARBITRATE_NUM-1:0]     bpsi_en             ;
wire                [ARBITRATE_NUM-1:0]     arbitr_result       ;

wire                                        FBCi_arbitr_en      ;
wire                                        FBCr1_arbitr_en     ;
wire                                        FBCr2_arbitr_en     ;
wire                [24-1:0]                FBCi_dout           ;
wire                [24-1:0]                FBCr1_dout          ;
wire                [24-1:0]                FBCr2_dout          ;
wire                                        FBCi_full           ;
wire                                        FBCi_empty          ;
wire                                        FBCr1_full          ;
wire                                        FBCr1_empty         ;
wire                                        FBCr2_full          ;
wire                                        FBCr2_empty         ;

wire                                        fbc_arbitr_en       ;

wire                                        FBC_bg_arbitr_en    ;
wire                                        FBC_cali_arbitr_en  ;

wire                [16-1:0]                fbc_vol_dout        ;
wire                                        fbc_vol_arbitr_en   ;

wire                                        laser_rx_en         ;
wire                [ 8-1:0]                laser_rx_dout       ;

wire                                        spi_slave_ack_en [2:0];
wire                [32-1:0]                spi_ack_rd_dout  [2:0];
wire                                        spi_ack_full     [2:0];
wire                                        spi_ack_empty    [2:0];

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<





//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
bpsi_fbc_fifo bpsi_fbci_fifo_inst(
  .clk                      ( clk_i                                 ),
  .srst                     ( rst_i || FBC_out_fifo_rst_i           ),
  .din                      ( FBCi_din                              ),
  .wr_en                    ( FBCi_wr_en_d1 || FBCi_wr_en_d0        ),
  .rd_en                    ( FBCi_rd_en                            ),
  .dout                     ( FBCi_dout                             ),
  .full                     ( FBCi_full                             ),
  .empty                    ( FBCi_empty                            )
);

bpsi_fbc_fifo bpsi_fbcr1_fifo_inst(
  .clk                      ( clk_i                                 ),
  .srst                     ( rst_i || FBC_out_fifo_rst_i           ),
  .din                      ( FBCr1_din                             ),
  .wr_en                    ( FBCr1_wr_en_d1 || FBCr1_wr_en_d0      ),
  .rd_en                    ( FBCr1_rd_en                           ),
  .dout                     ( FBCr1_dout                            ),
  .full                     ( FBCr1_full                            ),
  .empty                    ( FBCr1_empty                           )
);

bpsi_fbc_fifo bpsi_fbcr2_fifo_inst(
  .clk                      ( clk_i                                 ),
  .srst                     ( rst_i || FBC_out_fifo_rst_i           ),
  .din                      ( FBCr2_din                             ),
  .wr_en                    ( FBCr2_wr_en_d1 || FBCr2_wr_en_d0      ),
  .rd_en                    ( FBCr2_rd_en                           ),
  .dout                     ( FBCr2_dout                            ),
  .full                     ( FBCr2_full                            ),
  .empty                    ( FBCr2_empty                           )
);

fbc_voltage_fifo fbc_voltage_fifo_inst(
  .clk                      ( clk_i                                 ),
  .srst                     ( rst_i                                 ),
  .din                      ( fbc_vol_din                           ),
  .wr_en                    ( fbc_vol_wr_en_d1 || fbc_vol_wr_en_d0  ),
  .rd_en                    ( fbc_vol_rd_en                         ),
  .dout                     ( fbc_vol_dout                          ),
  .full                     ( fbc_vol_full                          ),
  .empty                    ( fbc_vol_empty                         )
);

laser_rx_fifo laser_rx_fifo_inst(
    .clk                    ( clk_i                                 ),
    .srst                   ( rst_i                                 ),
    .din                    ( laser_rx_data_i                       ),
    .wr_en                  ( laser_rx_vld_i                        ),
    .rd_en                  ( laser_rx_rd_en                        ),
    .dout                   ( laser_rx_dout                         ),
    .full                   ( laser_rx_full                         ),
    .empty                  ( laser_rx_empty                        )
);

generate
    for (i = 0; i<3; i=i+1) begin
        spi_slave_ack_fifo spi_slave_ack_fifo_inst(
            .clk            ( clk_i                                 ),
            .srst           ( rst_i                                 ),
            .din            ( spi_slave_ack_data[i]                 ),
            .wr_en          ( spi_slave_ack_vld[i]                  ),
            .rd_en          ( spi_ack_rd_en[i]                      ),
            .dout           ( spi_ack_rd_dout[i]                    ),
            .full           ( spi_ack_full[i]                       ),
            .empty          ( spi_ack_empty[i]                      )
        );
    end
endgenerate

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// FBCi,FBCr1,FBCr2 data ready. 
// generate a transmission every 100 times.
// fifo width is 3byte,twice write en,for save resources.
always @(posedge clk_i) FBCi_wr_en_d0   <= #TCQ FBCi_out_en_i;
always @(posedge clk_i) FBCi_wr_en_d1   <= #TCQ FBCi_wr_en_d0;
always @(posedge clk_i) FBCr1_wr_en_d0  <= #TCQ FBCr1_out_en_i;
always @(posedge clk_i) FBCr1_wr_en_d1  <= #TCQ FBCr1_wr_en_d0;
always @(posedge clk_i) FBCr2_wr_en_d0  <= #TCQ FBCr2_out_en_i;
always @(posedge clk_i) FBCr2_wr_en_d1  <= #TCQ FBCr2_wr_en_d0;

always @(posedge clk_i) FBCi_data_b     <= #TCQ FBCi_out_b_i;
always @(posedge clk_i) FBCr1_data_b    <= #TCQ FBCr1_out_b_i;
always @(posedge clk_i) FBCr2_data_b    <= #TCQ FBCr2_out_b_i;
always @(posedge clk_i) begin
    if(FBCi_out_en_i)
        FBCi_din <= #TCQ FBCi_out_a_i;
    else if(FBCi_wr_en_d0)
        FBCi_din <= #TCQ FBCi_data_b;
end
always @(posedge clk_i) begin
    if(FBCr1_out_en_i)
        FBCr1_din <= #TCQ FBCr1_out_a_i;
    else if(FBCr1_wr_en_d0)
        FBCr1_din <= #TCQ FBCr1_data_b;
end
always @(posedge clk_i) begin
    if(FBCr2_out_en_i)
        FBCr2_din <= #TCQ FBCr2_out_a_i;
    else if(FBCr2_wr_en_d0)
        FBCr2_din <= #TCQ FBCr2_data_b;
end

always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        FBCi_cnt <= #TCQ 'd0;
    else if(FBCi_wr_en_d1)begin
        if(FBCi_cnt == FBC_ACTUAL_DATA_NUM - 1)
            FBCi_cnt <= #TCQ 'd0;
        else 
            FBCi_cnt <= #TCQ FBCi_cnt + 1;
    end
end

always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        FBCr1_cnt <= #TCQ 'd0;
    else if(FBCr1_wr_en_d1)begin
        if(FBCr1_cnt == FBC_ACTUAL_DATA_NUM - 1)
            FBCr1_cnt <= #TCQ 'd0;
        else 
            FBCr1_cnt <= #TCQ FBCr1_cnt + 1;
    end
end

always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        FBCr2_cnt <= #TCQ 'd0;
    else if(FBCr2_wr_en_d1)begin
        if(FBCr2_cnt == FBC_ACTUAL_DATA_NUM - 1)
            FBCr2_cnt <= #TCQ 'd0;
        else 
            FBCr2_cnt <= #TCQ FBCr2_cnt + 1;
    end
end

always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        fbc_i_full <= #TCQ 'd0;
    else if(FBCi_arbitr_en)
        fbc_i_full <= #TCQ 'd1;
    else if(fbc_arbitr_en)
        fbc_i_full <= #TCQ 'd0;
end
always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        fbc_r1_full <= #TCQ 'd0;
    else if(FBCr1_arbitr_en)
        fbc_r1_full <= #TCQ 'd1;
    else if(fbc_arbitr_en)
        fbc_r1_full <= #TCQ 'd0;
end
always @(posedge clk_i) begin
    if(rst_i || FBC_out_fifo_rst_i)
        fbc_r2_full <= #TCQ 'd0;
    else if(FBCr2_arbitr_en)
        fbc_r2_full <= #TCQ 'd1;
    else if(fbc_arbitr_en)
        fbc_r2_full <= #TCQ 'd0;
end
assign FBCi_arbitr_en = (FBCi_cnt == FBC_ACTUAL_DATA_NUM - 1) && FBCi_wr_en_d1;
assign FBCr1_arbitr_en = (FBCr1_cnt == FBC_ACTUAL_DATA_NUM - 1) && FBCr1_wr_en_d1;
assign FBCr2_arbitr_en = (FBCr2_cnt == FBC_ACTUAL_DATA_NUM - 1) && FBCr2_wr_en_d1;

assign fbc_arbitr_en = fbc_i_full && fbc_r1_full && fbc_r2_full;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< generate fbc_sensor enable end

// fbc_background_voltage and fbc_cali_position data ready.
always @(posedge clk_i) begin
    if(FBCi_bg_en_i)begin
        FBCi_bg_data_a_sync <= #TCQ FBCi_bg_a_i;
        FBCi_bg_data_b_sync <= #TCQ FBCi_bg_b_i;
    end 

    if(FBCi_cali_en_i)begin
        FBCi_cali_data_a_sync <= #TCQ FBCi_cali_a_i;
        FBCi_cali_data_b_sync <= #TCQ FBCi_cali_b_i;
    end
end

always @(posedge clk_i) begin
    if(FBCr1_bg_en_i)begin
        FBCr1_bg_data_a_sync <= #TCQ FBCr1_bg_a_i;
        FBCr1_bg_data_b_sync <= #TCQ FBCr1_bg_b_i;
    end 

    if(FBCr1_cali_en_i)begin
        FBCr1_cali_data_a_sync <= #TCQ FBCr1_cali_a_i;
        FBCr1_cali_data_b_sync <= #TCQ FBCr1_cali_b_i;
    end
end

always @(posedge clk_i) begin
    if(FBCr2_bg_en_i)begin
        FBCr2_bg_data_a_sync <= #TCQ FBCr2_bg_a_i;
        FBCr2_bg_data_b_sync <= #TCQ FBCr2_bg_b_i;
    end 

    if(FBCr2_cali_en_i)begin
        FBCr2_cali_data_a_sync <= #TCQ FBCr2_cali_a_i;
        FBCr2_cali_data_b_sync <= #TCQ FBCr2_cali_b_i;
    end
end

always @(posedge clk_i) begin
    if(FBCi_bg_en_i)            FBCi_bg_ready <= #TCQ 'd1;
    else if(FBC_bg_arbitr_en)   FBCi_bg_ready <= #TCQ 'd0;
    
    if(FBCr1_bg_en_i)           FBCr1_bg_ready <= #TCQ 'd1;
    else if(FBC_bg_arbitr_en)   FBCr1_bg_ready <= #TCQ 'd0;
    
    if(FBCr2_bg_en_i)           FBCr2_bg_ready <= #TCQ 'd1;
    else if(FBC_bg_arbitr_en)   FBCr2_bg_ready <= #TCQ 'd0;
end
assign FBC_bg_arbitr_en = FBCi_bg_ready && FBCr1_bg_ready && FBCr2_bg_ready;

always @(posedge clk_i) begin
    if(FBCi_cali_en_i)            FBCi_cali_ready <= #TCQ 'd1;
    else if(FBC_cali_arbitr_en)   FBCi_cali_ready <= #TCQ 'd0;
    
    if(FBCr1_cali_en_i)           FBCr1_cali_ready <= #TCQ 'd1;
    else if(FBC_cali_arbitr_en)   FBCr1_cali_ready <= #TCQ 'd0;
    
    if(FBCr2_cali_en_i)           FBCr2_cali_ready <= #TCQ 'd1;
    else if(FBC_cali_arbitr_en)   FBCr2_cali_ready <= #TCQ 'd0;
end
assign FBC_cali_arbitr_en = FBCi_cali_ready && FBCr1_cali_ready && FBCr2_cali_ready;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< generate fbc_background and fbc_calibration enable end

// FBC_Uop FBC_Ufeed data ready. 
// generate a transmission every 10 times.
// fifo width is 2byte,twice write en,for save resources.
always @(posedge clk_i) fbc_vol_wr_en_d0 <= #TCQ motor_data_in_en_i;
always @(posedge clk_i) fbc_vol_wr_en_d1 <= #TCQ fbc_vol_wr_en_d0;

always @(posedge clk_i) fbc_vol_feed_d <= #TCQ motor_data_out_i;
always @(posedge clk_i) begin
    if(motor_data_in_en_i)
        fbc_vol_din <= #TCQ motor_data_in_i;
    else if(fbc_vol_wr_en_d0)
        fbc_vol_din <= #TCQ fbc_vol_feed_d;
end

always @(posedge clk_i) begin
    if(fbc_vol_wr_en_d1)begin
        if(fbc_vol_cnt == FBC_VOL_DATA_NUM - 1)
            fbc_vol_cnt <= #TCQ 'd0;
        else 
            fbc_vol_cnt <= #TCQ fbc_vol_cnt + 1;
    end
end

assign fbc_vol_arbitr_en = (fbc_vol_cnt == FBC_VOL_DATA_NUM - 1) && fbc_vol_wr_en_d1;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< generate fbc_uop and fbc_ufeed enable end 

// generate laser_rx enable
// when laser_rx_last, upload LASER_RX_NUM and sync enable. 
always @(posedge clk_i) begin
    if(laser_rx_vld_i && ~laser_rx_last_i)
        laser_rx_cnt <= #TCQ laser_rx_cnt + 1;
    else if(laser_rx_last_i)
        laser_rx_cnt <= #TCQ 'd0;
end

always @(posedge clk_i) begin
    if(laser_rx_last_i)
        laser_rx_num <= #TCQ laser_rx_cnt + 1;
end

assign laser_rx_en = laser_rx_vld_i && laser_rx_last_i;

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< generate laser_rx enable end
// wire            spi_slave_ack_rst  [2:0];
wire            spi_slave_ack_vld  [2:0];
wire            spi_slave_ack_last [2:0];
wire [32-1:0]   spi_slave_ack_data [2:0];

// assign  spi_slave_ack_rst [0] = spi_slave0_ack_rst_i ;
assign  spi_slave_ack_vld [0] = spi_slave0_ack_vld_i ;
assign  spi_slave_ack_last[0] = spi_slave0_ack_last_i;
assign  spi_slave_ack_data[0] = spi_slave0_ack_data_i;
// assign  spi_slave_ack_rst [1] = spi_slave1_ack_rst_i ;
assign  spi_slave_ack_vld [1] = spi_slave1_ack_vld_i ;
assign  spi_slave_ack_last[1] = spi_slave1_ack_last_i;
assign  spi_slave_ack_data[1] = spi_slave1_ack_data_i;
// assign  spi_slave_ack_rst [2] = spi_slave2_ack_rst_i ;
assign  spi_slave_ack_vld [2] = spi_slave2_ack_vld_i ;
assign  spi_slave_ack_last[2] = spi_slave2_ack_last_i;
assign  spi_slave_ack_data[2] = spi_slave2_ack_data_i;

// generate new arbitrate enable
// generate spi slave ack enable, from PMT board
generate
    for (i=0;i<3;i=i+1) begin
        always @(posedge clk_i) begin
            if(rst_i)
                spi_slave_ack_cnt[i] <= #TCQ 'd0;
            else if(spi_slave_ack_vld[i] && ~spi_slave_ack_last[i])
                spi_slave_ack_cnt[i] <= #TCQ spi_slave_ack_cnt[i] + 1;
            else if(spi_slave_ack_last[i])
                spi_slave_ack_cnt[i] <= #TCQ 'd0;
        end
        
        always @(posedge clk_i) begin
            if(spi_slave_ack_last[i])
                spi_slave_ack_num[i] <= #TCQ spi_slave_ack_cnt[i] + 1;
        end
        
        assign spi_slave_ack_en[i] = spi_slave_ack_last[i];
    end
endgenerate

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< generate new_arbitrate enable end

// generate arbitrate enable , alwaye modify when arbitrate channel add.
`ifdef FBC_UDP_OFF
assign bpsi_en          = {  
                             readback_vld_i
                            ,rd_mfpga_version_i
                            ,spi_slave_ack_en[2]
                            ,spi_slave_ack_en[1]
                            ,spi_slave_ack_en[0]
                            ,laser_rx_en
                            ,1'b0 //fbc_vol_arbitr_en
                            ,1'b0 //FBC_cali_arbitr_en
                            ,1'b0 //FBC_bg_arbitr_en
                            ,1'b0 //fbc_arbitr_en 
                        };
`else
assign bpsi_en          = {  readback_vld_i
                            ,rd_mfpga_version_i
                            ,spi_slave_ack_en[2]
                            ,spi_slave_ack_en[1]
                            ,spi_slave_ack_en[0]
                            ,laser_rx_en
                            ,fbc_vol_arbitr_en
                            ,FBC_cali_arbitr_en
                            ,FBC_bg_arbitr_en
                            ,fbc_arbitr_en };
`endif // FBC_UDP_OFF
assign arbitr_result    = bpsi_type & arbitrate;

// arbitr trigger
generate
    for(i=0;i<ARBITRATE_NUM;i=i+1)begin
        always @(posedge clk_i) begin
            if(bpsi_en[i])
                bpsi_type[i] <= #TCQ 1'b1;
            else if(slave_tx_ack_i && arbitr_result[i])
                bpsi_type[i] <= #TCQ 1'b0;
        end
    end
endgenerate

// check arbitr
always @(posedge clk_i) begin
    if(rst_i)
        arbitrate <= #TCQ 'd1;
    else if(arbitr_result=='d0)
        arbitrate <= #TCQ {arbitrate[ARBITRATE_NUM-2:0],arbitrate[ARBITRATE_NUM-1]};
    else 
        arbitrate <= #TCQ arbitrate;
end
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< arbitrate control , no need to change

// with arbitrate number change
always @(posedge clk_i) begin
    case(arbitr_result)
        // 'b0000001:slave_tx_data <= #TCQ {24'd0,8'h00,FBCi_dout};
        // 'b0000010:slave_tx_data <= #TCQ {24'd0,8'h01,FBCr1_dout};
        // 'b0000100:slave_tx_data <= #TCQ {24'd0,8'h02,FBCr2_dout};
        'b0000000010:slave_tx_data <= #TCQ {8'd0,8'h02 ,FBCi_bg_data_a_sync[23:0],FBCi_bg_data_b_sync[23:0]
                                        ,FBCr1_bg_data_a_sync[23:0],FBCr1_bg_data_b_sync[23:0]
                                        ,FBCr2_bg_data_a_sync[23:0],FBCr2_bg_data_b_sync[23:0]};

        'b0000000100:slave_tx_data <= #TCQ {8'd0,8'h01 ,FBCi_cali_data_a_sync[23:0],FBCi_cali_data_b_sync[23:0]
                                        ,FBCr1_cali_data_a_sync[23:0],FBCr1_cali_data_b_sync[23:0]
                                        ,FBCr2_cali_data_a_sync[23:0],FBCr2_cali_data_b_sync[23:0]};
        'b0100000000:slave_tx_data <= #TCQ MFPGA_VERSION;
        default:/*default*/;
    endcase
end

// FBC sensor byte control
reg [2:0] fbc_cnt  = 'd5;
reg [1:0] fbc_byte_cnt = 'd2;
reg [11-1:0] fbc_tx_cnt = 'd0;
always @(posedge clk_i) begin
    if(arbitr_result[0] && fbc_tx_cnt>='d1)begin
        if(fbc_byte_cnt=='d2 && fbc_cnt=='d5)begin
            fbc_byte_cnt <= #TCQ 'd0;
            fbc_cnt      <= #TCQ 'd0;
        end
        else if(fbc_byte_cnt=='d2)begin
            fbc_byte_cnt <= #TCQ 'd0;
            fbc_cnt      <= #TCQ fbc_cnt + 1;
        end
        else begin
            fbc_byte_cnt <= #TCQ fbc_byte_cnt + 1;
            fbc_cnt      <= #TCQ fbc_cnt;
        end
    end
    else begin
        fbc_byte_cnt <= #TCQ 'd2;
        fbc_cnt      <= #TCQ 'd5;
    end
end
always @(posedge clk_i) begin
    if(arbitr_result[0])begin
        if(fbc_tx_cnt<FBC_ACTUAL_DATA_NUM + 2 && fbc_cnt=='d5 && fbc_byte_cnt=='d2)begin
            fbc_tx_cnt <= #TCQ fbc_tx_cnt + 1;
        end
    end
    else begin
        fbc_tx_cnt <= #TCQ 'd0;
    end
end

// spi ack
reg [2-1:0] spi_byte_cnt = 'd0;
reg [6-1:0] spi_ack_cnt = 'd0;
always @(posedge clk_i) begin
    if(arbitr_result[5])begin
        if(spi_ack_cnt < 'd2)begin
            spi_ack_cnt  <= #TCQ spi_ack_cnt + 1;
            spi_byte_cnt <= #TCQ 'd0;
        end
        else if(spi_ack_cnt < spi_slave_ack_num[0]+2) begin
            spi_ack_cnt  <= #TCQ (spi_byte_cnt=='d3) ? spi_ack_cnt + 1 : spi_ack_cnt;
            spi_byte_cnt <= #TCQ (spi_byte_cnt=='d3) ? 'd0 : spi_byte_cnt + 1;
        end
        else begin
            spi_ack_cnt  <= #TCQ spi_slave_ack_num[0]+3;
            spi_byte_cnt <= #TCQ 'd0;
        end
    end
    else if(arbitr_result[6])begin
        if(spi_ack_cnt < 'd2)begin
            spi_ack_cnt  <= #TCQ spi_ack_cnt + 1;
            spi_byte_cnt <= #TCQ 'd0;
        end
        else if(spi_ack_cnt < spi_slave_ack_num[1]+2) begin
            spi_ack_cnt  <= #TCQ (spi_byte_cnt=='d3) ? spi_ack_cnt + 1 : spi_ack_cnt;
            spi_byte_cnt <= #TCQ (spi_byte_cnt=='d3) ? 'd0 : spi_byte_cnt + 1;
        end
        else begin
            spi_ack_cnt  <= #TCQ spi_slave_ack_num[1]+3;
            spi_byte_cnt <= #TCQ 'd0;
        end
    end
    else if(arbitr_result[7])begin
        if(spi_ack_cnt < 'd2)begin
            spi_ack_cnt  <= #TCQ spi_ack_cnt + 1;
            spi_byte_cnt <= #TCQ 'd0;
        end
        else if(spi_ack_cnt < spi_slave_ack_num[2]+2) begin
            spi_ack_cnt  <= #TCQ (spi_byte_cnt=='d3) ? spi_ack_cnt + 1 : spi_ack_cnt;
            spi_byte_cnt <= #TCQ (spi_byte_cnt=='d3) ? 'd0 : spi_byte_cnt + 1;
        end
        else begin
            spi_ack_cnt  <= #TCQ spi_slave_ack_num[2]+3;
            spi_byte_cnt <= #TCQ 'd0;
        end
    end
    else begin
        spi_ack_cnt <= #TCQ 'd0;
        spi_byte_cnt <= #TCQ 'd0;
    end
end


reg [1:0] byte_cnt = 'd0;
always @(posedge clk_i) begin
    if(arbitr_result[3])begin
        if(slave_tx_cnt >= 'd2)begin
            byte_cnt <= #TCQ (byte_cnt=='d1) ? 'd0 : (byte_cnt + 1);
        end
    end
    else begin
        byte_cnt <= #TCQ 'd0;
    end
end

always @(posedge clk_i) begin
    if(arbitr_result[1] || arbitr_result[2])begin
        if(slave_tx_cnt < FBC_BG_CALI_NUM + 1)
            slave_tx_cnt <= #TCQ slave_tx_cnt + 1;
    end
    else if(arbitr_result[8])begin
        if(slave_tx_cnt < MFPGA_VERSION_NUM + 1)
            slave_tx_cnt <= #TCQ slave_tx_cnt + 1;
    end
    else if(arbitr_result[3])begin
        if(slave_tx_cnt < 'd3)begin
            slave_tx_cnt <= #TCQ slave_tx_cnt + 1;
        end
        else if(slave_tx_cnt < FBC_VOL_DATA_NUM*2+3)begin
            slave_tx_cnt <= #TCQ (byte_cnt=='d0) ? (slave_tx_cnt + 1) : slave_tx_cnt;
        end
    end
    else if(arbitr_result[4])begin
        if(slave_tx_cnt<laser_rx_num+3)
            slave_tx_cnt <= #TCQ slave_tx_cnt + 1;
    end
    else begin
        slave_tx_cnt <= #TCQ 'd0;
    end
end

always @(posedge clk_i) begin
    if((arbitr_result[0]) && slave_tx_byte_en && fbc_byte_cnt=='d1)begin
        FBCi_rd_en  <= #TCQ fbc_cnt[2:1]=='b00;
        FBCr1_rd_en <= #TCQ fbc_cnt[2:1]=='b01;
        FBCr2_rd_en <= #TCQ fbc_cnt[2:1]=='b10;
    end
    else begin
        FBCi_rd_en  <= #TCQ 'd0;
        FBCr1_rd_en <= #TCQ 'd0;
        FBCr2_rd_en <= #TCQ 'd0;
    end
end

always @(posedge clk_i) begin  // rd_en to slave_tx_byte need 1clk
    if(byte_cnt=='d0 && slave_tx_cnt >= 'd2 && slave_tx_cnt < FBC_VOL_DATA_NUM*2+3)
        fbc_vol_rd_en <= #TCQ (arbitr_result[3]) && slave_tx_byte_en;
    else 
        fbc_vol_rd_en <= #TCQ 'd0;
end

always @(posedge clk_i) begin  // rd_en to slave_tx_byte need 1clk
    if(slave_tx_cnt > 'd0 && slave_tx_cnt < laser_rx_num + 1)
        laser_rx_rd_en <= #TCQ (arbitr_result[4]) && slave_tx_byte_en;
    else 
        laser_rx_rd_en <= #TCQ 'd0;
end

always @(posedge clk_i) begin  // rd_en to slave_tx_byte need 1clk
    if(spi_ack_cnt > 'd1 && spi_byte_cnt =='d2)begin
        spi_ack_rd_en[0] <= #TCQ (arbitr_result[5]) && slave_tx_byte_en;
        spi_ack_rd_en[1] <= #TCQ (arbitr_result[6]) && slave_tx_byte_en;
        spi_ack_rd_en[2] <= #TCQ (arbitr_result[7]) && slave_tx_byte_en;
    end
    else begin
        spi_ack_rd_en[0] <= #TCQ 'd0;
        spi_ack_rd_en[1] <= #TCQ 'd0;
        spi_ack_rd_en[2] <= #TCQ 'd0;
    end
end

reg [3:0] readback_cnt = 'd0;
always @(posedge clk_i) begin
    if(arbitr_result[9])begin
        if(readback_cnt < READBACK_NUM + 1)
            readback_cnt <= #TCQ readback_cnt + 1;
    end
    else 
        readback_cnt <= 'd0;
end

always @(posedge clk_i) begin
    if(arbitr_result[0])begin
        if(fbc_tx_cnt=='d0)
            slave_tx_byte <= #TCQ FBC_ACTUAL_TYPE[15:8];
        else if(fbc_tx_cnt=='d1)
            slave_tx_byte <= #TCQ FBC_ACTUAL_TYPE[7:0];
        else begin
            case(fbc_cnt[2:1])
            'b00 : slave_tx_byte <= #TCQ FBCi_dout[(2-fbc_byte_cnt)*8 +: 8];
            'b01 : slave_tx_byte <= #TCQ FBCr1_dout[(2-fbc_byte_cnt)*8 +: 8];
            'b10 : slave_tx_byte <= #TCQ FBCr2_dout[(2-fbc_byte_cnt)*8 +: 8];
            default : /*default*/;
            endcase
        end
    end
    else if((arbitr_result[1] || arbitr_result[2]) && slave_tx_cnt < FBC_BG_CALI_NUM)begin
        case(slave_tx_cnt)
            'd0 : slave_tx_byte <= #TCQ FBC_BG_CALI_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ FBC_BG_CALI_TYPE[7:0];
            default: slave_tx_byte <= #TCQ slave_tx_data[(FBC_BG_CALI_NUM - slave_tx_cnt - 1)*8 +: 8];
        endcase
    end
    else if((arbitr_result[8]) && slave_tx_cnt < MFPGA_VERSION_NUM)begin
        case(slave_tx_cnt)
            'd0 : slave_tx_byte <= #TCQ MFPGA_VERSION_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ MFPGA_VERSION_TYPE[7:0];
            default: slave_tx_byte <= #TCQ slave_tx_data[(MFPGA_VERSION_NUM - slave_tx_cnt - 1)*8 +: 8];
        endcase
    end
    else if(arbitr_result[3])begin
        case(slave_tx_cnt)
            'd0 : slave_tx_byte <= #TCQ FBC_VOL_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ FBC_VOL_TYPE[7:0];
            default:slave_tx_byte <= #TCQ fbc_vol_dout[(1-byte_cnt)*8 +: 8];
        endcase
    end
    else if(arbitr_result[4])begin
        case(slave_tx_cnt)
            'd0 : slave_tx_byte <= #TCQ LASER_UART_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ LASER_UART_TYPE[7:0];
            default:slave_tx_byte <= #TCQ laser_rx_dout[0 +: 8];
        endcase
    end
    else if(arbitr_result[5])begin
        case(spi_ack_cnt)
            'd0 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[7:0];
            default:slave_tx_byte <= #TCQ spi_ack_rd_dout[0][(3-spi_byte_cnt)*8 +: 8];
        endcase
    end
    else if(arbitr_result[6])begin
        case(spi_ack_cnt)
            'd0 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[7:0];
            default:slave_tx_byte <= #TCQ spi_ack_rd_dout[1][(3-spi_byte_cnt)*8 +: 8];
        endcase
    end
    else if(arbitr_result[7])begin
        case(spi_ack_cnt)
            'd0 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ SPI_SLAVE_ACK_TYPE[7:0];
            default:slave_tx_byte <= #TCQ spi_ack_rd_dout[2][(3-spi_byte_cnt)*8 +: 8];
        endcase
    end
    else if(arbitr_result[9] && readback_cnt<READBACK_NUM)begin
        case(readback_cnt)
            'd0 : slave_tx_byte <= #TCQ READBACK_TYPE[15:8];
            'd1 : slave_tx_byte <= #TCQ READBACK_TYPE[7:0];
            default:slave_tx_byte <= #TCQ readback_data_i[(4'd9-readback_cnt)*8 +: 8];
        endcase
    end
end

wire fbc_tx_byte_en ;
assign fbc_tx_byte_en = (fbc_tx_cnt>'d0) && (fbc_tx_cnt < FBC_ACTUAL_DATA_NUM + 2);
reg fbc_tx_byte_en_d = 'd0;
always @(posedge clk_i) fbc_tx_byte_en_d <= #TCQ fbc_tx_byte_en;

always @(*) begin
    if(arbitr_result[0])
        slave_tx_byte_en = fbc_tx_byte_en_d || fbc_tx_byte_en;
    else if(arbitr_result[1] || arbitr_result[2])
        slave_tx_byte_en = (slave_tx_cnt>'d0) && (slave_tx_cnt < FBC_BG_CALI_NUM + 1);
    else if(arbitr_result[8])
        slave_tx_byte_en = (slave_tx_cnt>'d0) && (slave_tx_cnt < MFPGA_VERSION_NUM + 1);
    else if(arbitr_result[3])
        slave_tx_byte_en = (slave_tx_cnt>'d0) && (slave_tx_cnt < FBC_VOL_DATA_NUM*2+3);
    else if(arbitr_result[4])
        slave_tx_byte_en = (slave_tx_cnt>'d0) && (slave_tx_cnt < laser_rx_num + 3);
    else if(arbitr_result[5])
        slave_tx_byte_en = (spi_ack_cnt>'d0) && (spi_ack_cnt < spi_slave_ack_num[0] + 3);
    else if(arbitr_result[6])
        slave_tx_byte_en = (spi_ack_cnt>'d0) && (spi_ack_cnt < spi_slave_ack_num[1] + 3);
    else if(arbitr_result[7])
        slave_tx_byte_en = (spi_ack_cnt>'d0) && (spi_ack_cnt < spi_slave_ack_num[2] + 3);
    else if(arbitr_result[9])
        slave_tx_byte_en = (readback_cnt>'d0) && (readback_cnt < READBACK_NUM + 1);
    else 
        slave_tx_byte_en = 'd0;
end

always @(posedge clk_i) begin
    if(arbitr_result[0])
        slave_tx_byte_num <= #TCQ FBC_ACTUAL_BYTE_NUM;
    if(arbitr_result[1] || arbitr_result[2])
        slave_tx_byte_num <= #TCQ FBC_BG_CALI_NUM;
    if(arbitr_result[8])
        slave_tx_byte_num <= #TCQ MFPGA_VERSION_NUM;
    if(arbitr_result[3])
        slave_tx_byte_num <= #TCQ FBC_VOL_BYTE_NUM;
    if(arbitr_result[4])
        slave_tx_byte_num <= #TCQ laser_rx_num + 2;
    if(arbitr_result[5])
        slave_tx_byte_num <= #TCQ {spi_slave_ack_num[0][5:0],2'b00} + 'd2;
    if(arbitr_result[6])
        slave_tx_byte_num <= #TCQ {spi_slave_ack_num[1][5:0],2'b00} + 'd2;
    if(arbitr_result[7])
        slave_tx_byte_num <= #TCQ {spi_slave_ack_num[2][5:0],2'b00} + 'd2;
    if(arbitr_result[9])
        slave_tx_byte_num <= #TCQ READBACK_NUM;
end

always @(posedge clk_i) begin
    arbitr_result_d0 <= #TCQ |arbitr_result;
    arbitr_result_d1 <= #TCQ arbitr_result_d0;
end

assign slave_tx_byte_num_en_o = arbitr_result_d0 && (~arbitr_result_d1);
assign slave_tx_byte_num_o    = slave_tx_byte_num;
assign slave_tx_byte_en_o     = slave_tx_byte_en;
assign slave_tx_byte_o        = slave_tx_byte;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


endmodule
