`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: songyuxin
// 
// Create Date: 2023/6/1 
// Design Name:  
// Module Name: bpsi_top_if_v2
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

module bpsi_top_if_v2#(
    parameter                   TCQ             = 0.1
)(
    input   wire                clk_sys_i                   ,
    input   wire                clk_h_i                     ,
    input   wire                rst_i                       ,
                
    input   wire    [2:0]       data_acq_en_i               , // motor enable signal
    input   wire                bg_data_acq_en_i            , // background sample. pulse
    input   wire                position_cali_en_i          , // test
    input   wire    [24:0]      position_aim_i              , // aim position
    input   wire    [22:0]      kp_i                        , // PID controller kp parameter
    input   wire    [1:0]       motor_freq_i                , // motor response frequency. 0:100Hz 1:200Hz 2:300Hz
    input   wire    [15:0]      fbc_bias_voltage_i          , // 
    input   wire    [15:0]      fbc_cali_uop_set_i          , // Uop set
        
    output  wire                motor_rd_en_o               , // read Ufeed en
    input   wire                motor_data_out_en_i         , // Ufeed en
    input   wire    [15:0]      motor_data_out_i            , // Ufeed
    output  wire                motor_data_in_en_o          , // Uop en
    output  wire    [15:0]      motor_Ufeed_latch_o         , // Ufeed from motor
    output  wire    [15:0]      motor_data_in_o             , // Uop to motor
        
    output  wire                cali_data_en_o              ,
    output  wire    [23:0]      cali_data_a_o               ,
    output  wire    [23:0]      cali_data_b_o               ,
    
    // actual voltage
    output  wire                data_out_en_o               ,
    output  wire    [23:0]      data_out_a_o                ,
    output  wire    [23:0]      data_out_b_o                ,
    
    // background voltage. dark current * R
    output  wire                bg_data_en_o                ,
    output  wire    [23:0]      bg_data_a_o                 ,
    output  wire    [23:0]      bg_data_b_o                 ,

    // spi info
    output  wire                MSPI_CLK                    ,
    output  wire                MSPI_MOSI                   ,
    input   wire                SSPI_CLK                    ,
    input   wire                SSPI_MOSI                   
);
//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam                              SPI_CLK_DIVIDER         = 6  ; // SPI Clock Control / Divid
localparam                              SPI_MASTER_WIDTH        = 64 ; // master spi data width
localparam                              SPI_SLAVE_WIDTH         = 48 ; // slave spi data width

localparam                              BG_POS_IDLE             = 3'd0;
localparam                              BG_WAIT                 = 3'd1;
localparam                              BG_SUM                  = 3'd2;
localparam                              BG_AVG                  = 3'd3;
localparam                              POS_WAIT                = 3'd4;
localparam                              POS_SUM                 = 3'd5;
localparam                              POS_AVG                 = 3'd6;
localparam                              BG_POS_FINISH           = 3'd7;

`ifdef SIMULATE    // simulate use 100 us + 300us
localparam                              USELESS_LENGTH          = 16'd10; 
localparam                              BG_SUM_LENGTH           = 16'd30; 
localparam                              POSITION_SUM_LENGTH     = 16'd30; 
`else
localparam                              USELESS_LENGTH          = 16'd499;  // first 0.5s data is useless
localparam                              BG_SUM_LENGTH           = 16'd8191; // 8s
localparam                              POSITION_SUM_LENGTH     = 16'd8191; // 8s
`endif //SIMULATE

localparam                              DATA_WIDTH              = 'd24;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg         [2:0]                       bg_pos_state         = BG_POS_IDLE;
reg         [2:0]                       bg_pos_state_next    = BG_POS_IDLE;

reg                                     data_acq_en_d0          ;
reg                                     data_acq_en_d1          ;
// reg                                     bg_data_acq_en_d0       ;
// reg                                     bg_data_acq_en_d1       ;
// reg                                     position_test_en_d0     ;
// reg                                     position_test_en_d1     ;
reg                                     set_data_acq_en         = 'd0;
// reg                                     set_bg_data_acq_en      = 'd0;
// reg                                     set_position_test_en    = 'd0;

reg                                     mspi_wr_en              = 'd0;   
reg         [SPI_MASTER_WIDTH-1:0]      mspi_wr_data            = 'd0; 

reg         [16-1:0]                    state_data_cnt          = 'd0;
(* use_dsp = "yes"*)reg  signed [40-1:0]                    state_data_a_sum            = 'd0;
(* use_dsp = "yes"*)reg  signed [40-1:0]                    state_data_b_sum            = 'd0;
(* use_dsp = "yes"*)reg  signed [17-1:0]                    state_data_a_sum_l          = 'd0;
(* use_dsp = "yes"*)reg  signed [17-1:0]                    state_data_b_sum_l          = 'd0;
(* use_dsp = "yes"*)reg  signed [17-1:0]                    state_data_a_sum_h          = 'd0;
(* use_dsp = "yes"*)reg  signed [17-1:0]                    state_data_b_sum_h          = 'd0;

reg         [24-1:0]                    state_data_a_avg        = 'd0;
reg         [24-1:0]                    state_data_b_avg        = 'd0;
reg         [ 3-1:0]                    avg_beat                = 'd1;
// reg                                     position_cali_out_en    = 'd0;

reg                                     bg_data_en              = 'd0;
reg         [24-1:0]                    bg_data_a               = 'd0;
reg         [24-1:0]                    bg_data_b               = 'd0;
reg                                     data_out_en             = 'd0;
reg         [24-1:0]                    data_out_a              = 'd0;
reg         [24-1:0]                    data_out_b              = 'd0;
reg                                     cali_data_en            = 'd0;
reg         [24-1:0]                    cali_data_a             = 'd0;
reg         [24-1:0]                    cali_data_b             = 'd0;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                                    sspi_rd_vld                 ;  
wire        [SPI_SLAVE_WIDTH-1:0]       sspi_rd_data                ; 

wire                                    actual_data_vld             ;
wire signed [24-1:0]                    actual_data_a               ;
wire signed [24-1:0]                    actual_data_b               ;

// wire                                    pose_position_out_en        ;
wire                                    bg_data_acq_en_sync         ;
wire                                    position_cali_en_sync       ;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
`ifdef SIMULATE
reg                             sspi_rd_vld_sim = 'd0; 
reg  [SPI_SLAVE_WIDTH-1:0]      sspi_rd_data_sim = {24'd8192,24'd8192};
reg  [32-1:0]                   sim_cnt  = 'd0;
always @(posedge clk_h_i) begin
    if(sim_cnt == 'd3_000)begin    // simulate 10 us
        sspi_rd_vld_sim <= #TCQ 'd1;
        sspi_rd_data_sim[23:0] <= #TCQ sspi_rd_data_sim[23:0] + 1 ;
        sspi_rd_data_sim[47:24] <= #TCQ sspi_rd_data_sim[47:24] + 2 ;
        sim_cnt <= #TCQ 'd0;
    end
    else begin
        sim_cnt <= #TCQ sim_cnt + 1;
        sspi_rd_vld_sim <= #TCQ 'd0;
    end
end
assign sspi_rd_vld  = sspi_rd_vld_sim; 
assign sspi_rd_data = sspi_rd_data_sim;

`else
bspi_ctrl #(
    .SPI_CLK_DIVIDER                ( SPI_CLK_DIVIDER       ), // SPI Clock Control / Divid
    .SPI_MASTER_WIDTH               ( SPI_MASTER_WIDTH      ), // master spi data width
    .SPI_SLAVE_WIDTH                ( SPI_SLAVE_WIDTH       )  // slave spi data width

)bspi_ctrl_inst(
    // clk & rst
    .clk_i                          ( clk_h_i               ),
    .rst_i                          ( rst_i                 ),
    
    .mspi_wr_en_i                   ( mspi_wr_en            ),
    .mspi_wr_data_i                 ( mspi_wr_data          ),
    .sspi_rd_vld_o                  ( sspi_rd_vld           ),
    .sspi_rd_data_o                 ( sspi_rd_data          ),
    // .fbc_spi_ready_i                ( fbc_ready             ),
    // bspi info
    .MSPI_CLK                       ( MSPI_CLK              ),
    .MSPI_MOSI                      ( MSPI_MOSI             ),
    .SSPI_CLK                       ( SSPI_CLK              ),
    .SSPI_MOSI                      ( SSPI_MOSI             )
);
`endif //SIMULATE

xpm_cdc_pulse #(
    .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
    .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
    .REG_OUTPUT(0),     // DECIMAL; 0=disable registered output, 1=enable registered output
    .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
    .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_bg_inst (
    .dest_pulse(bg_data_acq_en_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                             // transfer is correctly initiated on src_pulse input. This output is
                             // combinatorial unless REG_OUTPUT is set to 1.

    .dest_clk(clk_h_i),     // 1-bit input: Destination clock.
    .dest_rst('d0),     // 1-bit input: optional; required when RST_USED = 1
    .src_clk(clk_sys_i),       // 1-bit input: Source clock.
    .src_pulse(bg_data_acq_en_i),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
                             // destination clock domain. The minimum gap between each pulse transfer must be
                             // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
                             // between the falling edge of a src_pulse to the rising edge of the next
                             // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
                             // will generate a pulse the size of one dest_clk period in the destination
                             // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
                             // src_rst and/or dest_rst are asserted.

    .src_rst('d0)        // 1-bit input: optional; required when RST_USED = 1
);

xpm_cdc_pulse #(
    .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
    .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
    .REG_OUTPUT(0),     // DECIMAL; 0=disable registered output, 1=enable registered output
    .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
    .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_cali_inst (
    .dest_pulse(position_cali_en_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                             // transfer is correctly initiated on src_pulse input. This output is
                             // combinatorial unless REG_OUTPUT is set to 1.

    .dest_clk(clk_h_i),     // 1-bit input: Destination clock.
    .dest_rst('d0),     // 1-bit input: optional; required when RST_USED = 1
    .src_clk(clk_sys_i),       // 1-bit input: Source clock.
    .src_pulse(position_cali_en_i),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
                             // destination clock domain. The minimum gap between each pulse transfer must be
                             // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
                             // between the falling edge of a src_pulse to the rising edge of the next
                             // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
                             // will generate a pulse the size of one dest_clk period in the destination
                             // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
                             // src_rst and/or dest_rst are asserted.

    .src_rst('d0)        // 1-bit input: optional; required when RST_USED = 1
);


// background data xpm cdc handshake module.
reg [48-1:0]    bg_data_ff      = 'd0;
reg [2-1:0]     bg_src_ff       = 'd0;
reg             bg_handshake_ff = 'd0;
reg [2-1:0]     bg_dest_ff      = 'd0;

always @(posedge clk_h_i) begin
    if(avg_beat[2] & (bg_pos_state==BG_AVG))begin
        bg_data_ff <= {state_data_a_avg[23:0],state_data_b_avg[23:0]};
    end
end

always @(posedge clk_h_i) begin
    bg_src_ff <= {bg_src_ff[0],(avg_beat[2] & (bg_pos_state==BG_AVG))};
end

always @(posedge clk_h_i) begin
    if(bg_src_ff==2'b10)
        bg_handshake_ff <= 'd1;
    else if(bg_dest_ff==2'b11)
        bg_handshake_ff <= 'd0;
end

always @(posedge clk_sys_i) begin
    bg_dest_ff <= {bg_dest_ff[0],bg_handshake_ff};
end

always @(posedge clk_sys_i) begin
    if(bg_dest_ff==2'b01)begin
        {bg_data_a[23:0],bg_data_b[23:0]} <= bg_data_ff;
    end
end

always @(posedge clk_sys_i) begin
    bg_data_en <= bg_dest_ff==2'b01;
end

// cali data xpm cdc handshake module.
reg [48-1:0]    cali_data_ff      = 'd0;
reg [2-1:0]     cali_src_ff       = 'd0;
reg             cali_handshake_ff = 'd0;
reg [2-1:0]     cali_dest_ff      = 'd0;

always @(posedge clk_h_i) begin
    if(avg_beat[2] & (bg_pos_state==POS_AVG))begin
        cali_data_ff <= {state_data_a_avg[23:0],state_data_b_avg[23:0]};
    end
end

always @(posedge clk_h_i) begin
    cali_src_ff <= {cali_src_ff[0],(avg_beat[2] & (bg_pos_state==POS_AVG))};
end

always @(posedge clk_h_i) begin
    if(cali_src_ff==2'b10)
        cali_handshake_ff <= 'd1;
    else if(cali_dest_ff==2'b11)
        cali_handshake_ff <= 'd0;
end

always @(posedge clk_sys_i) begin
    cali_dest_ff <= {cali_dest_ff[0],cali_handshake_ff};
end

always @(posedge clk_sys_i) begin
    if(cali_dest_ff==2'b01)begin
        {cali_data_a[23:0],cali_data_b[23:0]} <= cali_data_ff;
    end
end

always @(posedge clk_sys_i) begin
    cali_data_en <= cali_dest_ff==2'b01;
end

// actual data xpm cdc handshake module.
reg [48-1:0]    actual_data_ff      = 'd0;
reg [2-1:0]     actual_src_ff       = 'd0;
reg             actual_handshake_ff = 'd0;
reg [2-1:0]     actual_dest_ff      = 'd0;

always @(posedge clk_h_i) begin
    if(actual_data_vld)begin
        actual_data_ff <= {actual_data_a[23:0],actual_data_b[23:0]};
    end
end

always @(posedge clk_h_i) begin
    actual_src_ff <= {actual_src_ff[0],actual_data_vld};
end

always @(posedge clk_h_i) begin
    if(actual_src_ff==2'b10)
        actual_handshake_ff <= 'd1;
    else if(actual_dest_ff==2'b11)
        actual_handshake_ff <= 'd0;
end

always @(posedge clk_sys_i) begin
    actual_dest_ff <= {actual_dest_ff[0],actual_handshake_ff};
end

always @(posedge clk_sys_i) begin
    if(actual_dest_ff==2'b01)begin
        {data_out_a[23:0],data_out_b[23:0]} <= actual_data_ff;
    end
end

always @(posedge clk_sys_i) begin
    data_out_en <= actual_dest_ff==2'b01;
end


// xpm_cdc_pulse #(
//    .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
//    .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
//    .REG_OUTPUT(0),     // DECIMAL; 0=disable registered output, 1=enable registered output
//    .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
//    .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
// )
// xpm_cdc_pulse_cali_out_inst (
//    .dest_pulse(pose_position_out_en), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
//                             // transfer is correctly initiated on src_pulse input. This output is
//                             // combinatorial unless REG_OUTPUT is set to 1.

//    .dest_clk(clk_sys_i),     // 1-bit input: Destination clock.
//    .dest_rst(1'b0),     // 1-bit input: optional; required when RST_USED = 1
//    .src_clk(clk_h_i),       // 1-bit input: Source clock.
//    .src_pulse(position_cali_out_en),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
//                             // destination clock domain. The minimum gap between each pulse transfer must be
//                             // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
//                             // between the falling edge of a src_pulse to the rising edge of the next
//                             // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
//                             // will generate a pulse the size of one dest_clk period in the destination
//                             // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
//                             // src_rst and/or dest_rst are asserted.

//    .src_rst(1'b0)        // 1-bit input: optional; required when RST_USED = 1
// );


PID_control #(
    .DATA_WIDTH                 ( DATA_WIDTH                ) // actual and backgroud data width

)PID_control_inst(
    // clk & rst
    .clk_i                      ( clk_sys_i                 ),
    .rst_i                      ( rst_i                     ),

    .data_acq_en_i              ( data_acq_en_i             ), // motor control enable
    .motor_freq_i               ( motor_freq_i              ), // motor close freq  0:100Hz 1:200Hz 2:300Hz
    .kp_i                       ( kp_i                      ), // parameter kp
    .position_aim_i             ( position_aim_i            ), // aim position
    .fbc_bias_voltage_i         ( fbc_bias_voltage_i        ), // bais voltage
    .fbc_cali_uop_set_i         ( fbc_cali_uop_set_i        ), // cali voltage
    .actual_data_en_i           ( data_out_en               ),
    .actual_data_a_i            ( data_out_a                ),
    .actual_data_b_i            ( data_out_b                ),
    .bg_data_a_i                ( bg_data_a                 ),
    .bg_data_b_i                ( bg_data_b                 ),

    // .position_cali_en_i         ( position_cali_en_i        ),
    // .pose_position_en_i         ( pose_position_out_en      ),
    // .position_actual_avg_en_o   ( position_actual_avg_en_o  ),
    // .position_actual_avg_o      ( position_actual_avg_o     ),

    .motor_Ufeed_en_i           ( motor_data_out_en_i       ),
    .motor_Ufeed_i              ( motor_data_out_i          ),
    .motor_data_in_en_o         ( motor_data_in_en_o        ),
    .motor_rd_en_o              ( motor_rd_en_o             ),
    .motor_Ufeed_latch_o        ( motor_Ufeed_latch_o       ),
    .motor_data_in_o            ( motor_data_in_o           )
);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// spi tx
always @(posedge clk_h_i)begin
    data_acq_en_d0      <= (|data_acq_en_i)     ;
    data_acq_en_d1      <= data_acq_en_d0       ;
    // bg_data_acq_en_d0   <= bg_data_acq_en_i     ;
    // bg_data_acq_en_d1   <= bg_data_acq_en_d0    ;
    // position_test_en_d0 <= position_cali_en_i   ;
    // position_test_en_d1 <= position_test_en_d0  ;
end

always @(posedge clk_h_i)begin
    // if(rst_i) begin    
    //     set_data_acq_en      <= 1'b0;
        // set_bg_data_acq_en   <= 1'b0;
        // set_position_test_en <= 1'b0;
    // end
    // else begin
        set_data_acq_en      <= data_acq_en_d1 ^ data_acq_en_d0;
        // set_bg_data_acq_en   <= bg_data_acq_en_d1 ^ bg_data_acq_en_d0;
        // set_position_test_en <= position_test_en_d1 ^ position_test_en_d0;
    // end
end

always @(posedge clk_h_i) begin
    if(set_data_acq_en)begin
        mspi_wr_en <= 'd1;
        mspi_wr_data <= {16'h55aa,16'h0001,15'h0,data_acq_en_d1,16'd0};
    end
    // else if(set_bg_data_acq_en)begin
    //     mspi_wr_en <= 'd1;
    //     mspi_wr_data <= {16'h55aa,16'h0001,15'h0,bg_data_acq_en_d1,16'd0};
    // end
    // else if(set_position_test_en)begin
    //     mspi_wr_en <= 'd1;
    //     mspi_wr_data <= {16'h55aa,16'h0001,15'h0,position_test_en_d1,16'd0};
    // end
    else begin
        mspi_wr_en <= 'd0;
    end
end

// spi rx
assign actual_data_vld = sspi_rd_vld;
assign actual_data_a   = sspi_rd_data[47:24];
assign actual_data_b   = sspi_rd_data[23:0];


// background noise calculation
reg [1:0] actual_data_vld_d = 'd0;
always @(posedge clk_h_i) begin
    actual_data_vld_d <= {actual_data_vld_d[0],actual_data_vld};
end
wire [2:0] actual_data_vld_logic = {actual_data_vld_d,actual_data_vld};

// FSM control
always @(posedge clk_h_i) begin
    if(rst_i)
        bg_pos_state <= BG_POS_IDLE;
    else 
        bg_pos_state <= bg_pos_state_next;
end

always @(*) begin
    bg_pos_state_next = bg_pos_state;
    case (bg_pos_state)
        BG_POS_IDLE: begin
            if(bg_data_acq_en_sync)
                bg_pos_state_next = BG_WAIT;
            else if(position_cali_en_sync && data_acq_en_i=='d3)
                bg_pos_state_next = POS_WAIT;
        end

        BG_WAIT : begin
            if(actual_data_vld_logic[2] && (state_data_cnt == USELESS_LENGTH))
                bg_pos_state_next = BG_SUM;
        end

        BG_SUM: begin
            if(actual_data_vld_logic[2] && (state_data_cnt == USELESS_LENGTH + BG_SUM_LENGTH))
                bg_pos_state_next = BG_AVG;
        end
        BG_AVG: begin
            if(avg_beat[2])
                bg_pos_state_next = BG_POS_FINISH;
        end
        
        POS_WAIT : begin
            if(actual_data_vld_logic[2] && (state_data_cnt == USELESS_LENGTH))
                bg_pos_state_next = POS_SUM;
        end

        POS_SUM: begin
            if(actual_data_vld_logic[2] && (state_data_cnt == USELESS_LENGTH + BG_SUM_LENGTH))
                bg_pos_state_next = POS_AVG;
        end
        POS_AVG: begin
            if(avg_beat[2])
                bg_pos_state_next = BG_POS_FINISH;
        end

        BG_POS_FINISH : begin
            // if(~bg_data_acq_en_d1 && ~position_test_en_d1)
                bg_pos_state_next = BG_POS_IDLE;
        end
        default: bg_pos_state_next = BG_POS_IDLE;
    endcase
end

always @(posedge clk_h_i) begin
    if(bg_pos_state==BG_POS_IDLE || bg_pos_state==BG_POS_FINISH)
        state_data_cnt <= 'd0;
    else if(actual_data_vld)
        state_data_cnt <= state_data_cnt + 1;
end

// calculate sum and average
always @(posedge clk_h_i) begin
    case (bg_pos_state)
        BG_POS_IDLE: begin
            state_data_a_sum <= 'd0;
            state_data_b_sum <= 'd0;
        end
        BG_SUM,
        POS_SUM : begin
            if(actual_data_vld_logic[0])begin
                state_data_a_sum_l[16:0] <= state_data_a_sum[15:0] + actual_data_a[15:0];
                state_data_b_sum_l[16:0] <= state_data_b_sum[15:0] + actual_data_b[15:0];
            end
            else if(actual_data_vld_logic[1])begin
                state_data_a_sum_h[16:0] <= state_data_a_sum[31:16] + {{'d8{actual_data_a[23]}},actual_data_a[23:16]} + state_data_a_sum_l[16];
                state_data_b_sum_h[16:0] <= state_data_b_sum[31:16] + {{'d8{actual_data_b[23]}},actual_data_b[23:16]} + state_data_b_sum_l[16];
            end
            else if(actual_data_vld_logic[2])begin
                state_data_a_sum <= {(state_data_a_sum[39:32] + {'d8{actual_data_a[23]}} + state_data_a_sum_h[16]),state_data_a_sum_h[15:0],state_data_a_sum_l[15:0]};
                state_data_b_sum <= {(state_data_b_sum[39:32] + {'d8{actual_data_b[23]}} + state_data_b_sum_h[16]),state_data_b_sum_h[15:0],state_data_b_sum_l[15:0]};
            end
        end
        BG_AVG,
        POS_AVG : begin
            if(avg_beat[0])begin
                state_data_a_sum <= {{'d13{state_data_a_sum[39]}},state_data_a_sum[39:13]};
                state_data_b_sum <= {{'d13{state_data_b_sum[39]}},state_data_b_sum[39:13]};
            end
            else if(avg_beat[1])begin
                state_data_a_avg <= state_data_a_sum[23:0];
                state_data_b_avg <= state_data_b_sum[23:0];
            end
        end
        default: /*default*/;
    endcase
end

always @(posedge clk_h_i) begin
    if(bg_pos_state==BG_POS_IDLE)
        avg_beat <= 'd1;
    else if(bg_pos_state==BG_AVG || bg_pos_state==POS_AVG)
        avg_beat <= {avg_beat[1:0],1'b0};
end

// always @(posedge clk_h_i) begin
//     if(bg_pos_state==BG_POS_IDLE)
//         position_cali_out_en <= 'd0;
//     else if(bg_pos_state==POS_AVG && avg_beat[1])
//         position_cali_out_en <= 'd1;
// end


assign bg_data_en_o     = bg_data_en;
assign bg_data_a_o      = bg_data_a[23:0];
assign bg_data_b_o      = bg_data_b[23:0];

assign data_out_en_o    = data_out_en;
assign data_out_a_o     = data_out_a[23:0];
assign data_out_b_o     = data_out_b[23:0];

assign cali_data_en_o   = cali_data_en;
assign cali_data_a_o    = cali_data_a[23:0];
assign cali_data_b_o    = cali_data_b[23:0];

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<




endmodule