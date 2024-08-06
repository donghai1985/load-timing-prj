`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/30
// Design Name: 
// Module Name: command_map
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
// `define FBC_OFF


module command_map #(
    parameter                   TCQ           = 0.1,
    parameter                   COMMAND_WIDTH = 16,
    parameter                   COMMAND_LENG  = 16

)(
    // clk & rst
    input   wire                clk_sys_i               ,
    input   wire                rst_i                   ,
    // ethernet interface for message data
    input   wire                slave_rx_data_vld_i     ,
    input   wire    [7:0]       slave_rx_data_i         ,
    // comm info
    output  wire    [2:0]       data_acq_en_o           ,
    output  wire                bg_data_acq_en_o        ,
    output  wire    [24:0]      position_arm_o          ,
    output  wire    [22:0]      kp_o                    ,
    output  wire    [22:0]      ki_o                    ,
    output  wire    [22:0]      kd_o                    ,
    output  wire    [3:0]       motor_freq_o            ,
    output  wire                bpsi_position_en_o      ,
    output  wire    [15:0]      fbc_bias_voltage_o      ,
    output  wire    [15:0]      fbc_cali_uop_set_o      ,
    input   wire    [15:0]      motor_Ufeed_latch_i     ,
    input   wire    [15:0]      motor_data_in_i         ,

    output  wire                eds_power_en_o          ,
    output  wire                eds_frame_en_o          ,
    output  wire                eds_test_en_o           ,
    output  wire    [32-1:0]    eds_texp_time_o         ,
    output  wire    [32-1:0]    eds_frame_to_frame_time_o,
    output  wire    [32-1:0]    laser_uart_data_o       ,
    output  wire                laser_uart_vld_o        ,

    output  wire                encode_sim_en_o         ,
    input   wire                scan_finish_comm_i      ,
    output  wire                scan_finish_comm_ack_o  ,
    input   wire                scan_error_comm_i       ,
    output  wire                scan_soft_reset_o       ,
    output  wire                real_scan_start_o       ,
    output  wire    [3-1:0]     real_scan_sel_o         ,
    output  wire    [32-1:0]    x_start_encode_o        ,
    output  wire    [32-1:0]    x_end_encode_o          ,
    output  wire    [32-1:0]    plc_x_encode_o          ,
    output  wire                plc_x_encode_en_o       ,
    input   wire                fbc_close_loop_i        ,
    input   wire                fbc_open_loop_i         ,
    output  wire    [25-1:0]    position_pid_thr_o      ,

    // timing to pmt communication
    output  wire    [32-1:0]    pmt_master_spi_data_o   ,
    output  wire                pmt_master_spi_vld_o    ,
    // PMT adc start
    output  wire    [32-1:0]    pmt_adc_start_data_o    ,
    output  wire                pmt_adc_start_vld_o     ,
    output  wire    [32-1:0]    pmt_adc_start_hold_o    ,

    // mfpga version read
    output  wire                rd_mfpga_version_o      ,

    output  wire                soft_fast_shutter_set_o ,
    output  wire                soft_fast_shutter_en_o  ,
    input   wire                laser_fast_shutter_i    ,
    input   wire    [32-1:0]    fast_shutter_act_time_i ,
    output  wire                FBC_fifo_rst_o          ,

    output  wire    [64-1:0]    readback_data_o         ,
    output  wire                readback_vld_o          ,

    // overload register
    output  wire                overload_motor_en_o     ,
    output  wire    [15:0]      overload_ufeed_thre_o   ,
    input   wire    [31:0]      overload_pid_result_i   ,
    output  wire                pmt_encode_align_rst_o  ,
    output  wire                eds_encode_align_rst_o  ,
    output  wire    [32-1:0]    pmt_encode_align_set_o  ,
    output  wire    [32-1:0]    eds_encode_align_set_o  ,

    output  wire                x_encode_zero_calib_o   ,
    input   wire    [31:0]      pmt_encode_w_i          ,
    input   wire    [31:0]      pmt_encode_x_i          ,
    output  wire    [31:0]      pmt_precise_encode_x_offset_o,

    input   wire    [3-1:0]     scan_state_i            ,

    input   wire    [32-1:0]    eds_pack_cnt_1_i        ,
    input   wire    [32-1:0]    encode_pack_cnt_1_i     ,
    input   wire    [32-1:0]    eds_pack_cnt_2_i        ,
    input   wire    [32-1:0]    encode_pack_cnt_2_i     ,
    input   wire    [32-1:0]    eds_pack_cnt_3_i        ,
    input   wire    [32-1:0]    encode_pack_cnt_3_i     ,

    // output  wire                encode_interval_rst_o   ,
    // input   wire    [32-1:0]    encode_interval_max_i   ,
    // input   wire    [32-1:0]    encode_w_diff_max_i     ,
    // input   wire    [32-1:0]    pmt_encode_w_diff_max_i ,
    // input   wire    [32-1:0]    delta_w_encode_acce_i   ,
    
    // output  wire    [32-1:0]    pmt_w_encode_thr_o      ,
    // output  wire    [32-1:0]    acs_w_encode_thr_o      ,
    // output  wire                pmt_encode_rd_en_o      ,
    // input   wire    [32-1:0]    pmt_x_encode_i          ,
    // input   wire    [32-1:0]    pmt_w_encode_i          ,
    // input   wire    [2-1:0]     pmt_fifo_state_i        ,
    // output  wire                acs_encode_rd_en_o      ,
    // input   wire    [32-1:0]    acs_x_encode_i          ,
    // input   wire    [32-1:0]    acs_w_encode_i          ,
    // input   wire    [2-1:0]     acs_fifo_state_i        ,

    output  wire                debug_info
);


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>






//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg     [16-1:0]                command_sel                 = 'd0;
reg                             command_state               = 'd0;
reg     [COMMAND_LENG-1:0]      command_addr                = 'd0;
reg     [32-1:0]                command_data                = 'd0;
reg                             slave_rx_data_vld_d         = 'd0;

reg     [3-1:0]                 data_acq_en                 = 'd0;
reg                             bg_data_acq_en              = 'd0;
reg     [24:0]                  position_arm                = 'd0;
reg     [22:0]                  kp                          = 'h10_0000;
reg     [22:0]                  ki                          = 'h0000;
reg     [22:0]                  kd                          = 'h0000;
reg     [3:0]                   motor_freq                  = 'd0;
reg                             bpsi_position_en            = 'd0;
reg     [15:0]                  fbc_bias_voltage            = 'd0;       // default = 13107/65535*4.096 = 0.8192V
reg     [15:0]                  fbc_cali_uop_set            = 'd0;       // default = 13107/65535*4.096 = 0.8192V
reg     [32-1:0]                laser_uart_data             = 'd0;
reg                             laser_uart_vld              = 'd0;
reg     [32-1:0]                pmt_master_spi_data         = 'd0;
reg                             pmt_master_spi_vld          = 'd0;
reg     [32-1:0]                pmt_adc_start_data          = 'd0;
reg                             pmt_adc_start_vld           = 'd0;
reg     [32-1:0]                pmt_adc_start_hold          = 'd60000;
reg                             rd_mfpga_version            = 'd0;
reg                             FBC_fifo_rst                = 'd0;
reg                             eds_power_en                = 'd0;
reg                             eds_frame_en                = 'd0;
reg                             eds_test_en                 = 'd0;
reg                             scan_soft_reset             = 'd0;
reg                             scan_soft_reset_d           = 'd0;
reg                             scan_soft_reset_pose        = 'd0;
reg     [32-1:0]                real_scan_command           = 'd0;
reg     [32-1:0]                real_scan_command_d         = 'd0;
reg     [3-1:0]                 real_scan_sel               = 'd0;
reg                             real_scan_start             = 'd0;
reg     [32-1:0]                x_start_encode              = 'd0;
reg     [32-1:0]                x_end_encode                = 'hff;
reg     [25-1:0]                position_pid_thr            = 'd2097;   // 0xFFFFF * 10 / 5000 
reg                             fast_shutter_set            = 'd0;
reg                             fast_shutter_en             = 'd0;
reg     [32-1:0]                eds_texp_time               = 'd0;
reg     [32-1:0]                eds_frame_to_frame_time     = 'd0;
reg                             pmt_encode_align_rst        = 'd0;
reg     [32-1:0]                pmt_encode_align_set        = 'd0;
reg                             eds_encode_align_rst        = 'd0;
reg     [32-1:0]                eds_encode_align_set        = 'd0;
reg                             encode_sim_en               = 'd0;
reg                             encode_interval_rst         = 'd0;
reg                             pmt_encode_rd_en            = 'd0;
reg                             acs_encode_rd_en            = 'd0;
reg     [32-1:0]                pmt_w_encode_thr            = 'd100;
reg     [32-1:0]                acs_w_encode_thr            = 'd100;
reg                             scan_finish_comm_ack        = 'd0;
reg     [32-1:0]                plc_x_encode                = 'd0;
reg                             plc_x_encode_en             = 'd0;


// overload register
reg     [32-1:0]                overload_motor_set          = 'd13107;  // default = 13107/65535*4.096 = 0.8192V
reg                             x_encode_zero_calib         = 'd0;
reg     [32-1:0]                pmt_precise_encode_x_offset = 'h300000; // β

reg     [32-1:0]                readback_reg                = 'd0;
reg                             readback_en                 = 'd0;
reg     [2-1:0]                 readback_cnt                = 'd2;
reg     [32-1:0]                register_data               = 'd0;
reg     [64-1:0]                readback_data               = 'd0;
reg                             readback_vld                = 'd0;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                            slave_rx_start  ;
wire                            command_en      ;
wire    [COMMAND_WIDTH-1:0]     command         ;
wire                            command_data_vld;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// check command and command data 
always @(posedge clk_sys_i) command_data        <= #TCQ {command_data[23:0],slave_rx_data_i[7:0]};
always @(posedge clk_sys_i) slave_rx_data_vld_d <= #TCQ slave_rx_data_vld_i;

assign slave_rx_start = ~slave_rx_data_vld_d && slave_rx_data_vld_i;

always @(posedge clk_sys_i) begin
    if(slave_rx_start || command_en)begin
        command_addr <= #TCQ 'd0;
    end
    else if(slave_rx_data_vld_d)begin
        command_addr <= #TCQ command_addr + 1;
    end
end

assign command_en   = (command_addr=='d1) && slave_rx_data_vld_d && (~command_state);
assign command      = command_data[15:0];

always @(posedge clk_sys_i) begin
    if(slave_rx_data_vld_i)begin
        if(command_en) 
            command_state <= #TCQ 'd1;
    end
    else begin
        command_state <= #TCQ 'd0;
    end
end

assign command_data_vld = (command_addr[1:0]=='b11) && command_state;

always @(posedge clk_sys_i) begin
    if(command_en)begin
        command_sel <= #TCQ command[15:0];
    end
end

// write register
always @(posedge clk_sys_i) begin
    if(command_data_vld)begin
        case (command_sel)
            'h0100: rd_mfpga_version        <= #TCQ 'd1                 ;

            // FBC register
            'h010E: FBC_fifo_rst            <= #TCQ command_data[0]     ;
            'h0116: bg_data_acq_en          <= #TCQ command_data[0]     ;
            'h0117: position_arm            <= #TCQ command_data[24:0]  ;
            'h0118: kp                      <= #TCQ command_data[22:0]  ;
            'h0119: motor_freq              <= #TCQ command_data[3:0]   ;
            'h011a: bpsi_position_en        <= #TCQ command_data[0]     ;
            'h011b: fbc_bias_voltage        <= #TCQ command_data[15:0]  ;
            'h011c: fbc_cali_uop_set        <= #TCQ command_data[15:0]  ;
            'h011d: ki                      <= #TCQ command_data[22:0]  ;
            'h011e: kd                      <= #TCQ command_data[22:0]  ;

            // scan register
            'h0120: eds_power_en            <= #TCQ command_data[0]     ;
            'h0121: eds_frame_en            <= #TCQ command_data[0]     ;
            'h0125: pmt_adc_start_hold      <= #TCQ command_data        ;
            'h0126: eds_test_en             <= #TCQ command_data[0]     ;
            'h0127: scan_soft_reset         <= #TCQ command_data[0]     ;
            'h0128: real_scan_command       <= #TCQ command_data        ;
            'h0129: x_start_encode          <= #TCQ command_data        ;
            'h012a: x_end_encode            <= #TCQ command_data        ;
            'h012b: position_pid_thr        <= #TCQ command_data[24:0]  ;
            'h012c: fast_shutter_set        <= #TCQ command_data[0]     ;

            // encode
            'h0130: x_encode_zero_calib     <= #TCQ command_data[0]     ;
            'h0131: pmt_precise_encode_x_offset <= #TCQ command_data    ;
            'h0133: pmt_encode_align_set    <= #TCQ command_data        ;
            'h0134: eds_encode_align_set    <= #TCQ command_data        ;
            'h0135: encode_sim_en           <= #TCQ command_data[0]     ;
            
            'h0137: scan_finish_comm_ack    <= #TCQ command_data[0]     ;
            'h0138: plc_x_encode            <= #TCQ command_data        ;
            'h0139: plc_x_encode_en         <= #TCQ command_data        ;
            // overload register
            'h0201: overload_motor_set      <= #TCQ command_data        ;

            // 'h0406: encode_interval_rst     <= #TCQ command_data        ;
            // 'h0410: pmt_encode_rd_en        <= #TCQ command_data        ;
            // 'h0414: acs_encode_rd_en        <= #TCQ command_data        ;
            // 'h0418: pmt_w_encode_thr        <= #TCQ command_data        ;
            // 'h0419: acs_w_encode_thr        <= #TCQ command_data        ;
            default: /*default*/;
        endcase
    end
    else begin
        rd_mfpga_version    <= #TCQ 'd0;
        bg_data_acq_en      <= #TCQ 'd0;
        bpsi_position_en    <= #TCQ 'd0;
        // real_scan_start     <= #TCQ 'd0;
        scan_finish_comm_ack<= #TCQ 'd0;
        // scan_soft_reset     <= #TCQ 'd0;
        // x_encode_zero_calib <= #TCQ 'd0;
        // pmt_encode_rd_en    <= #TCQ 'd0;
        // acs_encode_rd_en    <= #TCQ 'd0;
    end
end

always @(posedge clk_sys_i) begin
    if(command_sel=='h012c && command_data_vld)
        fast_shutter_en <= #TCQ 'd1;
    else 
        fast_shutter_en <= #TCQ 'd0;
end

always @(posedge clk_sys_i) begin
    real_scan_command_d <= #TCQ real_scan_command;
    real_scan_start     <= #TCQ (~real_scan_command_d[0]) && real_scan_command[0];
end

always @(posedge clk_sys_i) begin
    if((~real_scan_command_d[0]) && real_scan_command[0])begin
        if(real_scan_command[15:8]=='d0)
            real_scan_sel <= #TCQ 'd7;
        else 
            real_scan_sel <= #TCQ real_scan_command_d[10:8];
    end
end

// motor parameter settings
always @(posedge clk_sys_i) begin
    if(command_sel=='h0115 && command_data_vld)begin
        data_acq_en       <= #TCQ command_data[2:0];
    end
    `ifdef FBC_OFF
    else if(overload_pid_result_i[31])begin 
        data_acq_en       <= #TCQ 'd1;
    end
    `else
    else if(overload_pid_result_i[31] || real_scan_start || fbc_open_loop_i)begin 
        data_acq_en       <= #TCQ 'd1;
    end
    else if(fbc_close_loop_i)begin
        data_acq_en       <= #TCQ 'd2;
    end
    `endif // FBC_OFF
end

// timing card to laser uart
always @(posedge clk_sys_i) begin
    if(command_sel=='h0122 && command_data_vld)begin
        laser_uart_data <= #TCQ command_data;
        laser_uart_vld  <= #TCQ 'd1;
    end
    else begin
        laser_uart_vld  <= #TCQ 'd0;
    end
end

always @(posedge clk_sys_i) begin
    if(command_sel=='h0123 && command_data_vld)begin
        pmt_master_spi_data <= #TCQ command_data;
        pmt_master_spi_vld  <= #TCQ 'd1;
    end
    else begin
        pmt_master_spi_vld  <= #TCQ 'd0;
    end
end

// PMT adc start sel
always @(posedge clk_sys_i) begin
    if(command_sel=='h0124 && command_data_vld)begin
        pmt_adc_start_data <= #TCQ command_data;
        pmt_adc_start_vld  <= #TCQ |command_data[10:8];
    end
    else begin
        pmt_adc_start_vld  <= #TCQ 'd0;
    end
end

// readback register
always @(posedge clk_sys_i) begin
    if(command_sel=='h0200 && command_data_vld)begin
        readback_reg    <= #TCQ command_data;
        readback_en     <= #TCQ 'd1;
    end
    else begin
        readback_en  <= #TCQ 'd0;
    end
end

always @(posedge clk_sys_i) begin
    if(command_sel=='h0133 && command_data_vld)begin
        pmt_encode_align_rst <= #TCQ 'd1;
    end
    else begin
        pmt_encode_align_rst <= #TCQ 'd0;
    end
end

always @(posedge clk_sys_i) begin
    if(command_sel=='h0134 && command_data_vld)begin
        eds_encode_align_rst <= #TCQ 'd1;
    end
    else begin
        eds_encode_align_rst <= #TCQ 'd0;
    end
end

always @(posedge clk_sys_i) begin
    if(readback_en)begin
        case (readback_reg)
            'h0115:  register_data <= #TCQ data_acq_en          ;
            'h0117:  register_data <= #TCQ position_arm         ;
            'h0118:  register_data <= #TCQ kp                   ;
            'h0119:  register_data <= #TCQ motor_freq           ;
            'h011B:  register_data <= #TCQ fbc_bias_voltage     ;
            'h011C:  register_data <= #TCQ fbc_cali_uop_set     ;
            'h011D:  register_data <= #TCQ ki                   ;
            'h011E:  register_data <= #TCQ kd                   ;
            'h0120:  register_data <= #TCQ eds_power_en         ;
            'h0121:  register_data <= #TCQ eds_frame_en         ;
            'h0125:  register_data <= #TCQ pmt_adc_start_hold   ;
            'h0126:  register_data <= #TCQ eds_test_en          ;
            'h0127:  register_data <= #TCQ scan_soft_reset      ;
            'h0128:  register_data <= #TCQ real_scan_command    ;
            'h0129:  register_data <= #TCQ x_start_encode       ;
            'h012a:  register_data <= #TCQ x_end_encode         ;
            'h012b:  register_data <= #TCQ position_pid_thr     ;
            'h012c:  register_data <= #TCQ fast_shutter_set     ;
            'h012d:  register_data <= #TCQ laser_fast_shutter_i ;
            'h012e:  register_data <= #TCQ fast_shutter_act_time_i ;
            'h012f:  register_data <= #TCQ scan_state_i         ;

            'h0130:  register_data <= #TCQ x_encode_zero_calib  ;
            'h0131:  register_data <= #TCQ pmt_precise_encode_x_offset;
            'h0132:  register_data <= #TCQ pmt_encode_x_i       ;
            'h0133:  register_data <= #TCQ pmt_encode_align_set ;
            'h0134:  register_data <= #TCQ eds_encode_align_set ;
            'h0135:  register_data <= #TCQ encode_sim_en        ;
            'h0136:  register_data <= #TCQ pmt_encode_w_i       ;
            'h0138:  register_data <= #TCQ plc_x_encode         ;
            'h0139:  register_data <= #TCQ plc_x_encode_en      ;
            
            'h0140:  register_data <= #TCQ motor_Ufeed_latch_i  ;
            'h0141:  register_data <= #TCQ motor_data_in_i      ;

            'h0201:  register_data <= #TCQ overload_motor_set   ;
            'h0202:  register_data <= #TCQ overload_pid_result_i;

            'h0400:  register_data <= #TCQ eds_pack_cnt_1_i     ;
            'h0401:  register_data <= #TCQ encode_pack_cnt_1_i  ;
            'h0402:  register_data <= #TCQ eds_pack_cnt_2_i     ;
            'h0403:  register_data <= #TCQ encode_pack_cnt_2_i  ;
            'h0404:  register_data <= #TCQ eds_pack_cnt_3_i     ;
            'h0405:  register_data <= #TCQ encode_pack_cnt_3_i  ;

            // 'h0406:  register_data <= #TCQ encode_interval_rst  ;
            // 'h0407:  register_data <= #TCQ encode_interval_max_i;
            // 'h0408:  register_data <= #TCQ encode_w_diff_max_i  ;
            // 'h0409:  register_data <= #TCQ pmt_encode_w_diff_max_i;
            // 'h0410:  register_data <= #TCQ delta_w_encode_acce_i;

            // 'h0411:  register_data <= #TCQ pmt_x_encode_i       ;
            // 'h0412:  register_data <= #TCQ pmt_w_encode_i       ;
            // 'h0413:  register_data <= #TCQ pmt_fifo_state_i     ;
            // 'h0415:  register_data <= #TCQ acs_x_encode_i       ;
            // 'h0416:  register_data <= #TCQ acs_w_encode_i       ;
            // 'h0417:  register_data <= #TCQ acs_fifo_state_i     ;
            // 'h0418:  register_data <= #TCQ pmt_w_encode_thr     ;
            // 'h0419:  register_data <= #TCQ acs_w_encode_thr     ;
            default: register_data <= #TCQ 'h00_DEAD_00         ;
        endcase
    end
end

reg readback_en_d = 'd0;
always @(posedge clk_sys_i) readback_en_d <= #TCQ readback_en;
always @(posedge clk_sys_i) readback_vld  <= #TCQ readback_en_d || scan_finish_comm_i || scan_error_comm_i;
always @(posedge clk_sys_i) begin
    if(readback_en_d)
        readback_data <= #TCQ {readback_reg[31:0],register_data[31:0]};
    else if(scan_finish_comm_i)
        readback_data <= #TCQ {32'h0000_0300,32'h0};
    else if(scan_error_comm_i)
        readback_data <= #TCQ {32'h0000_0300,32'h1};
end

always @(posedge clk_sys_i) scan_soft_reset_d    <= #TCQ scan_soft_reset;
always @(posedge clk_sys_i) scan_soft_reset_pose <= #TCQ ~scan_soft_reset_d && scan_soft_reset;

assign readback_data_o          = readback_data                 ;
assign readback_vld_o           = readback_vld                  ;

assign data_acq_en_o            = data_acq_en                   ;
assign bg_data_acq_en_o         = bg_data_acq_en                ;
assign position_arm_o           = position_arm                  ;
assign kp_o                     = kp                            ;
assign ki_o                     = ki                            ;
assign kd_o                     = kd                            ;
assign motor_freq_o             = motor_freq                    ;
assign bpsi_position_en_o       = bpsi_position_en              ;
assign fbc_bias_voltage_o       = fbc_bias_voltage              ;
assign fbc_cali_uop_set_o       = fbc_cali_uop_set              ;
assign eds_power_en_o           = eds_power_en                  ;
assign eds_frame_en_o           = eds_frame_en                  ;
assign eds_test_en_o            = eds_test_en                   ;
assign eds_texp_time_o          = eds_texp_time                 ;
assign eds_frame_to_frame_time_o= eds_frame_to_frame_time       ;
assign laser_uart_data_o        = laser_uart_data               ;
assign laser_uart_vld_o         = laser_uart_vld                ;
assign pmt_master_spi_data_o    = pmt_master_spi_data           ;
assign pmt_master_spi_vld_o     = pmt_master_spi_vld            ;
assign pmt_adc_start_data_o     = pmt_adc_start_data            ;
assign pmt_adc_start_vld_o      = pmt_adc_start_vld             ;
assign pmt_adc_start_hold_o     = pmt_adc_start_hold            ;
assign rd_mfpga_version_o       = rd_mfpga_version              ;
assign FBC_fifo_rst_o           = FBC_fifo_rst                  ;
assign scan_soft_reset_o        = scan_soft_reset_pose          ; 
assign real_scan_start_o        = real_scan_start               ;
assign real_scan_sel_o          = real_scan_sel                 ;
assign x_start_encode_o         = x_start_encode                ; 
assign x_end_encode_o           = x_end_encode                  ; 
assign position_pid_thr_o       = position_pid_thr              ;
assign soft_fast_shutter_set_o  = fast_shutter_set              ;
assign soft_fast_shutter_en_o   = fast_shutter_en               ;
assign pmt_encode_align_rst_o   = pmt_encode_align_rst          ;
assign pmt_encode_align_set_o   = pmt_encode_align_set          ;
assign eds_encode_align_rst_o   = eds_encode_align_rst          ;
assign eds_encode_align_set_o   = eds_encode_align_set          ;
assign encode_sim_en_o          = encode_sim_en                 ;
// assign encode_interval_rst_o    = encode_interval_rst           ;
assign scan_finish_comm_ack_o   = scan_finish_comm_ack          ;
assign plc_x_encode_o           = plc_x_encode                  ;
assign plc_x_encode_en_o        = plc_x_encode_en               ;

assign overload_motor_en_o      = overload_motor_set[31]        ;
assign overload_ufeed_thre_o    = overload_motor_set[15:0]      ;
assign x_encode_zero_calib_o    = x_encode_zero_calib           ;
assign pmt_precise_encode_x_offset_o = pmt_precise_encode_x_offset;
// assign pmt_encode_rd_en_o       = pmt_encode_rd_en              ;
// assign acs_encode_rd_en_o       = acs_encode_rd_en              ;
// assign pmt_w_encode_thr_o       = pmt_w_encode_thr              ;
// assign acs_w_encode_thr_o       = acs_w_encode_thr              ;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

endmodule
