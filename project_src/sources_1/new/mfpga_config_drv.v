`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/24 
// Design Name: 
// Module Name: mfpga_config_drv
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


module mfpga_config_drv(
    //clk
    input   wire        clk                         , // main clk 50MHz
    input   wire        rst_n                       ,
    
    input   wire        phy_clk_i                   ,
    input   wire        load_error_i                ,
    input   wire        cfg_load_last_i             ,

    input   wire        cfg_fifo_ready_i            ,
    input   wire        cfg_fifo_empty_i            ,
    input   wire        cfg_rd_vld_i                ,
    input   wire [15:0] cfg_rd_data_i               ,

    output  wire        cfg_rd_seq_o                ,
    
    //config fpga
    output  wire [15:0] FPGA_CONFIG_D               ,
    output  wire        FPGA_CONFIG_RDWR_B          ,
    output  wire        FPGA_CONFIG_CSI_B           ,
    input   wire        FPGA_INIT_B_SFPGA           ,
    output  wire        FPGA_PROG_B_SFPGA           ,
    output  wire        FPGA_CCLK_SFPGA             ,
    input   wire        FPGA_DONE_SFPGA             
);

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam              IDLE                    = 3'd0;
localparam              CHECK                   = 3'd1;    
localparam              SYNC                    = 3'd2;    
localparam              LOAD                    = 3'd3;    
localparam              SUCC                    = 3'd4;

localparam              SYNC_NUM                = 'd4;

genvar i;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg  [ 3-1:0]           state                   = IDLE;
reg  [ 3-1:0]           state_next              = IDLE;

// FSM control
reg  [ 8-1:0]           sync_cnt                = 'd0;
reg  [16-1:0]           sync_byte               = 'hffff;
reg                     init_reg_d0             = 'd0;
reg                     init_reg_d1             = 'd0;
reg                     init_pose               = 'd0;
reg                     fpga_done_r             = 'd0;
reg                     cfg_load_finish         = 'd0;
reg  [ 8-1:0]           wait_cnt                = 'd0;

reg                     program_b_r             = 'd1;
reg                     csi_b_r                 = 'd1;
reg                     rdwr_b_r                = 'd1;
reg  [16-1:0]           cfg_data_r              = 'hffff;


// fifo contorl
// reg                     cfg_rd_en               = 'd0;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                    load_error_sync ;
wire                    cfg_load_last_sync;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
xpm_cdc_pulse #(
   .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
   .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
   .REG_OUTPUT(0),     // DECIMAL; 0=disable registered output, 1=enable registered output
   .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
   .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_error_inst (
   .dest_pulse(load_error_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                            // transfer is correctly initiated on src_pulse input. This output is
                            // combinatorial unless REG_OUTPUT is set to 1.

   .dest_clk(clk),     // 1-bit input: Destination clock.
   .dest_rst(1'd0),     // 1-bit input: optional; required when RST_USED = 1
   .src_clk(phy_clk_i),       // 1-bit input: Source clock.
   .src_pulse(load_error_i),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
                            // destination clock domain. The minimum gap between each pulse transfer must be
                            // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
                            // between the falling edge of a src_pulse to the rising edge of the next
                            // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
                            // will generate a pulse the size of one dest_clk period in the destination
                            // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
                            // src_rst and/or dest_rst are asserted.

   .src_rst(1'd0)        // 1-bit input: optional; required when RST_USED = 1
);

xpm_cdc_pulse #(
   .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
   .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
   .REG_OUTPUT(0),     // DECIMAL; 0=disable registered output, 1=enable registered output
   .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
   .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_last_inst (
   .dest_pulse(cfg_load_last_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                            // transfer is correctly initiated on src_pulse input. This output is
                            // combinatorial unless REG_OUTPUT is set to 1.

   .dest_clk(clk),     // 1-bit input: Destination clock.
   .dest_rst(1'd0),     // 1-bit input: optional; required when RST_USED = 1
   .src_clk(phy_clk_i),       // 1-bit input: Source clock.
   .src_pulse(cfg_load_last_i),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
                            // destination clock domain. The minimum gap between each pulse transfer must be
                            // at the minimum 2*(larger(src_clk period, dest_clk period)). This is measured
                            // between the falling edge of a src_pulse to the rising edge of the next
                            // src_pulse. This minimum gap will guarantee that each rising edge of src_pulse
                            // will generate a pulse the size of one dest_clk period in the destination
                            // clock domain. When RST_USED = 1, pulse transfers will not be guaranteed while
                            // src_rst and/or dest_rst are asserted.

   .src_rst(1'd0)        // 1-bit input: optional; required when RST_USED = 1
);
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

always @(posedge clk) begin
    init_reg_d0 <= FPGA_INIT_B_SFPGA;
    init_reg_d1 <= init_reg_d0;
    init_pose   <= ~init_reg_d1 && init_reg_d0;
end

always @(posedge clk) begin
    if(!rst_n)
        state <= IDLE;
    else 
        state <= state_next;
end

always @(*) begin
    state_next = state;
    case(state)
        IDLE: 
            if(cfg_fifo_ready_i)
                state_next = CHECK;
        CHECK: 
            if(init_pose)
                state_next = SYNC;
        SYNC:
            if(sync_cnt==SYNC_NUM)
                state_next = LOAD;
        LOAD:
            if(fpga_done_r || load_error_sync || cfg_load_last_sync)
                state_next = IDLE;
         
        default: 
                state_next = IDLE;
    endcase
end

// PROGRAM_B info
always @(posedge clk) begin
    if(state==IDLE && cfg_fifo_ready_i)begin
        program_b_r <= 'd0;
    end
    else if(state==CHECK)begin
        if(~init_reg_d0)
            program_b_r <= 'd1;
    end
end

// sync byte
always @(posedge clk) begin
    if(state==SYNC)
        sync_cnt <= sync_cnt + 1;
    else 
        sync_cnt <= 'd0;
end

always @(posedge clk) begin
    case(sync_cnt)
        'd0: sync_byte <= 'hffff;
        'd1: sync_byte <= 'h00BB;
        'd2: sync_byte <= 'h0022;
        'd3: sync_byte <= 'h5566;
        'd4: sync_byte <= 'hffff;
        default:/*default*/;
    endcase
end

// load_succ_r signal
always @(posedge clk) begin
    fpga_done_r <= FPGA_DONE_SFPGA;
end

// CSI_B info
always @(negedge clk ) begin
    if(state==IDLE)
        csi_b_r <= 'd1;
    else if(state==SYNC)
        csi_b_r <= 'd0;
    else if(state==LOAD)
        csi_b_r <= ~(cfg_rd_vld_i || cfg_load_finish);
end

always @(negedge clk) begin
    if(state==SYNC)
        cfg_data_r <= sync_byte;
    else if(cfg_rd_vld_i)
        cfg_data_r <= cfg_rd_data_i;
    else 
        cfg_data_r <= 'hffff;
end

always @(negedge clk) begin
    if(state==IDLE && csi_b_r)
        rdwr_b_r <= 'd1;
    else 
        rdwr_b_r <= 'd0;
end

assign  cfg_rd_seq_o        = ~cfg_fifo_empty_i && (state==LOAD || state==SUCC);
generate
    for(i=0;i<8;i=i+1)begin
    assign  FPGA_CONFIG_D[i]       = cfg_data_r[7-i];
    assign  FPGA_CONFIG_D[i+8]     = cfg_data_r[7-i+8];
        
    end
endgenerate
// assign  FPGA_CONFIG_D       = cfg_data_r;
assign  FPGA_CONFIG_RDWR_B  = rdwr_b_r;
assign  FPGA_CONFIG_CSI_B   = csi_b_r;
assign  FPGA_PROG_B_SFPGA   = program_b_r;
assign  FPGA_CCLK_SFPGA     = clk;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
endmodule
