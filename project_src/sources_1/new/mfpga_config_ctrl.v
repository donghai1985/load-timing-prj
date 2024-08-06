`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/24 08:15:24
// Design Name: 
// Module Name: mfpga_config_ctrl
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


module mfpga_config_ctrl(
    //clk
    input   wire        clk                         , // main clk 50MHz
    input   wire        rst_n                       ,
    //ethernet interface
    input   wire        eth_clk                     ,
    input   wire        eth_rec_cfg_pkg_total_en    ,
    input   wire [15:0] eth_rec_cfg_pkg_total       ,
    input   wire        eth_rec_cfg_pkg_num_en      ,
    input   wire [15:0] eth_rec_cfg_pkg_num         ,
    input   wire        eth_rec_cfg_en              ,
    input   wire [7:0]  eth_rec_cfg_data            ,
    // udp tx signal
    output  wire        udp_load_loss_o             ,
    output  wire [8:0]  udp_load_status_o           ,
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
localparam              CHECK_NUM               = 3'd1;    
localparam              CONFIG_FPGA             = 3'd2;    
localparam              WAIT                    = 3'd3;    

`ifdef SIMULATE
localparam              TIME_THRE               = 'd1000; 
`else
localparam              TIME_THRE               = 'd65535; // 65535 * 5ns * 256 = 84ms
`endif // SIMULATE

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg  [ 3-1:0]           state                   = IDLE;
reg  [ 3-1:0]           state_next              = IDLE;

// FSM control
reg                     init_ready              = 'd0;
reg                     load_last               = 'd0;
reg                     fpga_done_r             = 'd0;
reg                     init_reg_d0             = 'd0;
reg                     init_reg_d1             = 'd0;
reg                     init_pose               = 'd0;

reg                     load_loss_r             = 'd0;
reg  [ 9-1:0]           load_status_r           = 'd0;

// fifo contorl
reg                     cfg_rd_en               = 'd0;
reg                     cfg_data_vld            = 'd0;

// config pkg check
reg  [16-1:0]           cfg_pkg_cnt             = 'd0;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire                    init_ready_sync         ;


wire                    cfg_rd_seq              ;
wire [16-1:0]           cfg_rd_data             ;
wire                    cfg_full                ;
wire                    cfg_empty               ;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

mfpga_cfg_data_fifo mfpga_cfg_data_fifo_inst(
    .rst                        ( ~rst_n                    ),  // input wire rst
    .wr_clk                     ( eth_clk                   ),  // input wire wr_clk
    .rd_clk                     ( clk                       ),  // input wire rd_clk
    .din                        ( eth_rec_cfg_data          ),  // input wire [7 : 0] din
    .wr_en                      ( eth_rec_cfg_en            ),  // input wire wr_en
    .rd_en                      ( cfg_rd_en                 ),  // input wire rd_en
    .dout                       ( cfg_rd_data               ),  // output wire [15 : 0] dout
    .full                       ( cfg_full                  ),  // output wire full
    .empty                      ( cfg_empty                 )   // output wire empty
);  


mfpga_config_drv mfpga_config_drv_inst(
    //clk
    .clk                        ( clk                       ), // main clk 50MHz
    .rst_n                      ( rst_n                     ),
    
    .phy_clk_i                  ( eth_clk                   ),
    .load_error_i               ( load_loss_r               ),
    .cfg_load_last_i            ( load_last                 ),

    .cfg_fifo_ready_i           ( init_ready_sync           ),
    .cfg_fifo_empty_i           ( cfg_empty                 ),
    .cfg_rd_vld_i               ( cfg_data_vld              ),
    .cfg_rd_data_i              ( cfg_rd_data               ),

    .cfg_rd_seq_o               ( cfg_rd_seq                ),

    .FPGA_CONFIG_D              ( FPGA_CONFIG_D             ),
    .FPGA_CONFIG_RDWR_B         ( FPGA_CONFIG_RDWR_B        ),
    .FPGA_CONFIG_CSI_B          ( FPGA_CONFIG_CSI_B         ),
    .FPGA_INIT_B_SFPGA          ( FPGA_INIT_B_SFPGA         ),
    .FPGA_PROG_B_SFPGA          ( FPGA_PROG_B_SFPGA         ),
    .FPGA_CCLK_SFPGA            ( FPGA_CCLK_SFPGA           ),
    .FPGA_DONE_SFPGA            ( FPGA_DONE_SFPGA           )
);

xpm_cdc_pulse #(
   .DEST_SYNC_FF(2),   // DECIMAL; range: 2-10
   .INIT_SYNC_FF(0),   // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
   .REG_OUTPUT(1),     // DECIMAL; 0=disable registered output, 1=enable registered output
   .RST_USED(0),       // DECIMAL; 0=no reset, 1=implement reset
   .SIM_ASSERT_CHK(0)  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
)
xpm_cdc_pulse_init_inst (
   .dest_pulse(init_ready_sync), // 1-bit output: Outputs a pulse the size of one dest_clk period when a pulse
                            // transfer is correctly initiated on src_pulse input. This output is
                            // combinatorial unless REG_OUTPUT is set to 1.

   .dest_clk(clk),     // 1-bit input: Destination clock.
   .dest_rst(1'd0),     // 1-bit input: optional; required when RST_USED = 1
   .src_clk(eth_clk),       // 1-bit input: Source clock.
   .src_pulse(init_ready),   // 1-bit input: Rising edge of this signal initiates a pulse transfer to the
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
always @(posedge eth_clk) begin
    if(!rst_n)
        state <= IDLE;
    else 
        state <= state_next;
end

always @(*) begin
    state_next = state;
    case(state)
        IDLE: 
            if(init_ready)
                state_next = WAIT;
        WAIT: 
            if(~fpga_done_r)
                state_next = CHECK_NUM;
        CHECK_NUM: 
                state_next = CONFIG_FPGA;
        CONFIG_FPGA:
            if(eth_rec_cfg_pkg_num_en)
                state_next = CHECK_NUM;
            else if(load_last || fpga_done_r)
                state_next = IDLE;
        default: 
                state_next = IDLE;
    endcase
end

always @(posedge eth_clk) begin
    fpga_done_r <= FPGA_DONE_SFPGA;
end

// start loading fpga when fifo empty 
always @(posedge eth_clk) begin
    if(eth_rec_cfg_pkg_num_en && (eth_rec_cfg_pkg_num=='d1) && state==IDLE && cfg_empty)
        init_ready <= 'd1;
    else if(state==CONFIG_FPGA)
        init_ready <= 'd0;
end


always @(posedge eth_clk) begin
    load_last <= eth_rec_cfg_pkg_total_en;
end

// load_error signal
always @(posedge eth_clk) begin
    if(state==WAIT && state_next==CHECK_NUM)begin
        cfg_pkg_cnt <= 'd1;     // the next package count
    end
    else if(eth_rec_cfg_pkg_num_en) begin
        cfg_pkg_cnt <= cfg_pkg_cnt + 1;
    end
end

// load_loss signal
always @(posedge eth_clk) begin
    load_loss_r <= ((eth_rec_cfg_pkg_total != cfg_pkg_cnt) && eth_rec_cfg_pkg_total_en) ;
end

// load_status signal
always @(posedge eth_clk) begin
    if(load_last && fpga_done_r)begin
        load_status_r[7:0] <= 'h01;
        load_status_r[8]   <= 1'd1;
    end
    else if(load_last && (~fpga_done_r))begin
        load_status_r[7:0] <= 'h00;
        load_status_r[8]   <= 1'd1;
    end
    else begin
        load_status_r[8]   <= 1'd0;
    end
end

// read config data
always @(posedge clk) begin
    if(state==IDLE)begin
        cfg_rd_en <= (~init_ready);     // clean fifo
    end
    else if(state==CONFIG_FPGA)begin
        cfg_rd_en <= cfg_rd_seq;
    end
    else begin
        cfg_rd_en <= 'd0;
    end
end

always @(posedge clk) cfg_data_vld <= cfg_rd_en && (~cfg_empty) && (state==CONFIG_FPGA);

assign  udp_load_loss_o     = load_loss_r;
assign  udp_load_status_o   = load_status_r;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** simulate logic
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>






//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
endmodule
