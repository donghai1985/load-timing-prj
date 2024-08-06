`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/18
// Design Name: 
// Module Name: udp_message_sim
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
// `define CONFIG_FPGA_SIM

module udp_message_sim #(
    parameter               DATA_WIDTH     = 8 ,
    parameter               BYTE_NUM_WIDTH = 16 
)(
    // clk & rst
    input    wire                         phy_clk               ,  //125MHz 
    input    wire                         rst_n                 ,
    // ethernet interface for message data
    output   wire                         rec_pkt_done_o        ,
    output   wire                         rec_en_o              ,
    output   wire    [DATA_WIDTH-1:0]     rec_data_o            ,
    output   wire                         rec_byte_num_en_o     ,
    output   wire    [BYTE_NUM_WIDTH-1:0] rec_byte_num_o        ,

    // config FPGA info
    output   wire                         rec_cfg_pkg_total_en_o, 
    output   wire    [15:0]               rec_cfg_pkg_total_o   , 
    output   wire                         rec_cfg_pkg_num_en_o  ,
    output   wire    [15:0]               rec_cfg_pkg_num_o     ,
    // output   wire                         rec_cfg_done_o        ,
    output   wire                         rec_cfg_en_o          ,
    output   wire    [7:0]                rec_cfg_data_o        
    // output   wire                         rec_cfg_byte_num_en_o ,
    // output   wire    [15:0]               rec_cfg_byte_num_o    
);


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam                  ST_IDLE         = 3'd0;
localparam                  ST_BYTE         = 3'd1;
localparam                  ST_UDP          = 3'd2;
localparam                  ST_FINISH       = 3'd3;
localparam                  ST_CFG_HEAD     = 3'd4;
localparam                  ST_CFG_LOAD     = 3'd5;
localparam                  ST_CFG_PKG_NUM  = 3'd6;
localparam                  ST_CFG_FINISH   = 3'd7;

localparam                  UDP_NUM         = 'd16;
localparam                  CFG_DATA_NUM    = 'd1024;
localparam                  CFG_PKG_NUM     = 'd14;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

reg [ 3-1:0]                state           = ST_IDLE;
reg [ 3-1:0]                state_next      = ST_IDLE;


reg [32-1:0]                st_wait_cnt     = 'd0;
reg [BYTE_NUM_WIDTH-1:0]    msg_data_count  = 'd0;
reg                         msg_data_en_r   = 'd0;
reg [DATA_WIDTH-1:0]        msg_data_r      = 'd0;

reg [16-1:0]                rec_cfg_pkg_num_r = 'd1;
reg [16-1:0]                cfg_data_count  = 'd0;

reg init_done = 'd0;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<





//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
always @(posedge phy_clk) begin
    if(state==ST_IDLE)begin
        st_wait_cnt <= st_wait_cnt + 1;
    end
    else begin
        st_wait_cnt <= 'd0;
    end
end

always @(posedge phy_clk) begin
    if(~rst_n)
        state <= ST_IDLE;
    else 
        state <= state_next;
end

always @(*) begin
    state_next = state;
    case(state)
    ST_IDLE:
        if(st_wait_cnt == 'd6250)
        `ifdef CONFIG_FPGA_SIM
            state_next = ST_CFG_HEAD;
        `else
            state_next = ST_BYTE;
        `endif // CONFIG_FPGA_SIM
    ST_BYTE :
            state_next = ST_UDP;
    ST_UDP :
        if(msg_data_count==UDP_NUM-1)
            state_next = ST_FINISH;
    ST_FINISH :
            state_next = ST_IDLE;
    
    ST_CFG_HEAD:
            state_next = ST_CFG_LOAD;
    ST_CFG_LOAD:
        if(cfg_data_count==CFG_DATA_NUM-1)begin
            if(rec_cfg_pkg_num_r<=CFG_PKG_NUM)
                state_next = ST_CFG_HEAD;
            else
                state_next = ST_CFG_FINISH;
        end
    // ST_CFG_PKG_NUM:
    //     if(rec_cfg_pkg_num_r==CFG_PKG_NUM)
    //         state_next = ST_CFG_FINISH ;
    //     else 
    //         state_next = ST_CFG_LOAD;
    ST_CFG_FINISH :
        if(~init_done)
            state_next = ST_IDLE;
    default:
            state_next = ST_IDLE;
    endcase
end

always @(posedge phy_clk ) begin
    if(state==ST_UDP)
        msg_data_count <= msg_data_count + 1;
    else 
        msg_data_count <= 'd0;
end

always @(posedge phy_clk) begin
    if(state==ST_UDP)
        case(msg_data_count)
        'd0,
        'd1,
        'd2,
        'd3,
        'd4,
        'd5,
        'd6,
        'd7,
        'd8,
        'd9,
        'd10,
        'd11,
        'd12,
        'd13,
        'd14,
        'd15: begin 
                msg_data_r <= msg_data_r + 1;
                msg_data_en_r <= 'd1;
              end
        default : /*default*/;
        endcase
    else 
        msg_data_en_r <= 'd0;
end

reg rec_byte_num_en = 'd0;
always @(posedge phy_clk) begin
    rec_byte_num_en <= state==ST_BYTE;
end

assign rec_pkt_done_o    = state==ST_FINISH;
assign rec_en_o          = msg_data_en_r;
assign rec_data_o        = msg_data_r;
assign rec_byte_num_en_o = rec_byte_num_en;
assign rec_byte_num_o    = UDP_NUM;



// config FPGA sim
always @(posedge phy_clk) begin
    if(state_next == ST_CFG_HEAD && state==ST_IDLE)begin
        rec_cfg_pkg_num_r <= 'd1;
    end
    else if(state_next==ST_CFG_HEAD && state==ST_CFG_LOAD)begin
        rec_cfg_pkg_num_r <= rec_cfg_pkg_num_r + 1;
    end
end

always @(posedge phy_clk) begin
    if(state==ST_CFG_LOAD)begin
        cfg_data_count <= cfg_data_count + 1;
    end
    else 
        cfg_data_count <= 'd0;
end

always @(posedge phy_clk) begin
    if(state==ST_CFG_LOAD && state_next==ST_CFG_FINISH)
        init_done <= 'd1;
end

assign rec_cfg_pkg_total_en_o = state==ST_CFG_LOAD && state_next==ST_CFG_FINISH;
assign rec_cfg_pkg_total_o    = rec_cfg_pkg_num_r;
assign rec_cfg_en_o         = state==ST_CFG_LOAD;
assign rec_cfg_data_o       = cfg_data_count;
assign rec_cfg_pkg_num_o    = rec_cfg_pkg_num_r;
assign rec_cfg_pkg_num_en_o = state==ST_CFG_HEAD;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


endmodule
