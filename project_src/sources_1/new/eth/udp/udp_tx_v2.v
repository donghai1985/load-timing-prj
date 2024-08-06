`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/26 13:40:10
// Design Name: 
// Module Name: udp_tx_v2
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

module udp_tx_v2    #(
    parameter               BOARD_MAC    = 48'h00_11_22_33_44_55,
    parameter               BOARD_IP     = {8'd192,8'd168,8'd1,8'd10},
    parameter               DES_MAC_PARA = 48'hff_ff_ff_ff_ff_ff,
    parameter               DES_IP       = {8'd192,8'd168,8'd1,8'd102}
)
(    
    input                   clk                     , 
    input                   rst_n                   , 
    
    input                   tx_start_en              , 
    input        [15:0]     tx_byte_num              , 
(*mark_debug = "true"*)    input        [7:0]      tx_data                  , 
    input        [47:0]     des_mac                 , 
    input        [31:0]     des_ip                  , 
    input        [31:0]     crc_data                , 
    input         [7:0]     crc_next                , 
(*mark_debug = "true"*)    output  reg             tx_done         = 'd0    , 
(*mark_debug = "true"*)    output  wire            tx_req                   , 
    output  reg             gmii_tx_en      = 'd0    , 
    output  reg  [7:0]      gmii_txd        = 'd0    , 
    output  reg             crc_en          = 'd0    , 
    output  reg             crc_clr         = 'd0     
    );


//////////////////////////////////////////////////////////////////////////////////
// *********** Define Parameter Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
localparam                  ST_IDLE         = 7'b000_0001;
localparam                  ST_CHECK_SUM    = 7'b000_0010;
localparam                  ST_PREAMBLE     = 7'b000_0100;
localparam                  ST_ETH_HEAD     = 7'b000_1000;
localparam                  ST_IP_HEAD      = 7'b001_0000;
localparam                  ST_TX_DATA      = 7'b010_0000;
localparam                  ST_CRC          = 7'b100_0000;

localparam                  ETH_TYPE        = 16'h0800  ;  
localparam                  MIN_DATA_NUM    = 16'd18    ;    

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Register Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
reg             [6:0]       cur_state       = ST_IDLE; 

reg             [7:0]       preamble[7:0]  ; 
reg             [7:0]       eth_head[13:0] ; 
reg             [31:0]      ip_head[6:0]   ; 

reg             [15:0]      tx_data_num     = 'd0; 
reg             [15:0]      total_num       = 'd0; 
reg                         trig_tx_en      = 'd0; 
reg             [15:0]      udp_num         = 'd0; 
reg             [4:0]       cnt             = 'd0; 
reg             [31:0]      check_buffer    = 'd0; 
reg             [1:0]       tx_bit_sel      = 'd0; 
reg             [15:0]      data_cnt        = 'd0; 
reg                         tx_done_t       = 'd0; 
reg             [4:0]       real_add_cnt    = 'd0; 

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Define Wire Signal
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
wire            [15:0]      real_tx_data_num;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Instance Module
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////////////
// *********** Logic Design
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
assign  real_tx_data_num = (tx_data_num >= MIN_DATA_NUM) 
                           ? tx_data_num : MIN_DATA_NUM;  

// udp num initialization
always @(posedge clk) begin
    if(tx_start_en && cur_state==ST_IDLE) begin
        tx_data_num <= tx_byte_num;        
        total_num   <= tx_byte_num + 16'd28;  
        udp_num     <= tx_byte_num + 16'd8;               
    end    
end

always @(posedge clk) trig_tx_en <= tx_start_en; 

always @(posedge clk ) begin
    if(!rst_n)begin
        cur_state <= ST_IDLE;
        ip_head[1][31:16] <= 16'd0;
    end
    else begin
        case(cur_state)
        ST_IDLE: begin
            if(trig_tx_en)begin
                cur_state <= ST_CHECK_SUM;   // FSM jump

                // configuration parameter initialization
                ip_head[0]          <= {8'h45,8'h00,total_num};   
                ip_head[1][31:16]   <= ip_head[1][31:16] + 1'b1; 
                ip_head[1][15:0]    <= 16'h4000;    
                ip_head[2]          <= {8'h40,8'd17,16'h0};   
                ip_head[3]          <= BOARD_IP;

                if(des_ip != 32'd0)
                    ip_head[4]      <= des_ip;
                else
                    ip_head[4]      <= DES_IP; 

                ip_head[5]          <= {16'd1234,16'd1234};  
                ip_head[6]          <= {udp_num,16'h0000};  

                if(des_mac != 48'b0) begin
                    eth_head[0]     <= des_mac[47:40];
                    eth_head[1]     <= des_mac[39:32];
                    eth_head[2]     <= des_mac[31:24];
                    eth_head[3]     <= des_mac[23:16];
                    eth_head[4]     <= des_mac[15:8];
                    eth_head[5]     <= des_mac[7:0];
                end
                else begin
                    eth_head[0]     <= DES_MAC_PARA[47:40];
                    eth_head[1]     <= DES_MAC_PARA[39:32];
                    eth_head[2]     <= DES_MAC_PARA[31:24];
                    eth_head[3]     <= DES_MAC_PARA[23:16];
                    eth_head[4]     <= DES_MAC_PARA[15:8];
                    eth_head[5]     <= DES_MAC_PARA[7:0];
                end
                eth_head[6]  <= BOARD_MAC[47:40];
                eth_head[7]  <= BOARD_MAC[39:32];
                eth_head[8]  <= BOARD_MAC[31:24];
                eth_head[9]  <= BOARD_MAC[23:16];
                eth_head[10] <= BOARD_MAC[15:8];
                eth_head[11] <= BOARD_MAC[7:0];
                eth_head[12] <= ETH_TYPE[15:8];
                eth_head[13] <= ETH_TYPE[7:0];  

                
                preamble[0]  <= 8'h55;                 
                preamble[1]  <= 8'h55;
                preamble[2]  <= 8'h55;
                preamble[3]  <= 8'h55;
                preamble[4]  <= 8'h55;
                preamble[5]  <= 8'h55;
                preamble[6]  <= 8'h55;
                preamble[7]  <= 8'hd5;
            end
            
            // gmii tx ctrl register initialization
            cnt         <= 'd0;
            data_cnt    <= 'd0;
            tx_bit_sel  <= 'd0;
            gmii_tx_en  <= 'd0;
            gmii_txd    <= 'd0;
            tx_done_t   <= 'd0;
            crc_en      <= 'd0;
        end

        ST_CHECK_SUM: begin
            // FSM cnt control
            if(cnt == 'd3)begin
                cnt <= 'd0;
                cur_state <= ST_PREAMBLE;
            end 
            else begin
                cnt <= cnt + 1;
            end

            // check buffer and ip head
            if(cnt == 'd0)begin
                check_buffer <= ip_head[0][31:16] + ip_head[0][15:0]
                                + ip_head[1][31:16] + ip_head[1][15:0]
                                + ip_head[2][31:16] + ip_head[2][15:0]
                                + ip_head[3][31:16] + ip_head[3][15:0]
                                + ip_head[4][31:16] + ip_head[4][15:0];
            end
            else if(cnt == 'd1)                
                check_buffer <= check_buffer[31:16] + check_buffer[15:0];
            else if(cnt == 'd2)
                check_buffer <= check_buffer[31:16] + check_buffer[15:0];

            if(cnt == 'd3)
                ip_head[2][15:0] <= ~check_buffer[15:0];
        end

        ST_PREAMBLE: begin
            // FSM cnt control
            if(cnt == 'd7)begin
                cnt <= 'd0;
                cur_state <= ST_ETH_HEAD;
            end 
            else begin
                cnt <= cnt + 1;
            end

            // gmii tx
            gmii_tx_en  <= 1'b1;
            gmii_txd    <= preamble[cnt];
            
        end

        ST_ETH_HEAD: begin
            // FSM cnt control
            if(cnt == 'd13)begin
                cnt <= 'd0;
                cur_state <= ST_IP_HEAD;
            end 
            else begin
                cnt <= cnt + 1;
            end
            
            // gmii tx
            gmii_tx_en  <= 1'b1;
            gmii_txd    <= eth_head[cnt];

            // crc ctrl
            crc_en      <= 1'b1;
        end

        ST_IP_HEAD: begin
            // FSM cnt control
            if(tx_bit_sel=='d3)begin
                if(cnt == 'd6)begin
                    cnt         <= 'd0;
                    tx_bit_sel  <= 'd0;
                    cur_state   <= ST_TX_DATA;
                end
                else begin
                    tx_bit_sel  <= tx_bit_sel + 1;
                    cnt         <= cnt + 1;
                end
            end
            else begin
                tx_bit_sel <= tx_bit_sel + 1;
            end
            
            // gmii tx
            gmii_tx_en <= 1'b1;        
            case (tx_bit_sel)
                'd0: gmii_txd <= ip_head[cnt][31:24];
                'd1: gmii_txd <= ip_head[cnt][23:16];
                'd2: gmii_txd <= ip_head[cnt][15:8];
                'd3: gmii_txd <= ip_head[cnt][7:0];  
                default:/*default*/; 
            endcase

            // crc ctrl
            crc_en <= 1'b1;
        end

        ST_TX_DATA: begin
            // FSM control
            if(data_cnt == tx_data_num - 1)begin
                if(data_cnt + real_add_cnt < real_tx_data_num - 1)
                    real_add_cnt    <= real_add_cnt + 5'd1;  
                else begin
                    cur_state       <= ST_CRC;
                    data_cnt        <= 16'd0;
                    real_add_cnt    <= 5'd0;
                end    
            end
            else begin
                data_cnt <= data_cnt + 16'd1; 
            end

            // gmii tx 
            gmii_tx_en  <= 'd1;  // delay tx_req 1 clk
            crc_en      <= 'd1;

            if(real_add_cnt=='d0)
                gmii_txd <= tx_data;
            else 
                gmii_txd <= 'd0;
            
        end

        ST_CRC: begin
            // FSM control
            if(tx_bit_sel=='d3)begin
                cur_state <= ST_IDLE; 
            end
            else begin
                tx_bit_sel <= tx_bit_sel + 1;
            end

            case(tx_bit_sel)
            'd0: gmii_txd <= {~crc_next[0], ~crc_next[1], ~crc_next[2],~crc_next[3],
                             ~crc_next[4], ~crc_next[5], ~crc_next[6],~crc_next[7]};
            'd1: gmii_txd <= {~crc_data[16], ~crc_data[17], ~crc_data[18],~crc_data[19],
                             ~crc_data[20], ~crc_data[21], ~crc_data[22],~crc_data[23]};
            'd2: gmii_txd <= {~crc_data[8], ~crc_data[9], ~crc_data[10],~crc_data[11],
                             ~crc_data[12], ~crc_data[13], ~crc_data[14],~crc_data[15]};  
            'd3: gmii_txd <= {~crc_data[0], ~crc_data[1], ~crc_data[2],~crc_data[3],
                             ~crc_data[4], ~crc_data[5], ~crc_data[6],~crc_data[7]};  
            default:/*default*/;
            endcase

            // gmii tx
            gmii_tx_en  <= 1'b1;

            // tx done and crc clear
            crc_en      <= 'd0;
            tx_done_t   <= tx_bit_sel=='d3;
            
        end

        default :cur_state<= ST_IDLE;  
        endcase     
    end                                        
end       

assign tx_req = (data_cnt < tx_data_num) && (cur_state==ST_TX_DATA);

always @(posedge clk) begin
    begin
        tx_done <= tx_done_t;
        crc_clr <= tx_done_t;
    end
end

endmodule

