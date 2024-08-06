`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/27 13:40:10
// Design Name: 
// Module Name: eth_udp_loop
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

module eth_udp_loop(
    input              clk   , 		//ϵͳʱ��
	input			   idelay_clk,	//IDELAYʱ��
    input              rst_n , 		//ϵͳ��λ�źţ��͵�ƽ��Ч 
    //PL��̫��RGMII�ӿ�   
    input              eth_rxc   , //RGMII��������ʱ��
    input              eth_rx_ctl, //RGMII����������Ч�ź�
    input       [3:0]  eth_rxd   , //RGMII��������
    output             eth_txc   , //RGMII��������ʱ��    
    output             eth_tx_ctl, //RGMII���������Ч�ź�
    output      [3:0]  eth_txd   , //RGMII�������          
    output             eth_rst_n , //��̫��оƬ��λ�źţ��͵�ƽ��Ч 
	input              eth_intb  , //��̫��оƬ�ж��źţ��͵�ƽ��Ч 
	
	output			   eth_clk	 ,
	
	output             rec_pkt_done		,//UDP�������ݽ�������ź�
	output             rec_en      		,//UDP���յ�����ʹ���ź�
	output  	[7:0]  rec_data    		,//UDP���յ�����
	output		  	   rec_byte_num_en	,//UDP���յ���Ч�ֽ���ʹ���ź�
	output  	[15:0] rec_byte_num  	,//UDP���յ���Ч�ֽ��� ��λ:byte
    output             rd_sfpga_version_o,
    output             sfpga_comm_reset_o,

	output			   rec_cfg_pkg_total_en ,//����FPGA���ܰ���ʹ��
	output		[15:0] rec_cfg_pkg_total    ,//����FPGA���ܰ���
    output             rec_cfg_pkg_num_en,
    output      [15:0] rec_cfg_pkg_num   ,
	output             rec_cfg_done		,//UDP�������ݽ�������ź�
	output             rec_cfg_en 		,//UDP���յ�����ʹ���ź�
	output  	[7:0]  rec_cfg_data		,//UDP���յ�����
	output		  	   rec_cfg_byte_num_en	,//UDP���յ���Ч�ֽ���ʹ���ź�
	output  	[15:0] rec_cfg_byte_num	,//UDP���յ���Ч�ֽ��� ��λ:byte
	
	input  	  	 	   tx_start_en,//UDP���Ϳ�ʼ�ź�
	input  		[15:0] tx_byte_num,//UDP���͵���Ч�ֽ��� ��λ:byte 
	output             udp_tx_done,//UDP��������ź�
	output             tx_req     ,//UDP�����������ź�
	input  		[7:0]  tx_data     //UDP����������
    );

//parameter define
//������MAC��ַ 00-11-22-33-44-55
parameter  BOARD_MAC = 48'h0E_11_22_33_44_55;     
//������IP��ַ 192.168.1.10
parameter  BOARD_IP  = {8'd192,8'd168,8'd99,8'd14};  
//Ŀ��MAC��ַ ff_ff_ff_ff_ff_ff
parameter  DES_MAC   = 48'hff_ff_ff_ff_ff_ff;    
//Ŀ��IP��ַ 192.168.1.102     
parameter  DES_IP    = {8'd192,8'd168,8'd99,8'd1};  
//��������IO��ʱ,�˴�Ϊ0,������ʱ(���Ϊn,��ʾ��ʱn*78ps) 
parameter IDELAY_VALUE = 0;

//wire define
              
wire          gmii_rx_clk; //GMII����ʱ��
wire          gmii_rx_dv ; //GMII����������Ч�ź�
wire  [7:0]   gmii_rxd   ; //GMII��������
wire          gmii_tx_clk; //GMII����ʱ��
wire          gmii_tx_en ; //GMII��������ʹ���ź�
wire  [7:0]   gmii_txd   ; //GMII��������     

wire          arp_gmii_tx_en; //ARP GMII���������Ч�ź� 
wire  [7:0]   arp_gmii_txd  ; //ARP GMII�������
wire          arp_rx_done   ; //ARP��������ź�
wire          arp_rx_type   ; //ARP�������� 0:����  1:Ӧ��
wire  [47:0]  src_mac       ; //���յ�Ŀ��MAC��ַ
wire  [31:0]  src_ip        ; //���յ�Ŀ��IP��ַ    
wire          arp_tx_en     ; //ARP����ʹ���ź�
wire          arp_tx_type   ; //ARP�������� 0:����  1:Ӧ��
wire  [47:0]  des_mac       ; //���͵�Ŀ��MAC��ַ
wire  [31:0]  des_ip        ; //���͵�Ŀ��IP��ַ   
wire          arp_tx_done   ; //ARP��������ź�

wire          udp_gmii_tx_en; //UDP GMII���������Ч�ź� 
wire  [7:0]   udp_gmii_txd  ; //UDP GMII�������
// wire          rec_pkt_done  ; //UDP�������ݽ�������ź�
// wire          rec_en        ; //UDP���յ�����ʹ���ź�
// wire  [31:0]  rec_data      ; //UDP���յ�����
// wire		  rec_byte_num_en;//UDP���յ���Ч�ֽ���ʹ���ź�
// wire  [15:0]  rec_byte_num  ; //UDP���յ���Ч�ֽ��� ��λ:byte 
// wire  	  	 tx_start_en   ; //UDP���Ϳ�ʼ�ź�
// wire  [15:0]  tx_byte_num   ; //UDP���͵���Ч�ֽ��� ��λ:byte 
// wire          udp_tx_done   ; //UDP��������ź�
// wire          tx_req        ; //UDP�����������ź�
// wire  [31:0]  tx_data       ; //UDP����������

//*****************************************************
//**                    main code
//*****************************************************

// assign tx_start_en = rec_pkt_done;
// assign tx_byte_num = rec_byte_num;
assign des_mac = src_mac;
assign des_ip = src_ip;
assign eth_rst_n = rst_n;

assign eth_clk = gmii_rx_clk;

//GMII�ӿ�תRGMII�ӿ�
gmii_to_rgmii 
    #(
     .IDELAY_VALUE (IDELAY_VALUE)
     )
    u_gmii_to_rgmii(
	.idelay_clk	   (idelay_clk	),
	
    .gmii_rx_clk   (gmii_rx_clk ),
    .gmii_rx_dv    (gmii_rx_dv  ),
    .gmii_rxd      (gmii_rxd    ),
    .gmii_tx_clk   (gmii_tx_clk ),
    .gmii_tx_en    (gmii_tx_en  ),
    .gmii_txd      (gmii_txd    ),
    
    .rgmii_rxc     (eth_rxc     ),
    .rgmii_rx_ctl  (eth_rx_ctl  ),
    .rgmii_rxd     (eth_rxd     ),
    .rgmii_txc     (eth_txc     ),
    .rgmii_tx_ctl  (eth_tx_ctl  ),
    .rgmii_txd     (eth_txd     )
    );

//ARPͨ��
arp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //��������
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_arp(
    .rst_n         (rst_n  	   ),
                    
    .gmii_rx_clk   (gmii_rx_clk),
    .gmii_rx_dv    (gmii_rx_dv ),
    .gmii_rxd      (gmii_rxd   ),
    .gmii_tx_clk   (gmii_tx_clk),
    .gmii_tx_en    (arp_gmii_tx_en),
    .gmii_txd      (arp_gmii_txd),
                    
    .arp_rx_done   (arp_rx_done),
    .arp_rx_type   (arp_rx_type),
    .src_mac       (src_mac    ),
    .src_ip        (src_ip     ),
    .arp_tx_en     (arp_tx_en  ),
    .arp_tx_type   (arp_tx_type),
    .des_mac       (des_mac    ),
    .des_ip        (des_ip     ),
    .tx_done       (arp_tx_done)
    );

//UDPͨ��
udp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //��������
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_udp(
    .rst_n         (rst_n   	),  
    
    .gmii_rx_clk   (gmii_rx_clk ),           
    .gmii_rx_dv    (gmii_rx_dv  ),         
    .gmii_rxd      (gmii_rxd    ),                   
    .gmii_tx_clk   (gmii_tx_clk ), 
    .gmii_tx_en    (udp_gmii_tx_en),         
    .gmii_txd      (udp_gmii_txd),  

    .rec_pkt_done  (rec_pkt_done),    
    .rec_en        (rec_en      ),     
    .rec_data      (rec_data    ), 
	.rec_byte_num_en(rec_byte_num_en),
    .rec_byte_num  (rec_byte_num), 
    .rd_sfpga_version_o(rd_sfpga_version_o),
    .sfpga_comm_reset_o(sfpga_comm_reset_o),
	
	.rec_cfg_pkg_total_en(rec_cfg_pkg_total_en	),
	.rec_cfg_pkg_total	 (rec_cfg_pkg_total		),
    .rec_cfg_pkg_num_en  (rec_cfg_pkg_num_en    ),
    .rec_cfg_pkg_num     (rec_cfg_pkg_num       ),
	.rec_cfg_done    (rec_cfg_done		),      
    .rec_cfg_en      (rec_cfg_en  		),            
    .rec_cfg_data    (rec_cfg_data 		), 
	.rec_cfg_byte_num_en (rec_cfg_byte_num_en	),
    .rec_cfg_byte_num(rec_cfg_byte_num	),
	
    .tx_start_en   (tx_start_en ),        
    .tx_data       (tx_data     ),         
    .tx_byte_num   (tx_byte_num ),  
    .des_mac       (des_mac     ),
    .des_ip        (des_ip      ),    
    .tx_done       (udp_tx_done ),        
    .tx_req        (tx_req      )           
    ); 

//ͬ��FIFO
// sync_fifo_2048x32b u_sync_fifo_2048x32b (
    // .clk      (gmii_rx_clk),  // input wire clk
    // .rst      (~rst_n	 ),  // input wire rst
    // .din      (rec_data  ),  // input wire [31 : 0] din
    // .wr_en    (rec_en    ),  // input wire wr_en
    // .rd_en    (tx_req    ),  // input wire rd_en
    // .dout     (tx_data   ),  // output wire [31 : 0] dout
    // .full     (),            // output wire full
    // .empty    ()             // output wire empty
    // );    

//��̫������ģ��
eth_ctrl u_eth_ctrl(
    .clk            (gmii_rx_clk   ),
    .rst_n          (rst_n         ),

    .arp_rx_done    (arp_rx_done   ),
    .arp_rx_type    (arp_rx_type   ),
    .arp_tx_en      (arp_tx_en     ),
    .arp_tx_type    (arp_tx_type   ),
    .arp_tx_done    (arp_tx_done   ),
    .arp_gmii_tx_en (arp_gmii_tx_en),
    .arp_gmii_txd   (arp_gmii_txd  ),
                     
    .udp_gmii_tx_en (udp_gmii_tx_en),
    .udp_gmii_txd   (udp_gmii_txd  ),
                     
    .gmii_tx_en     (gmii_tx_en    ),
    .gmii_txd       (gmii_txd      )
    );

endmodule