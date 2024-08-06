`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/27 13:40:10
// Design Name: 
// Module Name: udp
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

module udp	#(
	//������MAC��ַ 00-11-22-33-44-55
	parameter BOARD_MAC = 48'h00_11_22_33_44_55,
	//������IP��ַ 192.168.1.10
	parameter BOARD_IP  = {8'd192,8'd168,8'd1,8'd10},
	//Ŀ��MAC��ַ ff_ff_ff_ff_ff_ff
	parameter  DES_MAC   = 48'hff_ff_ff_ff_ff_ff,
	//Ŀ��IP��ַ 192.168.1.102
	parameter  DES_IP    = {8'd192,8'd168,8'd1,8'd102}
)
(
    input                rst_n       , //��λ�źţ��͵�ƽ��Ч
    //GMII�ӿ�
    input                gmii_rx_clk , //GMII��������ʱ��
    input                gmii_rx_dv  , //GMII����������Ч�ź�
    input        [7:0]   gmii_rxd    , //GMII��������
    input                gmii_tx_clk , //GMII��������ʱ��    
    output               gmii_tx_en  , //GMII���������Ч�ź�
    output       [7:0]   gmii_txd    , //GMII������� 
    //�û��ӿ�
    output               rec_pkt_done, //��̫���������ݽ�������ź�
    output               rec_en      , //��̫�����յ�����ʹ���ź�
    output       [7:0]   rec_data    , //��̫�����յ�����
	output				 rec_byte_num_en,//��̫�����յ���Ч�ֽ���ʹ���ź�
    output       [15:0]  rec_byte_num, //��̫�����յ���Ч�ֽ��� ��λ:byte 
    output               rd_sfpga_version_o,
    output               sfpga_comm_reset_o,
	
	output				 rec_cfg_pkg_total_en,//����FPGA���ܰ���ʹ��
	output		 [15:0]	 rec_cfg_pkg_total,	  //����FPGA���ܰ���
    output               rec_cfg_pkg_num_en,
    output       [15:0]  rec_cfg_pkg_num   ,
	
	output               rec_cfg_done, //��̫���������ݽ�������ź�
    output               rec_cfg_en  , //��̫�����յ�����ʹ���ź�
    output       [7:0]   rec_cfg_data, //��̫�����յ�����
	output				 rec_cfg_byte_num_en,//��̫�����յ���Ч�ֽ���ʹ���ź�
    output       [15:0]  rec_cfg_byte_num, //��̫�����յ���Ч�ֽ��� ��λ:byte 
	
    input                tx_start_en , //��̫����ʼ�����ź�
    input        [7:0]   tx_data     , //��̫������������  
    input        [15:0]  tx_byte_num , //��̫�����͵���Ч�ֽ��� ��λ:byte  
    input        [47:0]  des_mac     , //���͵�Ŀ��MAC��ַ
    input        [31:0]  des_ip      , //���͵�Ŀ��IP��ַ    
    output               tx_done     , //��̫����������ź�
    output               tx_req        //�����������ź�    
    );

//wire define
wire          crc_en  ; //CRC��ʼУ��ʹ��
wire          crc_clr ; //CRC���ݸ�λ�ź� 
wire  [7:0]   crc_d8  ; //�����У��8λ����

wire  [31:0]  crc_data; //CRCУ������
wire  [31:0]  crc_next; //CRC�´�У���������

//*****************************************************
//**                    main code
//*****************************************************

assign  crc_d8 = gmii_txd;

//��̫������ģ��    
udp_rx 
   #(
    .BOARD_MAC       (BOARD_MAC),         //��������
    .BOARD_IP        (BOARD_IP )
    )
   u_udp_rx(
    .clk             (gmii_rx_clk 		),        
    .rst_n           (rst_n       		),             
    .gmii_rx_dv      (gmii_rx_dv  		),                                 
    .gmii_rxd        (gmii_rxd    		),       
    .rec_pkt_done    (rec_pkt_done		),      
    .rec_en          (rec_en      		),            
    .rec_data        (rec_data    		), 
	.rec_byte_num_en (rec_byte_num_en	),
    .rec_byte_num    (rec_byte_num		),
    .rd_sfpga_version_o(rd_sfpga_version_o),
    .sfpga_comm_reset_o(sfpga_comm_reset_o),
	
	.rec_cfg_pkg_total_en(rec_cfg_pkg_total_en	),
	.rec_cfg_pkg_total	 (rec_cfg_pkg_total		),
    .rec_cfg_pkg_num_en  (rec_cfg_pkg_num_en    ),
    .rec_cfg_pkg_num     (rec_cfg_pkg_num       ),
	.rec_cfg_done    (rec_cfg_done		),      
    .rec_cfg_en_o    (rec_cfg_en  		),            
    .rec_cfg_data    (rec_cfg_data 		), 
	.rec_cfg_byte_num_en (rec_cfg_byte_num_en	),
    .rec_cfg_byte_num(rec_cfg_byte_num	)
    );                                    

//��̫������ģ��
udp_tx_v2#(
    .BOARD_MAC       (BOARD_MAC),         //��������
    .BOARD_IP        (BOARD_IP ),
    .DES_MAC_PARA    (DES_MAC  ),
    .DES_IP          (DES_IP   )
    )
   u_udp_tx(
    .clk             (gmii_tx_clk),        
    .rst_n           (rst_n      ),             
    .tx_start_en     (tx_start_en),                   
    .tx_data         (tx_data    ),           
    .tx_byte_num     (tx_byte_num),    
    .des_mac         (des_mac    ),
    .des_ip          (des_ip     ),    
    .crc_data        (crc_data   ),          
    .crc_next        (crc_next[31:24]),
    .tx_done         (tx_done    ),           
    .tx_req          (tx_req     ),            
    .gmii_tx_en      (gmii_tx_en ),         
    .gmii_txd        (gmii_txd   ),       
    .crc_en          (crc_en     ),            
    .crc_clr         (crc_clr    )            
    );                                      

//��̫������CRCУ��ģ��
crc32_d8   u_crc32_d8(
    .clk             (gmii_tx_clk),                      
    .rst_n           (rst_n      ),                          
    .data            (crc_d8     ),            
    .crc_en          (crc_en     ),                          
    .crc_clr         (crc_clr    ),                         
    .crc_data        (crc_data   ),                        
    .crc_next        (crc_next   )                         
    );

endmodule