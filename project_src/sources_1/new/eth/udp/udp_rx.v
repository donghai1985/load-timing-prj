`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/27 13:40:10
// Design Name: 
// Module Name: udp_rx
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

module udp_rx	#(
	//������MAC��ַ 00-11-22-33-44-55
	parameter BOARD_MAC = 48'h00_11_22_33_44_55,
	//������IP��ַ 192.168.1.10
	parameter BOARD_IP  = {8'd192,8'd168,8'd1,8'd10}
)

(
    input                clk         ,    //ʱ���ź�
    input                rst_n       ,    //��λ�źţ��͵�ƽ��Ч
    
    input                gmii_rx_dv  ,    //GMII����������Ч�ź�
    input        [7:0]   gmii_rxd    ,    //GMII��������
	
	output  reg          rec_pkt_done       = 'd0, 	  //��̫���������ݽ�������ź�
    output  reg          rec_en             = 'd0,        //��̫�����յ�����ʹ���ź�
    output  reg  [7:0]   rec_data           = 'd0,        //��̫�����յ�����
	output	reg			 rec_byte_num_en    = 'd0, //��̫�����յ���Ч����ʹ���ź�
    output  reg  [15:0]  rec_byte_num       = 'd0,    //��̫�����յ���Ч���� ��λ:byte
    output  wire         rd_sfpga_version_o      ,
    output  wire         sfpga_comm_reset_o      ,
	
	output	reg			 rec_cfg_pkg_total_en   = 'd0,//����FPGA���ܰ���ʹ��
	output	reg	 [15:0]	 rec_cfg_pkg_total      = 'd0,	  //����FPGA���ܰ���
    output  reg          rec_cfg_pkg_num_en     = 'd0,
    output  reg  [15:0]  rec_cfg_pkg_num        = 'd0,
	
    output  reg          rec_cfg_done           = 'd0, 	  //��̫���������ݽ�������ź�
    output  wire         rec_cfg_en_o                ,    //��̫�����յ�����ʹ���ź�
    output  reg  [7:0]   rec_cfg_data           = 'd0,    //��̫�����յ�����
	output	reg			 rec_cfg_byte_num_en    = 'd0, //��̫�����յ���Ч����ʹ���ź�
    output  reg  [15:0]  rec_cfg_byte_num       = 'd0  //��̫�����յ���Ч���� ��λ:byte     
    );

localparam  st_idle     = 7'b000_0001; //��ʼ״̬���ȴ�����ǰ����
localparam  st_preamble = 7'b000_0010; //����ǰ����״̬ 
localparam  st_eth_head = 7'b000_0100; //������̫��֡ͷ
localparam  st_ip_head  = 7'b000_1000; //����IP�ײ�
localparam  st_udp_head = 7'b001_0000; //����UDP�ײ�
localparam  st_rx_data  = 7'b010_0000; //������Ч����
localparam  st_rx_end   = 7'b100_0000; //���ս���

localparam  ETH_TYPE    = 16'h0800   ; //��̫��Э������ IPЭ��

//reg define
reg  [6:0]   cur_state       = st_idle;
reg  [6:0]   next_state      = st_idle;
                             
reg          skip_en          = 'd0; //����״̬��תʹ���ź�
reg          error_en         = 'd0; //��������ʹ���ź�
reg  [4:0]   cnt              = 'd0; //�������ݼ�����
reg  [47:0]  des_mac          = 'd0; //Ŀ��MAC��ַ
reg  [15:0]  eth_type         = 'd0; //��̫������
reg  [31:0]  des_ip           = 'd0; //Ŀ��IP��ַ
reg  [5:0]   ip_head_byte_num = 'd0; //IP�ײ�����
reg  [15:0]  udp_byte_num     = 'd0; //UDP����
reg  [15:0]  data_cnt         = 'd0; //��Ч���ݼ���    
reg  [1:0]   rec_en_cnt      ; //8bitת32bit������

reg          rec_pkt_done_temp = 'd0;
reg          rec_en_temp = 'd0;
reg  [7:0]   rec_data_temp = 'd0;
reg			 rec_byte_num_en_temp = 'd0;
reg  [15:0]  rec_byte_num_temp = 'd0;

//*****************************************************
//**                    main code
//*****************************************************

//(����ʽ״̬��)ͬ��ʱ������״̬ת��
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        cur_state <= st_idle;  
    else
        cur_state <= next_state;
end

//����߼��ж�״̬ת������
always @(*) begin
    next_state = st_idle;
    case(cur_state)
        st_idle : begin                                     //�ȴ�����ǰ����
            if(skip_en) 
                next_state = st_preamble;
            else
                next_state = st_idle;    
        end
        st_preamble : begin                                 //����ǰ����
            if(skip_en) 
                next_state = st_eth_head;
            else if(error_en) 
                next_state = st_rx_end;    
            else
                next_state = st_preamble;    
        end
        st_eth_head : begin                                 //������̫��֡ͷ
            if(skip_en) 
                next_state = st_ip_head;
            else if(error_en) 
                next_state = st_rx_end;
            else
                next_state = st_eth_head;           
        end  
        st_ip_head : begin                                  //����IP�ײ�
            if(skip_en)
                next_state = st_udp_head;
            else if(error_en)
                next_state = st_rx_end;
            else
                next_state = st_ip_head;       
        end 
        st_udp_head : begin                                 //����UDP�ײ�
            if(skip_en)
                next_state = st_rx_data;
            else
                next_state = st_udp_head;    
        end                
        st_rx_data : begin                                  //������Ч����
            if(skip_en)
                next_state = st_rx_end;
            else
                next_state = st_rx_data;    
        end                           
        st_rx_end : begin                                   //���ս���
            if(skip_en)
                next_state = st_idle;
            else
                next_state = st_rx_end;          
        end
        default : next_state = st_idle;
    endcase                                          
end    

//ʱ���·����״̬���,������̫������
always @(posedge clk ) begin
    // if(!rst_n) begin
    //     skip_en <= 1'b0;
    //     error_en <= 1'b0;
    //     cnt <= 5'd0;
    //     des_mac <= 48'd0;
    //     eth_type <= 16'd0;
    //     des_ip <= 32'd0;
    //     ip_head_byte_num <= 6'd0;
    //     udp_byte_num <= 16'd0;
    //     data_cnt <= 16'd0;
    //     rec_en_cnt <= 2'd0;
    //     rec_en_temp <= 1'b0;
    //     rec_data_temp <= 'd0;
    //     rec_pkt_done_temp <= 1'b0;
	// 	rec_byte_num_en_temp <= 1'b0;
    //     rec_byte_num_temp <= 16'd0;
    // end
    // else 
    begin
        skip_en <= 1'b0;
        error_en <= 1'b0;  
        rec_en_temp <= 1'b0;
        rec_pkt_done_temp <= 1'b0;
		rec_byte_num_en_temp <= 1'b0;
        case(next_state)
            st_idle : begin
                if((gmii_rx_dv == 1'b1) && (gmii_rxd == 8'h55)) 
                    skip_en <= 1'b1;
            end
            st_preamble : begin
                if(gmii_rx_dv) begin                         //����ǰ����
                    cnt <= cnt + 5'd1;
                    if((cnt < 5'd6) && (gmii_rxd != 8'h55))  //7��8'h55  
                        error_en <= 1'b1;
                    else if(cnt==5'd6) begin
                        cnt <= 5'd0;
                        if(gmii_rxd==8'hd5)                  //1��8'hd5
                            skip_en <= 1'b1;
                        else
                            error_en <= 1'b1;    
                    end  
                end  
            end
            st_eth_head : begin
                if(gmii_rx_dv) begin
                    cnt <= cnt + 5'b1;
                    if(cnt < 5'd6) 
                        des_mac <= {des_mac[39:0],gmii_rxd}; //Ŀ��MAC��ַ
                    else if(cnt == 5'd12) 
                        eth_type[15:8] <= gmii_rxd;          //��̫��Э������
                    else if(cnt == 5'd13) begin
                        eth_type[7:0] <= gmii_rxd;
                        cnt <= 5'd0;
                        //�ж�MAC��ַ�Ƿ�Ϊ������MAC��ַ���߹�����ַ
                        if(((des_mac == BOARD_MAC) ||(des_mac == 48'hff_ff_ff_ff_ff_ff))
                       && eth_type[15:8] == ETH_TYPE[15:8] && gmii_rxd == ETH_TYPE[7:0])            
                            skip_en <= 1'b1;
                        else
                            error_en <= 1'b1;
                    end        
                end  
            end
            st_ip_head : begin
                if(gmii_rx_dv) begin
                    cnt <= cnt + 5'd1;
                    if(cnt == 5'd0)
                        ip_head_byte_num <= {gmii_rxd[3:0],2'd0};
					else if(cnt == 5'd9)
						if(gmii_rxd == 8'h11)	//ICMP:1,TCP:6,UDP:17
							error_en <= 1'b0;
						else
							error_en <= 1'b1;
                    else if((cnt >= 5'd16) && (cnt <= 5'd18))
                        des_ip <= {des_ip[23:0],gmii_rxd};   //Ŀ��IP��ַ
                    else if(cnt == 5'd19) begin
                        des_ip <= {des_ip[23:0],gmii_rxd}; 
                        //�ж�IP��ַ�Ƿ�Ϊ������IP��ַ
                       if((des_ip[23:0] == BOARD_IP[31:8]) && (gmii_rxd == BOARD_IP[7:0])) begin  
                            if(cnt == ip_head_byte_num - 1'b1) begin
                                skip_en <=1'b1;                     
                                cnt <= 5'd0;
                            end
							else begin
								skip_en <=1'b0;	
							end
                        end    
                        else begin            
                        //IP����ֹͣ��������                        
                            error_en <= 1'b1;               
                            cnt <= 5'd0;
                        end
                    end                          
                    else if(cnt == ip_head_byte_num - 1'b1) begin 
                        skip_en <=1'b1;                      //IP�ײ��������
                        cnt <= 5'd0;                    
                    end    
                end                                
            end 
            st_udp_head : begin
                if(gmii_rx_dv) begin
                    cnt <= cnt + 5'd1;
                    if(cnt == 5'd4)
                        udp_byte_num[15:8] <= gmii_rxd;      //����UDP�ֽڳ��� 
                    else if(cnt == 5'd5)
                        udp_byte_num[7:0] <= gmii_rxd;
                    else if(cnt == 5'd7) begin
                        //��Ч�����ֽڳ��ȣ���UDP�ײ�8���ֽڣ����Լ�ȥ8��
                        rec_byte_num_temp <= udp_byte_num - 16'd8;  
						rec_byte_num_en_temp <= 1'b1;
                        skip_en <= 1'b1;
                        cnt <= 5'd0;
                    end  
                end                 
            end          
            st_rx_data : begin         
                //�������ݣ�ת����32bit            
                if(gmii_rx_dv) begin
                    data_cnt <= data_cnt + 16'd1;
                    rec_en_cnt <= rec_en_cnt + 2'd1;
                    if(data_cnt == rec_byte_num_temp - 1) begin
                        skip_en <= 1'b1;                    //��Ч���ݽ������
                        data_cnt <= 16'd0;
                        rec_en_cnt <= 2'd0;
                        rec_pkt_done_temp <= 1'b1;               
                        // rec_en_temp <= 1'b1;                     
                    end   
					rec_en_temp 	<= 1'b1; 
					rec_data_temp	<= gmii_rxd;
                    //���յ������ݷ�����rec_data�ĸ�λ,���Ե����ݲ���4�ı���ʱ,
                    //��λ����Ϊ��Ч���ݣ��ɸ�����Ч�ֽ������ж�(rec_byte_num)
                    /*if(rec_en_cnt == 2'd0)
                        rec_data_temp[31:24] <= gmii_rxd;
                    else if(rec_en_cnt == 2'd1)
                        rec_data_temp[23:16] <= gmii_rxd;
                    else if(rec_en_cnt == 2'd2) 
                        rec_data_temp[15:8] <= gmii_rxd;        
                    else if(rec_en_cnt==2'd3) begin
                        rec_en_temp <= 1'b1;
                        rec_data_temp[7:0] <= gmii_rxd;
                    end  */  
                end  
				else begin
					rec_en_temp 	<= 1'b0; 
					data_cnt 		<= data_cnt;
				end
            end    
            st_rx_end : begin                               //�������ݽ������   
                if(gmii_rx_dv == 1'b0 && skip_en == 1'b0)
                    skip_en <= 1'b1; 
            end    
            default : ;
        endcase                                                        
    end
end

reg	[2:0]	rec_data_process_state = 'd0;
reg	[7:0]	rec_data_temp_delay1;
reg	[7:0]	rec_data_temp_delay2;
reg			rec_en_temp_delay1;
reg			rec_en_temp_delay2;
reg			rec_pkt_done_temp_delay1;
reg			rec_pkt_done_temp_delay2;

always @(posedge clk)
begin
    begin
		rec_data_temp_delay1	<=	rec_data_temp;
		rec_data_temp_delay2	<=	rec_data_temp_delay1;
		rec_en_temp_delay1		<=	rec_en_temp;
		rec_en_temp_delay2		<=	rec_en_temp_delay1;
		rec_pkt_done_temp_delay1<=	rec_pkt_done_temp;
		rec_pkt_done_temp_delay2<=	rec_pkt_done_temp_delay1;
	end
end

always @(posedge clk)
begin
    // if(!rst_n) begin
	// 	rec_data_process_state	<=	'd0;
	// end
	// else 
    begin
	case(rec_data_process_state)
	3'd0: begin
		if(rec_byte_num_en_temp) begin
			rec_data_process_state	<=	'd1;
		end
		else begin
			rec_data_process_state	<=	'd0;
		end
	end
	3'd1: begin
		if(rec_en_temp && (data_cnt == 'd4)) begin
			if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h01))
				rec_data_process_state	<=	'd2;
			else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h02))
				rec_data_process_state	<=	'd3;
            else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h04))
                rec_data_process_state	<=	'd5;
            else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h05))
                rec_data_process_state	<=	'd6;
			else
				rec_data_process_state	<=	'd4;
		end
		else begin
			rec_data_process_state	<=	rec_data_process_state;
		end
	end
	3'd2: begin
		if(rec_pkt_done_temp) begin
			rec_data_process_state	<=	3'd0;
		end
		else begin
			rec_data_process_state	<=	rec_data_process_state;
		end
	end
	3'd3: begin
		if(rec_pkt_done_temp) begin
			rec_data_process_state	<=	3'd0;
		end
		else begin
			rec_data_process_state	<=	rec_data_process_state;
		end
	end
	3'd4: begin
		if(rec_pkt_done_temp_delay2) begin
			rec_data_process_state	<=	3'd0;
		end
		else begin
			rec_data_process_state	<=	rec_data_process_state;
		end
	end
    3'd5: begin
        if(rec_pkt_done_temp_delay2) begin
            rec_data_process_state  <=  3'd0;
        end
        else begin
            rec_data_process_state  <=  rec_data_process_state;
        end
    end
    3'd6: begin
        if(rec_pkt_done_temp_delay2) begin
            rec_data_process_state  <=  3'd0;
        end
        else begin
            rec_data_process_state  <=  rec_data_process_state;
        end
    end
	default: begin
		rec_data_process_state	<=	'd0;
	end
	endcase
	end
end

reg          rec_cfg_en = 'd0;
always @(posedge clk)
begin
    // if(!rst_n) begin
	// 	rec_cfg_done		<=	'd0;
	// 	rec_cfg_en			<=	'd0;
	// 	rec_cfg_data		<=	'd0;
	// 	rec_cfg_byte_num_en	<=	'd0;
	// 	rec_cfg_byte_num	<=	'd0;
	// end
	// else 
    begin
	case(rec_data_process_state)
	3'd0: begin
		rec_cfg_done		<=	'd0;
		rec_cfg_en			<=	'd0;
		rec_cfg_data		<=	'd0;
		rec_cfg_byte_num_en	<=	'd0;
		if(rec_byte_num_en_temp)
			rec_cfg_byte_num	<=	rec_byte_num_temp;
		else
			rec_cfg_byte_num	<=	'd0;
	end
	3'd1: begin
		if(rec_en_temp && (data_cnt == 'd4)) begin
			if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h01)) begin
				rec_cfg_byte_num	<=	rec_cfg_byte_num - 'd4;
				rec_cfg_byte_num_en	<=	1'b1;
			end
			else begin
				rec_cfg_byte_num	<=	'd0;
				rec_cfg_byte_num_en	<=	1'b0;
			end
		end
		else begin
			rec_cfg_byte_num	<=	rec_cfg_byte_num;
			rec_cfg_byte_num_en	<=	1'b0;
		end
	end
	3'd2: begin
        if(rec_en_temp && (data_cnt == 'd5))begin
            rec_cfg_pkg_num    <= {rec_data_temp,rec_cfg_pkg_num[7:0]};
            rec_cfg_pkg_num_en <= 'd0;
        end
        else if(rec_en_temp && (data_cnt == 'd6))begin
            rec_cfg_pkg_num    <= {rec_cfg_pkg_num[15:8],rec_data_temp};
            rec_cfg_pkg_num_en <= 'd1;
        end
        else begin
            rec_cfg_pkg_num_en <= 'd0;
            
        end

        rec_cfg_done        <=    rec_pkt_done_temp;
        rec_cfg_en          <=    rec_en_temp && (data_cnt >= 'd7);
        rec_cfg_data        <=    rec_data_temp;
        rec_cfg_byte_num_en <=    1'b0;
	end
	default: begin
		rec_cfg_done		<=	'd0;
		rec_cfg_en			<=	'd0;
		rec_cfg_data		<=	'd0;
		rec_cfg_byte_num_en	<=	'd0;
		rec_cfg_byte_num	<=	'd0;
	end
	endcase
	end
end

reg rec_cfg_en_d = 'd0;
always @(posedge clk) begin
    rec_cfg_en_d <= rec_cfg_en;
end
assign rec_cfg_en_o = rec_cfg_en || rec_cfg_en_d;
	
always @(posedge clk)
begin
    // if(!rst_n) begin
	// 	rec_cfg_pkg_total_en		<=	'd0;
	// 	rec_cfg_pkg_total			<=	'd0;
	// end
	// else 
    // begin
	case(rec_data_process_state)
	3'd3: begin
		if(rec_en_temp && (data_cnt == 'd5)) begin
			rec_cfg_pkg_total		<=	{rec_data_temp,rec_cfg_pkg_total[7:0]};
			rec_cfg_pkg_total_en	<=	'd0;
		end
		else if(rec_en_temp && (data_cnt == 'd6)) begin
			rec_cfg_pkg_total	<=	{rec_cfg_pkg_total[15:8],rec_data_temp};
			rec_cfg_pkg_total_en	<=	'd1;
		end
		else begin
			rec_cfg_pkg_total		<=	rec_cfg_pkg_total;
			rec_cfg_pkg_total_en	<=	'd0;
		end
	end
	default: begin
		rec_cfg_pkg_total_en		<=	'd0;
		rec_cfg_pkg_total			<=	'd0;
	end
	endcase
	// end
end

always @(posedge clk )
begin
    // if(!rst_n) begin
	// 	rec_pkt_done	<=	'd0;
	// 	rec_en			<=	'd0;
	// 	rec_data		<=	'd0;
	// 	rec_byte_num_en	<=	'd0;
	// 	rec_byte_num	<=	'd0;
	// end
	// else 
    begin
	case(rec_data_process_state)
	3'd0: begin
		rec_pkt_done	<=	'd0;
		rec_en			<=	'd0;
		rec_data		<=	'd0;
		rec_byte_num_en	<=	'd0;
		if(rec_byte_num_en_temp)
			rec_byte_num	<=	rec_byte_num_temp;
		else
			rec_byte_num	<=	'd0;
	end
	3'd1: begin
		if(rec_en_temp && (data_cnt == 'd4)) begin
			if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h01)) begin
				rec_byte_num	<=	'd0;
				rec_byte_num_en	<=	1'b0;
			end
			else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h02)) begin
				rec_byte_num	<=	'd0;
				rec_byte_num_en	<=	1'b0;
			end
            else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h04)) begin
                rec_byte_num	<=	'd0;
                rec_byte_num_en	<=	1'b0;
            end
            else if((rec_data_temp_delay1 == 'h00) && (rec_data_temp == 'h05)) begin
                rec_byte_num	<=	'd0;
                rec_byte_num_en	<=	1'b0;
            end
			else begin
				rec_byte_num	<=	rec_byte_num - 'd2;
				rec_byte_num_en	<=	1'b1;
			end
		end
		else begin
			rec_byte_num	<=	rec_byte_num;
			rec_byte_num_en	<=	1'b0;
		end
	end
	3'd4: begin
		rec_pkt_done	<=	rec_pkt_done_temp_delay2;
		rec_en			<=	rec_en_temp_delay2;
		rec_data		<=	rec_data_temp_delay2;
		rec_byte_num	<=	rec_cfg_byte_num;
		rec_byte_num_en	<=	1'b0;
	end
	default: begin
		rec_pkt_done	<=	'd0;
		rec_en			<=	'd0;
		rec_data		<=	'd0;
		rec_byte_num_en	<=	'd0;
		rec_byte_num	<=	'd0;
	end
	endcase
	end
end

assign rd_sfpga_version_o = rec_data_process_state=='d5;
assign sfpga_comm_reset_o = rec_data_process_state=='d6;

// ila_udp_rx	ila_udp_rx_inst(
// 	.clk(clk),
// 	.probe0({gmii_rx_dv,rec_pkt_done,rec_en,rec_byte_num_en,rec_cfg_pkg_total_en,rec_cfg_done,rec_cfg_en,rec_cfg_byte_num_en}),
// 	.probe1(gmii_rxd),
// 	.probe2(rec_data),
// 	.probe3(rec_byte_num),
// 	.probe4(rec_cfg_pkg_total),
// 	.probe5(rec_cfg_data),
// 	.probe6(rec_cfg_byte_num),
// 	.probe7({cur_state,rec_data_process_state,6'd0})
// );


endmodule