`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/09/27 13:40:10
// Design Name: 
// Module Name: mdio_ctrl
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
module mdio_ctrl(
    input                clk           ,
    input                rst_n         ,
    input                op_done       , //��д���
    input        [15:0]  op_rd_data    , //����������
    input                op_rd_ack     , //��Ӧ���ź� 0:Ӧ�� 1:δӦ��
    output  reg          op_exec       , //������ʼ�ź�
    output  reg          op_rh_wl      , //�͵�ƽд���ߵ�ƽ��
    output  reg  [4:0]   op_addr       , //�Ĵ�����ַ
    output  reg  [15:0]  op_wr_data    , //д��Ĵ���������
	output 	reg			 link_error	   , //��·�Ͽ�������Э��δ���
	output	reg  [3:0]	 wr_cnt		   ,
    output       [1:0]   led             //LED��ָʾ��̫������״̬
    );

//reg define   
reg  [23:0]  timer_cnt;       //��ʱ������ 
reg          timer_done;      //��ʱ����ź�
reg          start_next;      //��ʼ����һ���Ĵ�������
reg          read_next;       //���ڶ���һ���Ĵ����Ĺ���
// reg          link_error;      //��·�Ͽ�������Э��δ���
reg  [2:0]   flow_cnt;        //���̿��Ƽ����� 
reg  [1:0]   speed_status;    //�������� 
reg			 negotiation_finished;
// reg	 [3:0]	 wr_cnt;

//wire define

//δ���ӻ�����ʧ��ʱled��ֵ00
// 01:10Mbps  10:100Mbps  11:1000Mbps 00���������
assign led = link_error ? 2'b00: speed_status;

//��ʱ����
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        timer_cnt <= 1'b0;
        timer_done <= 1'b0;
    end
    else if(~negotiation_finished) begin
        if(timer_cnt == 24'd1_000_000 - 1'b1) begin
            timer_done <= 1'b1;
            timer_cnt <= 'b0;//timer_cnt + 1'b1;
        end
		// else if(timer_cnt == 24'd1_000_000) begin
			// timer_done <= 1'b0;
            // timer_cnt <= timer_cnt;
		// end
        else begin
            timer_done <= 1'b0;
            timer_cnt <= timer_cnt + 1'b1;
        end
    end
	else begin
		timer_cnt <= 1'b0;
        timer_done <= 1'b0;
    end
end    

//������λ�źŶ�MDIO�ӿڽ�����λ,����ʱ��ȡ��̫��������״̬
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        flow_cnt <= 3'd0;
        speed_status <= 2'b00;
        op_exec <= 1'b0; 
        op_rh_wl <= 1'b0; 
        op_addr <= 1'b0;       
        op_wr_data <= 1'b0; 
        start_next <= 1'b0; 
        read_next <= 1'b0; 
        link_error <= 1'b0;
		negotiation_finished <= 1'b0;
		wr_cnt <= 'd0;
    end
    else if(~negotiation_finished) begin
		wr_cnt <= 'd0;
        op_exec <= 1'b0; 
        case(flow_cnt)
            2'd0 : begin
                if(timer_done) begin      //��ʱ���,��ȡ��̫������״̬
                    op_exec <= 1'b1; 
                    op_rh_wl <= 1'b1; 
                    op_addr <= 5'h01; 
                    flow_cnt <= 3'd2;
                end
                else if(start_next) begin       //��ʼ����һ���Ĵ�������ȡ��̫��ͨ���ٶ�
                    op_exec <= 1'b1; 
                    op_rh_wl <= 1'b1; 
                    op_addr <= 5'h1a;
                    flow_cnt <= 3'd2;
                    start_next <= 1'b0; 
                    read_next <= 1'b1; 
                end
            end    
            2'd2 : begin                       
                if(op_done) begin              //MDIO�ӿڶ��������
                    if(op_rd_ack == 1'b0 && read_next == 1'b0) //����һ���Ĵ������ӿڳɹ�Ӧ��
                        flow_cnt <= 3'd3;                      //������һ���Ĵ������ӿڳɹ�Ӧ��
                    else if(op_rd_ack == 1'b0 && read_next == 1'b1)begin 
                        read_next <= 1'b0;
                        flow_cnt <= 3'd4;
                    end
                    else begin
                        flow_cnt <= 3'd0;
                     end
                end    
            end
            2'd3 : begin                     
                flow_cnt <= 3'd0;          //��·����������Э�����
                if(op_rd_data[5] == 1'b1 && op_rd_data[2] == 1'b1)begin
                    start_next <= 1;
                    link_error <= 0;
                end
                else begin
                    link_error <= 1'b1;  
               end           
            end
            3'd4: begin
                flow_cnt <= 3'd0;
                if(op_rd_data[5:4] == 2'b10)
                    speed_status <= 2'b11; //1000Mbps
                else if(op_rd_data[5:4] == 2'b01) 
                    speed_status <= 2'b10; //100Mbps 
                else if(op_rd_data[5:4] == 2'b00) 
                    speed_status <= 2'b01; //10Mbps
                else
                    speed_status <= 2'b00; //������� 
					
				if(link_error)
					negotiation_finished <= 1'b0;
				else
					negotiation_finished <= 1'b1;
            end
        endcase
    end    
	// else begin
        // op_exec <= 1'b0; 
		// flow_cnt <= 'd0;
        // case(wr_cnt)
            // 4'd0: begin
				// op_exec <= 1'b1; 
				// op_rh_wl <= 1'b0; 
				// op_addr <= 5'h00; 
				// op_wr_data <= 16'b1001_0001_0100_0000;
				// wr_cnt <= 'd1;
            // end
			// 4'd1: begin
				// if(op_done) begin
					// wr_cnt <= 'd2;
				// end
				// else begin
					// wr_cnt <= 'd1;
				// end
			// end
			// 4'd2: begin
				// op_exec <= 1'b1; 
				// op_rh_wl <= 1'b1; 
				// op_addr <= 5'h01;
				// wr_cnt <= 'd3;
			// end
			// 4'd3: begin
				// if(op_done) begin              //MDIO�ӿڶ��������
                    // if(op_rd_ack == 1'b0)
                        // wr_cnt <= 'd4;
                    // else begin
                        // wr_cnt <= 'd2;
                     // end
                // end    
            // end
			// 4'd4: begin
				// if(op_rd_data[5] == 1'b1 && op_rd_data[2] == 1'b1)
					// wr_cnt <= 'd5;
                // else begin
                    // wr_cnt <= 'd2;
                // end   
            // end
			// 4'd5: begin
				// op_exec <= 1'b1; 
				// op_rh_wl <= 1'b1; 
				// op_addr <= 5'h1a;
				// wr_cnt <= 'd6;
			// end
			// 4'd6: begin
				// if(op_done) begin              //MDIO�ӿڶ��������
                    // if(op_rd_ack == 1'b0)
                        // wr_cnt <= 'd7;
                    // else begin
                        // wr_cnt <= 'd5;
                     // end
                // end    
            // end
			// 4'd7: begin
				// if(op_rd_data[5:4] == 2'b10)
                    // speed_status <= 2'b11; //1000Mbps
                // else if(op_rd_data[5:4] == 2'b01) 
                    // speed_status <= 2'b10; //100Mbps 
                // else if(op_rd_data[5:4] == 2'b00) 
                    // speed_status <= 2'b01; //10Mbps
                // else
                    // speed_status <= 2'b00; //������� 
			// end
		// endcase
    // end
end    

endmodule
