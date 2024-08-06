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
    input                op_done       , //读写完成
    input        [15:0]  op_rd_data    , //读出的数据
    input                op_rd_ack     , //读应答信号 0:应答 1:未应答
    output  reg          op_exec       , //触发开始信号
    output  reg          op_rh_wl      , //低电平写，高电平读
    output  reg  [4:0]   op_addr       , //寄存器地址
    output  reg  [15:0]  op_wr_data    , //写入寄存器的数据
	output 	reg			 link_error	   , //链路断开或者自协商未完成
	output	reg  [3:0]	 wr_cnt		   ,
    output       [1:0]   led             //LED灯指示以太网连接状态
    );

//reg define   
reg  [23:0]  timer_cnt;       //定时计数器 
reg          timer_done;      //定时完成信号
reg          start_next;      //开始读下一个寄存器标致
reg          read_next;       //处于读下一个寄存器的过程
// reg          link_error;      //链路断开或者自协商未完成
reg  [2:0]   flow_cnt;        //流程控制计数器 
reg  [1:0]   speed_status;    //连接速率 
reg			 negotiation_finished;
// reg	 [3:0]	 wr_cnt;

//wire define

//未连接或连接失败时led赋值00
// 01:10Mbps  10:100Mbps  11:1000Mbps 00：其他情况
assign led = link_error ? 2'b00: speed_status;

//定时计数
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

//根据软复位信号对MDIO接口进行软复位,并定时读取以太网的连接状态
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
                if(timer_done) begin      //定时完成,获取以太网连接状态
                    op_exec <= 1'b1; 
                    op_rh_wl <= 1'b1; 
                    op_addr <= 5'h01; 
                    flow_cnt <= 3'd2;
                end
                else if(start_next) begin       //开始读下一个寄存器，获取以太网通信速度
                    op_exec <= 1'b1; 
                    op_rh_wl <= 1'b1; 
                    op_addr <= 5'h1a;
                    flow_cnt <= 3'd2;
                    start_next <= 1'b0; 
                    read_next <= 1'b1; 
                end
            end    
            2'd2 : begin                       
                if(op_done) begin              //MDIO接口读操作完成
                    if(op_rd_ack == 1'b0 && read_next == 1'b0) //读第一个寄存器，接口成功应答，
                        flow_cnt <= 3'd3;                      //读第下一个寄存器，接口成功应答
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
                flow_cnt <= 3'd0;          //链路正常并且自协商完成
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
                    speed_status <= 2'b00; //其他情况 
					
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
				// if(op_done) begin              //MDIO接口读操作完成
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
				// if(op_done) begin              //MDIO接口读操作完成
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
                    // speed_status <= 2'b00; //其他情况 
			// end
		// endcase
    // end
end    

endmodule
