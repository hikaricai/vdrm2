`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:24:04 03/29/2026 
// Design Name: 
// Module Name:    hc595 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module hc595(
    input        clk,      // ��λ�Ĵ���ʱ�� SRCLK
    input        latch,    // �洢�Ĵ������� RCLK
    input        sr_in,    // �������� SER
    output reg [3:0] q     // ������� Q0-Q7
);

reg [3:0] shift_reg; // ��λ�Ĵ����ڲ�״̬

always @(posedge clk) begin
    shift_reg <= {shift_reg[2:0], sr_in}; // ����һλ�����뵽���λ
end

// ����Ĵ���
always @(negedge latch) begin
    q <= shift_reg; // ����λ�Ĵ���������������������
end

endmodule