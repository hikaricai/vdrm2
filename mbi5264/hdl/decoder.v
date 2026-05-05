`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:26:26 03/29/2026 
// Design Name: 
// Module Name:    decoder 
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
module decoder(
    input  [3:0] a,   // addr
    input  v,
    output wire [8:0] o // 9位输出，wire类型
);

wire [8:0] y;

// -----------------------------
// y 的组合逻辑，用 assign 替代 always
// -----------------------------
assign y = (a == 4'b000) ? 9'b000000001 :
           (a == 4'b001) ? 9'b000000010 :
           (a == 4'b010) ? 9'b000000100 :
           (a == 4'b011) ? 9'b000001000 :
           (a == 4'b100) ? 9'b000010000 :
           (a == 4'b101) ? 9'b000100000 :
           (a == 4'b110) ? 9'b001000000 :
           (a == 4'b111) ? 9'b010000000 :
                            9'b100000000;  // 默认

// -----------------------------
// 输出 o = v ? y : 0
// -----------------------------
assign o = v ? y : 9'b0;

endmodule