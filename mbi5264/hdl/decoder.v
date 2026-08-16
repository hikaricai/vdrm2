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
    input  [2:0] a,
    input  v,
    output wire [4:0] o
);

wire [4:0] y;
// (* KEEP = "TRUE" *) wire [8:0] o1;
// (* KEEP = "TRUE" *) wire [8:0] o2;

assign y = (a == 3'b000) ? 5'b00001 :
           (a == 3'b001) ? 5'b00010 :
           (a == 3'b010) ? 5'b00100 :
           (a == 3'b011) ? 5'b01000 :
           (a == 3'b100) ? 5'b10000 :
                           5'b11111 ;

assign o = v ? y : 5'b0;




endmodule
