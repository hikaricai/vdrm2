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
    input        clk,
    input        latch,
    input        sr_in,
    output reg [1:0] q
);

reg [1:0] shift_reg;

always @(posedge clk) begin
    shift_reg <= {shift_reg[0:0], sr_in};
end


always @(negedge latch) begin
    q <= shift_reg;
end

endmodule
