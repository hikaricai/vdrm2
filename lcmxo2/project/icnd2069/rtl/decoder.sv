`timescale 1ns / 1ps
`default_nettype none

// 0..5: suffix 1,3,5,7,9,11; 6 and 7: broadcast.
module decoder (
    input wire [2:0] a,
    input wire v,
    output wire [5:0] o
);
    wire [5:0] selected = (a < 3'd6) ? (6'b1 << a) : 6'b111111;
    assign o = v ? selected : 6'b0;
endmodule

`default_nettype wire
