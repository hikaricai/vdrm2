`timescale 1ns / 1ps
`default_nettype none

// Ported from mbi5264/hdl/decoder.v.
// Addresses 0..8 select one destination; 9..15 broadcast.
// The existing all-ones initialization stream selects address 15.
module decoder (
    input logic [3:0] a,
    input logic v,
    output logic [8:0] o
);
    wire [8:0] selected = (a < 4'd9) ? (9'b1 << a) : 9'h1ff;

    assign o = v ? selected : '0;
endmodule

`default_nettype wire
