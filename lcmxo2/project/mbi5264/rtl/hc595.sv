`timescale 1ns / 1ps
`default_nettype none

// Ported from mbi5264/hdl/hc595.v; retain both original sampling edges.
module hc595 (
    input logic clk,
    input logic latch,
    input logic sr_in,
    output logic [3:0] q = '0
);
    logic [3:0] shift_reg = '0;

    always_ff @(posedge clk)
        shift_reg <= {shift_reg[2:0], sr_in};

    always_ff @(negedge latch)
        q <= shift_reg;
endmodule

`default_nettype wire
