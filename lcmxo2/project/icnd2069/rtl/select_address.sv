`timescale 1ns / 1ps
`default_nettype none

// MSB first: a2 a1 a0 0 1 1 1 1. The zero separates address from marker.
// Only the first four ones after a zero commit; additional ones do not commit.
module select_address (
    input wire clk,
    input wire serial,
    output logic [2:0] address = '0
);
    logic [6:0] history = '0;
    logic [2:0] filled = '0;

    always_ff @(posedge clk) begin
        history <= {history[5:0], serial};
        if (filled != 3'd7)
            filled <= filled + 1'b1;
        if (filled == 3'd7 && history[3:0] == 4'b0111 && serial)
            address <= history[6:4];
    end
endmodule

`default_nettype wire
