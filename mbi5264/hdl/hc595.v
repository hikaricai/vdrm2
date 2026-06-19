`timescale 1ns / 1ps

`timescale 1ns / 1ps
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
