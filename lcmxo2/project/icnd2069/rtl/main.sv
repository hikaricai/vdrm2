`timescale 1ns / 1ps
`default_nettype none

module led (
    input wire CLK,
    input wire [2:0] SEL, R, G, B,
    output wire [5:0] R1, G1, B1,
    output wire [5:0] R2, G2, B2,
    output wire [5:0] R3, G3, B3
);
    wire [5:0] routed [0:2][0:2];
    wire main_clock;
`ifdef SYNTHESIS
    DCCA main_clock_buffer (.CLKI(CLK), .CE(1'b1), .CLKO(main_clock));
`else
    assign main_clock = CLK;
`endif

    assign {B1, G1, R1} = {routed[0][2], routed[0][1], routed[0][0]};
    assign {B2, G2, R2} = {routed[1][2], routed[1][1], routed[1][0]};
    assign {B3, G3, R3} = {routed[2][2], routed[2][1], routed[2][0]};

    for (genvar bank = 0; bank < 3; bank++) begin : banks
        wire [2:0] address;
        wire [2:0] rgb = {B[bank], G[bank], R[bank]};
        select_address selector (.clk(main_clock), .serial(SEL[bank]), .address(address));
        for (genvar channel = 0; channel < 3; channel++) begin : channels
            wire [5:0] decoded;
            logic [5:0] decoded_pos = '0;
            decoder select_output (.a(3), .v(rgb[channel]), .o(decoded));
            // Preserve the source project's current-edge, dual-edge sampling.
            // All six destinations hold between edges (no direct-path exception).
            always_ff @(negedge main_clock)
                decoded_pos <= decoded;
            assign routed[bank][channel] = decoded;
        end
    end
endmodule

`default_nettype wire
