`timescale 1ns / 1ps
`default_nettype none

// Three independent RGB banks, nine destinations per bank.
// Bus index 0 corresponds to the schematic's suffix 1 (R11/G11/B11).
module led (
    input logic CLK,
    input logic SEL_LAT,
    input logic [2:0] SEL,
    input logic [2:0] R,
    input logic [2:0] G,
    input logic [2:0] B,
    output logic [8:0] R1,
    output logic [8:0] G1,
    output logic [8:0] B1,
    output logic [8:0] R2,
    output logic [8:0] G2,
    output logic [8:0] B2,
    output logic [8:0] R3,
    output logic [8:0] G3,
    output logic [8:0] B3
);
    typedef logic [8:0] destinations_t;
    destinations_t routed [3][3]; // [bank][channel: R=0, G=1, B=2]
    logic pos = 1'b0;
    logic neg = 1'b0;

    logic late_pos = 1'b0;
    logic late_neg = 1'b0;

    logic main_clock;
    logic latch_clock;
    logic late_clk;
    assign late_clk = late_pos != late_neg;

`ifdef SYNTHESIS
    // Explicit routable globals for the 1200 database: automatic DCC1
    // placement fails dedicated routing; DCC0 and DCC2 route successfully.
    (* BEL = "X12/Y6/DCC0" *)
    DCCA main_clock_buffer (.CLKI(CLK), .CE(1'b1), .CLKO(main_clock));
    (* BEL = "X12/Y6/DCC2" *)
    DCCA latch_clock_buffer (.CLKI(SEL_LAT), .CE(1'b1), .CLKO(latch_clock));
`else
    assign main_clock = CLK;
    assign latch_clock = SEL_LAT;
`endif

    assign {B1, G1, R1} = {routed[0][2], routed[0][1], routed[0][0]};
    assign {B2, G2, R2} = {routed[1][2], routed[1][1], routed[1][0]};
    assign {B3, G3, R3} = {routed[2][2], routed[2][1], routed[2][0]};

    // Retain the original phase tracking and output hold behavior.
    always_ff @(posedge main_clock)
        if (late_pos == late_neg)
            late_pos <= ~late_pos;

    always_ff @(negedge main_clock)
        if (late_pos != late_neg)
            late_neg <= ~late_neg;

    always_ff @(posedge late_clk)
        if (pos == neg)
            pos <= ~pos;

    always_ff @(negedge late_clk)
        if (pos != neg)
            neg <= ~neg;

    for (genvar bank = 0; bank < 3; bank++) begin : banks
        logic [3:0] address;
        wire [2:0] rgb = {B[bank], G[bank], R[bank]};

        hc595 select_address (
            .clk(main_clock), .latch(latch_clock), .sr_in(SEL[bank]), .q(address)
        );

        for (genvar channel = 0; channel < 3; channel++) begin : channels
            destinations_t decoded;
            logic [7:0] held_neg = '0;
            logic [7:0] held_pos = '0;
            decoder select_output (
                .a(address), .v(rgb[channel]), .o(decoded)
            );
            always_ff @(posedge main_clock)
                held_pos <= decoded[7:0];

            always_ff @(negedge main_clock)
                held_neg <= decoded[7:0];

            assign routed[bank][channel] = {
                decoded[8], // The last destination retains the legacy direct path.
                (pos != neg) ? held_pos : held_neg
            };
        end
    end
endmodule

`default_nettype wire
