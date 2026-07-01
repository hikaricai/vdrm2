`timescale 1ns / 1ps

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    23:23:26 03/29/2026
// Design Name:
// Module Name:    main
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
module led (
    output wire [2:0] o_rgbs0,
    output wire [2:0] o_rgbs1,
    output wire [2:0] o_rgbs2,
    output wire [2:0] o_rgbs3,
    output wire [2:0] o_rgbs4,

    input  wire i_rgbs0,
    input  wire i_rgbs1,
    input  wire i_rgbs2,
    input  wire i_rgbs3,
    input  wire i_rgbs4,

    input  wire sr_addr,
    input  wire sr_addr2,
    input  wire sr_clk,
    input  wire latch,
    input  wire clk
);
// reg [1:0] o_rgbs_tmp_a[0:4];
reg [1:0] o_rgbs_tmp_b[0:4];
wire i_rgbs [0:4];
wire  [2:0] uo_rgbs [0:4];
wire  [2:0] o_rgbs [0:4];

// assign o_rgbs[0] = clk ? o_rgbs_tmp_a[0] : o_rgbs_tmp_b[0];

assign i_rgbs[0] = i_rgbs0;
assign i_rgbs[1] = i_rgbs1;
assign i_rgbs[2] = i_rgbs2;
assign i_rgbs[3] = i_rgbs3;
assign i_rgbs[4] = i_rgbs4;

assign o_rgbs0 = o_rgbs[0];
assign o_rgbs1 = o_rgbs[1];
assign o_rgbs2 = o_rgbs[2];
assign o_rgbs3 = o_rgbs[3];
assign o_rgbs4 = o_rgbs[4];

wire[1:0] shit_out_addr;
wire[1:0] shit_out_addr2;

hc595 u_hc595 (
        .clk(sr_clk),
        .latch(latch),
        .sr_in(sr_addr),
        .q(shit_out_addr)
);

hc595 u_hc595_2 (
        .clk(sr_clk),
        .latch(latch),
        .sr_in(sr_addr2),
        .q(shit_out_addr2)
);


genvar i;
generate
    for(i=0; i<3; i=i+1) begin : DECODER_ARR
        decoder u_decoder (
                .a(shit_out_addr),
                .v(i_rgbs[i]),
                .o(uo_rgbs[i])
                // .o(o_rgbs[i])
        );
    end
endgenerate
generate
    for(i=3; i<5; i=i+1) begin : DECODER_ARR2
        decoder u_decoder (
                .a(shit_out_addr2),
                .v(i_rgbs[i]),
                .o(uo_rgbs[i])
                // .o(o_rgbs[i])
        );
    end
endgenerate

// (* KEEP = "TRUE" *) wire [8:0] _clk2;
// (* KEEP = "TRUE" *) wire [8:0] _clk3;

// assign _clk2 = ~clk;
// assign _clk3 = ~_clk2;

reg pos;
reg neg;

always @(posedge clk) begin
    if (pos == neg) begin
        pos <= ~pos;
    end
end

always @(negedge clk) begin
    if (pos != neg) begin
        neg <= ~neg;
    end
end

generate
    for(i=0; i<5; i=i+1) begin : CLK_ARR
        assign o_rgbs[i][0:0] = uo_rgbs[i][0:0];
        // always @(posedge clk) begin
        //     o_rgbs_tmp_a[i] <= uo_rgbs[i][2:1];
        // end

        always @(negedge clk) begin
            o_rgbs_tmp_b[i] <= uo_rgbs[i][2:1];
        end
        assign o_rgbs[i][2:1] = (pos != neg) ? uo_rgbs[i][2:1] : o_rgbs_tmp_b[i];
    end
endgenerate

endmodule
