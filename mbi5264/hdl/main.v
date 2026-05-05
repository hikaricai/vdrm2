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
    // 展开 o_rgbs [0:8] 每个元素 9 位
    output wire [8:0] o_rgbs0,
    output wire [8:0] o_rgbs1,
    output wire [8:0] o_rgbs2,

    // 展开 i_rgbs [0:2] 每个元素 3 位
    input  wire i_rgbs0,
    input  wire i_rgbs1,
    input  wire i_rgbs2,

    input  wire sr_addr,
    input  wire sr_clk,    // 移位寄存器时钟 SRCLK
    input  wire latch
);

wire i_rgbs [0:2];
wire  [8:0] o_rgbs [0:8];
assign o_rgbs0 = o_rgbs[0];
assign o_rgbs1 = o_rgbs[1];
assign o_rgbs2 = o_rgbs[2];
assign i_rgbs[0] = i_rgbs0;
assign i_rgbs[1] = i_rgbs1;
assign i_rgbs[2] = i_rgbs2;

wire[3:0] shit_out_addr;

hc595 u_hc595 (
        .clk(sr_clk),
        .latch(latch),
        .sr_in(sr_addr),
        .q(shit_out_addr)
);

genvar i;
generate
    for(i=0; i<3; i=i+1) begin : DECODER_ARR
        decoder u_decoder (
                .a(shit_out_addr),
                .v(i_rgbs[i]),
                .o(o_rgbs[i])
        );
    end
endgenerate

endmodule

