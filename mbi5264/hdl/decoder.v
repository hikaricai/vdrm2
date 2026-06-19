`timescale 1ns / 1ps

`timescale 1ns / 1ps
module decoder(
    input  [1:0] a,
    input  v,
    output wire [2:0] o
);

wire [2:0] y;
// (* KEEP = "TRUE" *) wire [8:0] o1;
// (* KEEP = "TRUE" *) wire [8:0] o2;

assign y = (a == 2'b00) ? 3'b001 :
           (a == 2'b01) ? 3'b010 :
           (a == 2'b10) ? 3'b100 :
                          3'b111;

assign o = v ? y : 3'b0;




endmodule
