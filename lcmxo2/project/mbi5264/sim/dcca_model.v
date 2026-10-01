`timescale 1ns / 1ps
// Functional model for this project's permanently enabled clock buffers.
module DCCA(input wire CLKI, CE, output wire CLKO);
    assign CLKO = CLKI & CE;
    always @(CE)
        if (CE !== 1'b1)
            $fatal(1, "Project DCCA model only supports CE tied high");
endmodule
