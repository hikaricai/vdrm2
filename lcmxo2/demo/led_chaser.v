`timescale 1ns / 1ps
`default_nettype none

// MachXO2 LED chaser. Bind every led[] port in the board's LPF before programming.
module led_chaser #(
    parameter integer LED_COUNT = 4,        // >= 1; match the board's LED count
    parameter integer ACTIVE_LOW = 1,       // 1: drive low to light an LED
    parameter integer STEP_CYCLES = 520000  // >= 1; nominally 250 ms at 2.08 MHz
) (
    output wire [LED_COUNT-1:0] led
);
    wire clk;

    OSCH #(
        .NOM_FREQ("2.08")
    ) oscillator (
        .STDBY(1'b0),
        .OSC(clk),
        .SEDSTDBY()
    );

    localparam integer COUNTER_WIDTH =
        (STEP_CYCLES > 1) ? $clog2(STEP_CYCLES) : 1;
    localparam [COUNTER_WIDTH-1:0] LAST_CYCLE = STEP_CYCLES - 1;

    // Synthesis maps these initial values to the FPGA's power-up register state.
    reg [COUNTER_WIDTH-1:0] counter = 0;
    reg [LED_COUNT-1:0] lit = {{(LED_COUNT-1){1'b0}}, 1'b1};

    always @(posedge clk) begin
        if (counter == LAST_CYCLE) begin
            counter <= 0;
            lit <= (lit << 1) | (lit >> (LED_COUNT - 1));
        end else begin
            counter <= counter + 1'b1;
        end
    end

    assign led = ACTIVE_LOW ? ~lit : lit;
endmodule

`default_nettype wire
