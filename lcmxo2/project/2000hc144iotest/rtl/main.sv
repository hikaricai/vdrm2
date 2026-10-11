`default_nettype none

module io_test (
    output logic [107:0] io
);
    logic osc_raw;
    logic osc_clk;
    logic [10:0] divider = '0;

    OSCH #(
        .NOM_FREQ("2.08")
    ) internal_oscillator (
        .STDBY(1'b0),
        .OSC(osc_raw)
    );

    // Automatic DCC0 placement cannot use dedicated routing on this device.
    (* BEL = "X13/Y8/DCC2" *)
    DCCA clock_buffer (
        .CLKI(osc_raw),
        .CE(1'b1),
        .CLKO(osc_clk)
    );

    always_ff @(posedge osc_clk)
        divider <= divider + 1'b1;

    // Sixteen interleaved groups take turns being high. Each pin has a
    // nominal 1.015625 kHz frequency and a 1/16 duty cycle.
    for (genvar i = 0; i < 108; i++) begin : generate_phase_outputs
        assign io[i] = divider[10:7] == (i % 16);
    end
endmodule

`default_nettype wire
