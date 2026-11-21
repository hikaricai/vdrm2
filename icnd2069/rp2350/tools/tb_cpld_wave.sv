`timescale 1ns / 1ps
module tb_cpld_wave;
    reg CLK = 0;
    reg [31:0] pins = 0;
    wire [5:0] R1, G1, B1, R2, G2, B2, R3, G3, B3;
    wire [53:0] outputs = {B3,G3,R3,B2,G2,R2,B1,G1,R1};
    led dut (.CLK(CLK), .SEL({pins[11],pins[7],pins[3]}),
        .R({pins[8],pins[4],pins[0]}), .G({pins[9],pins[5],pins[1]}),
        .B({pins[10],pins[6],pins[2]}),
        .R1(R1),.G1(G1),.B1(B1),.R2(R2),.G2(G2),.B2(B2),
        .R3(R3),.G3(G3),.B3(B3));
    string path;
    integer file, status, count = 0, hz;
    reg [31:0] word;
    reg [53:0] expected;
    realtime half_period, setup_delay;
    initial begin
        if (!$value$plusargs("wave=%s", path)) $fatal(1, "missing wave");
        file = $fopen(path, "r");
        if (!file) $fatal(1, "cannot open wave");
        #100 CLK = 1; // The initial PULL rising edge clocks only parked zeros.
        #1;
        while (!$feof(file)) begin
            status = $fscanf(file, "%h %h %d\n", word, expected, hz);
            if (status != 3) $fatal(1, "invalid wave record");
            half_period = 500000000.0 / hz;
            // OUT executes after PULL's half-1 PIO cycles.
            setup_delay = half_period * ($test$plusargs("six_cycles") ? 2.0/3.0 : 0.5);
            #(setup_delay) pins = word;
            #(half_period - setup_delay) CLK = 0;
            #(half_period) CLK = 1;
            // ICND2069 reads BEFORE CPLD's posedge registers publish new RGB.
            if (outputs !== expected)
                $fatal(1, "word %0d pins=%h ICND SIN=%h expected=%h",
                    count, word, outputs, expected);
            count = count + 1;
        end
        $display("PASS actual RP2350 DMA -> CPLD RTL -> ICND rising-edge SIN: %0d clocks", count);
        $fclose(file);
        $finish;
    end
endmodule
