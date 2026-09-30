`timescale 1ns / 1ps

// Current contract: destinations 0..7 capture on BOTH CLK edges and hold
// between edges; destination 8 remains combinational. No T/2 pipeline delay.
// This is zero-delay RTL verification, not a physical glitch/timing proof.
module tb_led;
    reg CLK = 0;
    reg SEL_LAT = 0;
    reg [2:0] SEL = 0;
    reg [2:0] R = 0, G = 0, B = 0;
    wire [8:0] red [0:2], green [0:2], blue [0:2];
    reg [3:0] addresses [0:2];
    reg [7:0] expected_r [0:2], expected_g [0:2], expected_b [0:2];
    integer a, colors, checks = 0;
    integer rising_edges = 0, falling_edges = 0;
    integer late_rising_edges = 0, late_falling_edges = 0;
    reg monitoring = 0;
    realtime last_clock_edge;

    led dut (
        .CLK(CLK), .SEL_LAT(SEL_LAT), .SEL(SEL), .R(R), .G(G), .B(B),
        .R1(red[0]), .G1(green[0]), .B1(blue[0]),
        .R2(red[1]), .G2(green[1]), .B2(blue[1]),
        .R3(red[2]), .G3(green[2]), .B3(blue[2])
    );

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("build-open/tb_led.vcd");
            $dumpvars(0, tb_led);
        end
    end

    function [8:0] select_mask(input [3:0] address);
        begin
            if (address >= 9)
                select_mask = 9'h1ff;
            else begin
                select_mask = 0;
                select_mask[address] = 1'b1;
            end
        end
    endfunction

    task check_outputs(input string stage);
        integer bank;
        reg [8:0] mask;
        begin
            for (bank = 0; bank < 3; bank = bank + 1) begin
                mask = select_mask(addresses[bank]);
                if (red[bank] !== {mask[8] & R[bank], expected_r[bank]} ||
                    green[bank] !== {mask[8] & G[bank], expected_g[bank]} ||
                    blue[bank] !== {mask[8] & B[bank], expected_b[bank]})
                    $fatal(1, "%s: bank=%0d address=%0d CLK=%b RGB=%b/%b/%b outputs=%h/%h/%h",
                           stage, bank, addresses[bank], CLK, R, G, B,
                           red[bank], green[bank], blue[bank]);
            end
            checks = checks + 1;
        end
    endtask

    task set_colors(input integer pattern);
        begin
            R = pattern & 7;
            G = (pattern >> 3) & 7;
            B = (pattern >> 6) & 7;
        end
    endtask

    task clock_edge(input bit level);
        integer bank;
        reg [8:0] mask;
        begin
            if (CLK === level)
                $fatal(1, "testbench requested a clock edge without changing CLK");
            // Independent reference: use external data and the programmed address.
            // Expect THIS edge's sample, not the previous edge's sample.
            for (bank = 0; bank < 3; bank = bank + 1) begin
                mask = select_mask(addresses[bank]);
                expected_r[bank] = R[bank] ? mask[7:0] : 8'b0;
                expected_g[bank] = G[bank] ? mask[7:0] : 8'b0;
                expected_b[bank] = B[bank] ? mask[7:0] : 8'b0;
            end
            if (level)
                rising_edges = rising_edges + 1;
            else
                falling_edges = falling_edges + 1;
            CLK = level;
            #1; // Observe after all nonblocking/delta-cycle updates settle.
            check_outputs(level ? "rising-edge sample" : "falling-edge sample");
            if (dut.late_clk !== CLK || (dut.pos != dut.neg) !== CLK)
                $fatal(1, "phase mismatch: CLK=%b late_clk=%b pos/neg=%b/%b",
                       CLK, dut.late_clk, dut.pos, dut.neg);
            if (late_rising_edges != rising_edges || late_falling_edges != falling_edges)
                $fatal(1, "late_clk missed or added an edge");
        end
    endtask

    always @(CLK)
        last_clock_edge = $realtime;

    always @(posedge dut.late_clk)
        if (monitoring)
            late_rising_edges = late_rising_edges + 1;

    always @(negedge dut.late_clk)
        if (monitoring)
            late_falling_edges = late_falling_edges + 1;

    // Event-level checks catch glitches hidden by the #1 settled-value checks.
    // Every held output bit may change at most once at a CLK edge, and never
    // between edges. The ninth, intentionally direct, destination is excluded.
    for (genvar bank = 0; bank < 3; bank++) begin : transition_monitors
        wire [23:0] held_outputs = {red[bank][7:0], green[bank][7:0], blue[bank][7:0]};
        reg [23:0] previous = '0;
        reg [23:0] changed_at_edge = '0;
        reg [23:0] changed;

        always @(CLK)
            changed_at_edge = '0;

        always @(held_outputs) begin
            if (monitoring) begin
                if (^held_outputs === 1'bx)
                    $fatal(1, "unknown held output, bank=%0d", bank);
                if ($realtime != last_clock_edge)
                    $fatal(1, "held output changed between clock edges, bank=%0d", bank);
                changed = held_outputs ^ previous;
                if (|(changed & changed_at_edge))
                    $fatal(1, "held output toggled twice at one clock edge, bank=%0d", bank);
                changed_at_edge = changed_at_edge | changed;
            end
            previous = held_outputs;
        end
    end

    task program_addresses(input integer base, input bit latch_while_high);
        integer k;
        begin
            if (CLK)
                clock_edge(0);
            SEL_LAT = 1;
            set_colors(511);
            for (k = 3; k >= 0; k = k - 1) begin
                SEL[0] = (base >> k) & 1;
                SEL[1] = (((base + 3) & 15) >> k) & 1;
                SEL[2] = (((base + 7) & 15) >> k) & 1;
                #3;
                check_outputs("shifting: retain old address and held data");
                clock_edge(1);
                #2;
                if (k != 0 || !latch_while_high)
                    clock_edge(0);
            end
            #2;
            SEL_LAT = 0;
            addresses[0] = base;
            addresses[1] = (base + 3) & 15;
            addresses[2] = (base + 7) & 15;
            #2;
            // LAT alone changes only the direct destination; held outputs wait
            // for the next CLK edge. Cover LAT falling with CLK high AND low.
            check_outputs("address latch between clock edges");
        end
    endtask

    initial begin
        for (integer bank = 0; bank < 3; bank = bank + 1) begin
            addresses[bank] = 0;
            expected_r[bank] = 0;
            expected_g[bank] = 0;
            expected_b[bank] = 0;
        end
        #2;
        monitoring = 1;
        check_outputs("power-up");
        if (dut.late_clk !== 0 || (dut.pos != dut.neg) !== 0)
            $fatal(1, "phase registers did not initialize low");
        set_colors(511);
        #5;
        check_outputs("before first clock edge");

        // Every address in each bank, including address 8 and broadcasts 9..15.
        // Every RGB pattern is sampled at both polarities, with deliberately
        // different values on rising and falling edges and unequal half-periods.
        for (a = 0; a < 16; a = a + 1) begin
            program_addresses(a, a[0]);
            if (CLK)
                clock_edge(0);
            for (colors = 0; colors < 512; colors = colors + 1) begin
                set_colors(colors);
                #3;
                check_outputs("low-phase hold before rising edge");
                clock_edge(1);
                set_colors(colors ^ 511);
                #2;
                check_outputs("high-phase hold with changed inputs");
                clock_edge(0);
                set_colors(colors);
                #4;
                check_outputs("low-phase hold with changed inputs");
            end
        end

        // Pausing either clock level must not make destinations 0..7 transparent.
        for (integer level = 0; level < 2; level = level + 1) begin
            if (CLK != level[0])
                clock_edge(level[0]);
            for (integer pattern = 0; pattern < 8; pattern = pattern + 1) begin
                set_colors(pattern * 73);
                #10;
                check_outputs("paused clock");
            end
        end
        $display("PASS: %0d output checks; %0d rising / %0d falling samples; late_clk edges match",
                 checks, rising_edges, falling_edges);
        $display("PASS: 16 addresses x 512 RGB patterns x 3 banks; both-phase hold, LAT isolation, direct destination 9");
        $display("PASS: no extra held-output transitions observed in zero-delay RTL; physical glitches are not modeled");
        $finish;
    end

    initial begin
        #1000000;
        $fatal(1, "simulation timeout");
    end
endmodule
