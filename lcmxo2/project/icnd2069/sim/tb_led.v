`timescale 1ns / 1ps
module tb_led;
    reg CLK = 0;
    reg [2:0] SEL = 0, R = 0, G = 0, B = 0;
    wire [5:0] red [0:2], green [0:2], blue [0:2];
    reg [2:0] addresses [0:2];
    reg [5:0] er [0:2], eg [0:2], eb [0:2];
    integer checks = 0;
    led dut (
        .CLK(CLK), .SEL(SEL), .R(R), .G(G), .B(B),
        .R1(red[0]), .G1(green[0]), .B1(blue[0]),
        .R2(red[1]), .G2(green[1]), .B2(blue[1]),
        .R3(red[2]), .G3(green[2]), .B3(blue[2])
    );

    function [5:0] mask(input integer address);
        mask = address < 6 ? (6'b1 << address) : 6'b111111;
    endfunction

    task check(input string stage);
        for (integer bank = 0; bank < 3; bank++) begin
            if (red[bank] !== er[bank] || green[bank] !== eg[bank] ||
                blue[bank] !== eb[bank])
                $fatal(1, "%s bank=%0d address=%0d CLK=%b actual=%h/%h/%h expected=%h/%h/%h",
                    stage, bank, addresses[bank], CLK, red[bank], green[bank],
                    blue[bank], er[bank], eg[bank], eb[bank]);
        end
        checks++;
    endtask

    task edge_sample(input bit level);
        if (CLK == level)
            $fatal(1, "missing clock edge");
        for (integer bank = 0; bank < 3; bank++) begin
            er[bank] = R[bank] ? mask(addresses[bank]) : 6'b0;
            eg[bank] = G[bank] ? mask(addresses[bank]) : 6'b0;
            eb[bank] = B[bank] ? mask(addresses[bank]) : 6'b0;
        end
        CLK = level;
        #1;
        check("edge sample");
    endtask

    task colors(input integer pattern);
        R = pattern & 7;
        G = (pattern >> 3) & 7;
        B = (pattern >> 6) & 7;
    endtask

    // No RTL-internal state is used by this reference. Only complete externally
    // transmitted frames change its address, exactly after their eighth rise.
    task frame(input integer a, b, c, input bit valid_marker);
        reg [7:0] words [0:2];
        words[0] = (a << 5) | (valid_marker ? 15 : 14);
        words[1] = (b << 5) | (valid_marker ? 15 : 14);
        words[2] = (c << 5) | (valid_marker ? 15 : 14);
        for (integer k = 7; k >= 0; k--) begin
            SEL = {words[2][k], words[1][k], words[0][k]};
            colors(511);
            #2;
            check("before command rise");
            edge_sample(1);
            if (k == 0 && valid_marker) begin
                addresses[0] = a;
                addresses[1] = b;
                addresses[2] = c;
            end
            #2;
            check("address commit does not alter held RGB");
            edge_sample(0);
        end
    endtask

    initial begin
        for (integer bank = 0; bank < 3; bank++) begin
            addresses[bank] = 0;
            er[bank] = 0; eg[bank] = 0; eb[bank] = 0;
        end
        #2;
        check("power up");
        // A partial marker at cold start must not latch a synthetic address.
        SEL = 7;
        for (integer k = 0; k < 4; k++) begin
            edge_sample(1); edge_sample(0);
        end
        SEL = 0;
        for (integer k = 0; k < 8; k++) begin
            edge_sample(1); edge_sample(0);
        end
        // Every address, including 111; address suffix ones must not commit
        // early. Frames are back to back, without an extra CLK or reset.
        for (integer a = 0; a < 8; a++) begin
            frame(a, (a + 3) & 7, (a + 5) & 7, 1);
            // More ones after the marker must keep the committed address.
            SEL = 7;
            for (integer k = 0; k < 12; k++) begin
                edge_sample(1); edge_sample(0);
            end
            SEL = 0;
            for (integer pattern = 0; pattern < 512; pattern++) begin
                colors(pattern);
                #3; check("low phase hold");
                edge_sample(1);
                colors(pattern ^ 511);
                #2; check("high phase hold");
                edge_sample(0);
                colors(pattern);
                #4; check("low phase hold");
            end
            // A broken marker must not change the current address.
            frame((a + 1) & 7, (a + 2) & 7, (a + 4) & 7, 0);
        end
        // One bank receives a complete command; the other banks remain idle.
        for (integer bank = 0; bank < 3; bank++) begin
            for (integer a = 0; a < 8; a++) begin
                for (integer k = 7; k >= 0; k--) begin
                    SEL = 0;
                    SEL[bank] = (((a << 5) | 15) >> k) & 1;
                    edge_sample(1);
                    if (k == 0) addresses[bank] = a;
                    edge_sample(0);
                end
            end
        end
        SEL = 0;
        for (integer level = 0; level < 2; level++) begin
            if (CLK != level) edge_sample(level);
            for (integer p = 0; p < 512; p++) begin
                colors(p);
                #5; check("paused clock");
            end
        end
        $display("PASS: %0d output checks; eight-clock commands, all addresses/RGB patterns,", checks);
        $display("independent banks, adjacent frames, malformed/partial markers, long ones and dual-edge hold");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1, "simulation timeout");
    end
endmodule
