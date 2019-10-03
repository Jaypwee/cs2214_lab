`timescale 1ns / 1ps
/*
 * Do not change Module name
*/
module main(clk);

    input clk;

    integer alive = 1;

    integer instructions[4:0];
    integer idx_instr;
    integer curr_in;

    integer R[31:0];
    integer idx_reg;

    integer M[19:0];
    integer idx_mem;
    integer start_mem = 32'h10004000;

    reg[5:0] opcode;
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] rd;
    reg[4:0] shamt;
    reg[5:0] func;

    reg[15:0] immediate;

    // used for always mimic
    integer j;

    initial begin

        instructions[0] = { 6'b0, 5'b01000, 5'b01000, 5'b01001, 5'b0, 6'h20 };
        instructions[1] = { 6'b0, 5'b01000, 5'b01001, 5'b01010, 5'b0, 6'h22 };
        instructions[2] = { 6'b0, 5'b01010, 5'b01000, 5'b01010, 5'b0, 6'h24 };
        instructions[3] = { 6'b0, 5'b01001, 5'b01010, 5'b01011, 5'b0, 6'h25 };
        instructions[4] = { 6'b0, 5'b01011, 5'b01000, 5'b01100, 5'b0, 6'h2a };

        instructions[5] = { 6'b100011, 5'b10000, 5'b01011, 16'b0000000000000100 };
        instructions[6] = { 6'b100011, 5'b10000, 5'b01010, 16'b0000000000001000 };
        instructions[7] = { 6'b100011, 5'b10000, 5'b10001, 16'b0000000000000000 };
        instructions[8] = { 6'b100011, 5'b11110, 5'b10110, 16'b0000000000000000 };
        instructions[9] = { 6'b100011, 5'b10000, 5'b11001, 16'b0000000000001100 };
        instructions[10] = { 6'b101011, 5'b00101, 5'b11101, 16'b0000000000000100 };
        instructions[11] = { 6'b101011, 5'b11000, 5'b00100, 16'b0000000000000000 };
        instructions[12] = { 6'b101011, 5'b00011, 5'b11110, 16'b0000000000000100 };
        instructions[13] = { 6'b101011, 5'b10000, 5'b10000, 16'b0000000000001000 };
        instructions[14] = { 6'b101011, 5'b00100, 5'b00010, 16'b0000000000001100 };

        for (idx_instr = 0; idx_instr < 5; idx_instr = idx_instr + 1) begin
            $display("binary: %b, hex: %h", instructions[idx_instr], instructions[idx_instr]);
        end
        idx_instr = 0;

        for (idx_reg = 0; idx_reg < 32; idx_reg = idx_reg + 1) begin
            R[idx_reg] = 0;
        end

        // v0 - v1
        R[2] = 32'h0;
        R[3] = 32'h10004014;

        // a0-a3
        R[5] = 32'h10004018;

        // t0-t7
        R[8] = 4;
        R[9] = 14;
        R[10] = 12;
        R[11] = 9;
        R[12] = 7;
        R[13] = 5;
        R[14] = 5;
        R[15] = 12;

        // s0-s7
        R[16] = 32'h10004004;

        // t8-t9
        R[24] = 32'h10004010;

        // fp
        R[30] = 32'h1000400C;

        // initialize memory
        M[0] = 94;
        M[1] =  98;
        M[2] = 109;
        M[3] = 102;
        M[4] = 100;
        M[5] =  99;
        M[6] = 164;
        M[7] = 183;
        M[8] = 203;
        M[9] = 192;
        M[10] = 243;
        M[11] = 229;
        M[12] =  50;
        M[13] =   1;
        M[14] = 106;
        M[15] =  82;
        M[16] = 441;
        M[17] = 414;
        M[18] = 384;
        M[19] = 419;

        $write("$zero:\t%d", R[0]);
        $display();

        $write("at:\t%d", R[1]);
        $display();

        $write("$v0-v1: ");
        for (idx_reg = 2; idx_reg < 4; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$a0-$a3: ");
        for (idx_reg = 4; idx_reg < 8; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$t0-$t7: ");
        for (idx_reg = 8; idx_reg < 16; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$s0-s7: ");
        for (idx_reg = 16; idx_reg < 24; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$t8-$t9: ");
        for (idx_reg = 24; idx_reg < 26; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$k0-$k1: ");
        for (idx_reg = 26; idx_reg < 28; idx_reg = idx_reg + 1) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$gp:\t%d", R[28]);
        $display();

        $write("sp:\t%d", R[29]);
        $display();

        $write("fp:\t%d", R[30]);
        $display();

        $write("ra:\t%d", R[31]);
        $display();

        // print memory:
        $display("Initialized Memory (Subset):");
        for (idx_mem = 0; idx_mem < 20; idx_mem = idx_mem + 1) begin
            $display("0x%h: %d", start_mem, M[idx_mem]);
        end

    end

    always @(posedge clk) begin

        /* approach
        iterate over instructions
        create a variable for each part of each instr
        ...
        */

        if (alive) begin

            curr_in = instructions[idx_instr];

            opcode = curr_in[31:26];

            $display("_________________________________________________________");
            $display("Current Instruction:\t%b; instruction number:%d", curr_in, idx_instr + 1);
            $display("Opcode:\t%b", opcode);

            // R-type
            if (opcode == 6'h0) begin

                rs = curr_in[25:21];
                rt = curr_in[20:16];
                rd = curr_in[15:11];
                shamt = curr_in[10:6];
                func = curr_in[5:0];

                if (func == 6'h20) begin
                    R[rd] = R[rs] + R[rt];
                end

                else if (func == 6'h22) begin
                    R[rd] = R[rs] - R[rt];
                end

                else if (func == 6'h24) begin
                    R[rd] = R[rs] & R[rt];
                end

                else if (func == 6'h25) begin
                    R[rd] = R[rs] | R[rt];
                end

                else if (func == 6'h2a) begin
                    if (R[rs] < R[rt]) R[rd] = 1;
                    else R[rd] = 0;
                end

                $display("Rs:\t%b", rs);
                $display("Rt:\t%b", rt);
                $display("Rd:\t%b", rd);
                $display("Shamt:\t%b", shamt);
                $display("Func:\t%b", func);

            end

            // I-type
            // opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100
            else begin

                $display("I-TYPE");

                rs = curr_in[25:21];
                rt = curr_in[20:16];
                immediate = curr_in[15:0];

                // lw rt, off(rs)
                if (opcode == 6'b100011) begin
                    R[rt] = M[rs + (immediate/4)];
                end
                // sw rt, off(rs)
                else if (opcode == 6'b101011) begin
                    M[rs + (immediate/4)] = R[rt];
                end

                $display("Rs:\t%b", rs);
                $display("Rt:\t%b", rt);
                $display("Immediate:\t%b", immediate);

            end

            idx_instr = idx_instr + 1;

            if (idx_instr > 4) begin
                alive = 0;
            end

            $write("$zero:\t%d", R[0]);
            $display();

            $write("at:\t%d", R[1]);
            $display();

            $write("$v0-v1: ");
            for (idx_reg = 2; idx_reg < 4; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$a0-$a3: ");
            for (idx_reg = 4; idx_reg < 8; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$t0-$t7: ");
            for (idx_reg = 8; idx_reg < 16; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$s0-s7: ");
            for (idx_reg = 16; idx_reg < 24; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$t8-$t9: ");
            for (idx_reg = 24; idx_reg < 26; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$k0-$k1: ");
            for (idx_reg = 26; idx_reg < 28; idx_reg = idx_reg + 1) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$gp:\t%d", R[28]);
            $display();

            $write("sp:\t%d", R[29]);
            $display();

            $write("fp:\t%d", R[30]);
            $display();

            $write("ra:\t%d", R[31]);
            $display();

            // print memory:
            $display("Memory");
            for (idx_mem = 0; idx_mem < 20; idx_mem = idx_mem + 1) begin
                $display("0x%h: %d", start_mem, M[idx_mem]);
            end

        end

    end

endmodule
