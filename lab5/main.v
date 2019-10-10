`timescale 1ns / 1ps

module main(clk);

    input clk;

    integer alive = 1;

    integer curr_in;

    integer R[31:0];
    integer idx_reg;

    integer M[99:0];
    integer start_pc = 32'h00400000;
    integer pc;
    integer pc_print;
    integer addr;

    reg[5:0] opcode;
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] rd;
    reg[4:0] shamt;
    reg[5:0] func;

    reg[15:0] immediate;
    integer pc_as_index; // will never be assigned to except at start of always block
    integer print_index;

    integer instr_num = 0; // used to keep track of amount of instructions executed

    initial begin

        M[0] = { 6'b000000, 5'b01000, 5'b01000, 5'b01001, 5'b00000, 6'b100000 };
        M[1] = { 6'b000000, 5'b01000, 5'b01001, 5'b01010, 5'b00000, 6'b100010 };
        M[2] = { 6'b000000, 5'b01010, 5'b01000, 5'b01010, 5'b00000, 6'b100100 };
        M[3] = { 6'b000000, 5'b01001, 5'b01010, 5'b01011, 5'b00000, 6'b100101 };
        M[4] = { 6'b000000, 5'b01011, 5'b01000, 5'b01100, 5'b00000, 6'b101010 };
        M[5] = { 6'b100011, 5'b10000, 5'b01011, 5'b00000, 5'b00000, 6'b000100 };
        M[6] = { 6'b100011, 5'b10000, 5'b01010, 5'b00000, 5'b00000, 6'b001000 };
        M[7] = { 6'b100011, 5'b10000, 5'b10001, 5'b00000, 5'b00000, 6'b000000 };
        M[8] = { 6'b100011, 5'b11110, 5'b10110, 5'b00000, 5'b00000, 6'b000000 };
        M[9] = { 6'b100011, 5'b10000, 5'b11001, 5'b00000, 5'b00000, 6'b001100 };
        M[10] = { 6'b101011, 5'b00101, 5'b11101, 5'b00000, 5'b00000, 6'b000100 };
        M[11] = { 6'b101011, 5'b11000, 5'b00100, 5'b00000, 5'b00000, 6'b000000 };
        M[12] = { 6'b101011, 5'b00011, 5'b11110, 5'b00000, 5'b00000, 6'b000100 };
        M[13] = { 6'b101011, 5'b10000, 5'b10000, 5'b00000, 5'b00000, 6'b001000 };
        M[14] = { 6'b101011, 5'b00100, 5'b00010, 5'b00000, 5'b00000, 6'b001100 };
        M[15] = { 6'b000100, 5'b10000, 5'b10000, 5'b00000, 5'b00000, 6'b000010 };
        M[16] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[17] = { 6'b000011, 5'b00000, 5'b10000, 5'b00000, 5'b00000, 6'b010110 };
        M[18] = { 6'b000101, 5'b10000, 5'b01000, 5'b00000, 5'b00000, 6'b000011 };
        M[19] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[20] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[21] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[22] = { 6'b000111, 5'b11100, 5'b00000, 5'b00000, 5'b00000, 6'b000010 };
        M[23] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[24] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[25] = { 6'b000010, 5'b00000, 5'b10000, 5'b00000, 5'b00000, 6'b010000 };
        M[26] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[27] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[28] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[29] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[30] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[31] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[32] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[33] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[34] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[35] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[36] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[37] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[38] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[39] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[40] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[41] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[42] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[43] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[44] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[45] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[46] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[47] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[48] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[49] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000000 };
        M[50] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b001011 };
        M[51] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b000110 };
        M[52] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b001011 };
        M[53] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b010011 };
        M[54] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b101010 };
        M[55] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b010011 };
        M[56] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b100100 };
        M[57] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b101110 };
        M[58] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b111111 };
        M[59] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b111110 };
        M[60] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b101000 };
        M[61] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b001101 };
        M[62] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b011110 };
        M[63] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110111 };
        M[64] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110111 };
        M[65] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b010010 };
        M[66] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b101011 };
        M[67] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b000011 };
        M[68] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000110 };
        M[69] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b011011 };
        M[70] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b001010 };
        M[71] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b011000 };
        M[72] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b001011 };
        M[73] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b010110 };
        M[74] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b000110 };
        M[75] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b000011 };
        M[76] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b010100 };
        M[77] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b100111 };
        M[78] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110001 };
        M[79] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b111000 };
        M[80] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110011 };
        M[81] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000011 };
        M[82] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b110000 };
        M[83] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110101 };
        M[84] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b111100 };
        M[85] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b000010 };
        M[86] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110101 };
        M[87] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b111010 };
        M[88] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b001010 };
        M[89] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b001011 };
        M[90] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b111000 };
        M[91] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b000011 };
        M[92] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b010011 };
        M[93] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b010000 };
        M[94] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b100011 };
        M[95] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b101101 };
        M[96] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b101100 };
        M[97] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 6'b110011 };
        M[98] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00001, 6'b011110 };
        M[99] = { 6'b000000, 5'b00000, 5'b00000, 5'b00000, 5'b00010, 6'b000110 };

        pc = start_pc;
        for (print_index = 0; print_index < 100; print_index = print_index + 1) begin
            $display("binary: %b, hex: %h", M[print_index], M[print_index]);
        end

        // $zero-$ra; fill with 0's
        for (idx_reg = 0; idx_reg < 32; idx_reg = idx_reg + 1) begin
            R[idx_reg] = 0;
        end

        // v0 - v1
        R[3] = 32'h10004014;

        // a0-a3
        R[4] = 32'h10004014;
        R[5] = 32'h10004000;

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
        pc_print = start_pc;
        for (print_index = 0; print_index < 100; print_index = print_index + 1) begin
            $display("0x%h: %d", pc_print, M[print_index]);
            pc_print = pc_print + 4;
        end

    end

    always @(posedge clk) begin

        if (alive) begin

            // this is the only line that pc_as_index will be modified in
            pc_as_index = (pc - start_pc) / 4;

            curr_in = M[pc_as_index];

            opcode = curr_in[31:26];

            $display("_________________________________________________________");
            $display("Current Instruction:\t%b; instruction number:%d", curr_in, instr_num + 1);
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

                pc = pc + 4;

            end

            // I-type
            // division by 2^n truncates n digits
            else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin

                // R[rs] contains memory address when used in I-type instructions
                rs = curr_in[25:21];
                rt = curr_in[20:16];
                immediate = curr_in[15:0];

                // (R[rs] - pc) / 4: memory address in R[rs] converted into an index in M
                // immediate / 4: offset from immediate converted to index offset in M
                mem_as_index = ((R[rs] - pc) + immediate) / 4;

                // lw rt, off(rs); assign value in mem[rs + off] to rt
                if (opcode == 6'b100011) begin
                    R[rt] = M[mem_as_index];
                end
                // sw rt, off(rs); assign value in rt to mem[rs + off]
                else if (opcode == 6'b101011) begin
                    M[mem_as_index] = R[rt];
                end

                // immediate == label for 3 conditions below
                else if (opcode == 6'h4) begin
                    if (R[rs] == R[rt]) begin
                        pc = pc + immediate * 4; //We offset the pc by the label * 4. This needs to be additionally incremented by 4 at the end
                    end
                end

                if (opcode == 6'h5 )begin
                    if (R[rs] != R[rt]) begin
                        pc = pc + immediate * 4;
                    end
                end

                if(opcode == 6'h7) begin
                    if (R[rs] > R[rt]) begin
                        pc = pc + immediate * 4;
                    end
                end

                $display("Rs:\t%b", rs);
                $display("Rt:\t%b", rt);
                $display("Immediate:\t%b", immediate);

                pc = pc + 4;

            end

            else begin
                // no incrementing of pc occurs
                if (opcode / 2 == 5'b00001) begin
                  // Jtype
                  addr = curr_in[25:0] * 4;

                  if(opcode == 6'b000010) begin
                    // jump
                    pc = addr;
                  end

                  if (opcode == 6'b000011)begin
                    // jal
                    R[31] = addr;
                  end
                end

            end

            if (pc_as_index > 100) begin
                alive = 0;
            end

            instr_num = instr_num + 1;

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
            pc_print = start_pc;
            for (print_index = 0; print_index < 100; print_index = print_index + 1) begin
                $display("0x%h: %d", pc_print, M[print_index]);
                pc_print = pc_print + 4;
            end

        end

    end

endmodule
