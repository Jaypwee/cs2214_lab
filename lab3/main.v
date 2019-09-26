/*
 * Do not change Module name
*/
module main;

    input clk;

    integer alive = 1;

    integer instructions[4:0];
    integer idx_instr;
    integer curr_in;

    integer R[31:0];
    integer idx_reg;

    reg[5:0] opcode;
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] rd;
    reg[4:0] shamt;
    reg[5:0] func;

    // used for always mimic
    integer j;

    initial begin

        instructions[0] = { 6'b0, 5'b01000, 5'b01000, 5'b01001, 5'b0, 6'h20 };
        instructions[1] = { 6'b0, 5'b01000, 5'b01001, 5'b01010, 5'b0, 6'h22 };
        instructions[2] = { 6'b0, 5'b01010, 5'b01000, 5'b01010, 5'b0, 6'h24 };
        instructions[3] = { 6'b0, 5'b01001, 5'b01010, 5'b01011, 5'b0, 6'h25 };
        instructions[4] = { 6'b0, 5'b01011, 5'b01000, 5'b01100, 5'b0, 6'h2a };

        for (idx_instr = 0; idx_instr < 5; idx_instr++) begin
            $display("binary: %b, hex: %h", instructions[idx_instr], instructions[idx_instr]);
        end
        idx_instr = 0;

        for (idx_reg = 0; idx_reg < 32; idx_reg++) begin
            R[idx_reg] = 0;
        end

        R[8] = 4;
        R[9] = 14;
        R[10] = 12;
        R[11] = 9;
        R[12] = 7;
        R[13] = 5;
        R[14] = 5;
        R[15] = 12;

        $write("$zero:\t%d", R[0]);
        $display();

        $write("at:\t%d", R[1]);
        $display();

        $write("$v0-v1: ");
        for (idx_reg = 2; idx_reg < 4; idx_reg++) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$a0-$a3: ");
        for (idx_reg = 4; idx_reg < 8; idx_reg++) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$t0-$t7: ");
        for (idx_reg = 8; idx_reg < 16; idx_reg++) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$s0-s7: ");
        for (idx_reg = 16; idx_reg < 24; idx_reg++) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$t8-$t9: ");
        for (idx_reg = 24; idx_reg < 26; idx_reg++) begin
            $write("%d", R[idx_reg]);
        end
        $display();

        $write("$k0-$k1: ");
        for (idx_reg = 26; idx_reg < 28; idx_reg++) begin
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
            rs = curr_in[25:21];
            rt = curr_in[20:16];
            rd = curr_in[15:11];
            shamt = curr_in[10:6];
            func = curr_in[5:0];

            if (opcode == 6'h0) begin

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

            end

            idx_instr = idx_instr + 1;

            if (idx_instr > 4) begin
                alive = 0;
            end

            $display("_________________________________________________________");

            $display("Current Instruction:\t%b", curr_in);
            $display("Opcode:\t%b", opcode);
            $display("Rs:\t%b", rs);
            $display("Rt:\t%b", rt);
            $display("Rd:\t%b", rd);
            $display("Shamt:\t%b", shamt);
            $display("Func:\t%b", func);

            $write("$zero:\t%d", R[0]);
            $display();

            $write("at:\t%d", R[1]);
            $display();

            $write("$v0-v1: ");
            for (idx_reg = 2; idx_reg < 4; idx_reg++) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$a0-$a3: ");
            for (idx_reg = 4; idx_reg < 8; idx_reg++) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$t0-$t7: ");
            for (idx_reg = 8; idx_reg < 16; idx_reg++) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$s0-s7: ");
            for (idx_reg = 16; idx_reg < 24; idx_reg++) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$t8-$t9: ");
            for (idx_reg = 24; idx_reg < 26; idx_reg++) begin
                $write("%d", R[idx_reg]);
            end
            $display();

            $write("$k0-$k1: ");
            for (idx_reg = 26; idx_reg < 28; idx_reg++) begin
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

        end

    end

endmodule
