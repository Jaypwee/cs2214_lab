`timescale 1ns / 1ps

module main(clk);

    input clk;

    // ___lifecycle variables___
    reg alive = 1;
    integer instruction_count = 15;
    integer instructions_completed = 0;

    // ___stage wires___
    // the prefix of each wire explains *after* which stage the wire will have a value
    wire[31:0] if_instruction;
    wire if_new_pc;

    wire id_carried_pc;
    wire id_read_data_1;
    wire id_read_data_2;
    wire id_immediate;
    wire id_rs;
    wire id_rt;
    wire mem_write_back;

    wire mem_branch_pc;

    // ___control wires___
    wire control_handle_rtype;

    stage_IF stage_if(
        .pcsrc(1),
        .instruction(if_instruction),
        .branch_pc(mem_branch_pc),
        .new_pc(if_new_pc)
    );

    control ctrl(
        .opcode(if_instruction[31:26]),
        .regDest(control_handle_rtype)
    );

    stage_ID stage_id(

        //inputs
        .instruction(if_instruction),
        .new_pc(if_new_pc),
        .handle_rtype(control_handle_rtype),
        .wb_data(mem_write_back),

        //outputs
        .new_pc_carried(id_carried_pc),
        .rd1(id_read_data_1),
        .rd2(id_read_data_2),
        .immediate(id_immediate),
        .rs(id_rs),
        .rt(id_rt)

    );

    always @(posedge clk) begin

        if (alive) begin

          // printing read data 2 arbitrarily since it's value depends on the most modules
          $display("ID rd2: %d", id_read_data_2);

          instructions_completed <= instructions_completed + 1;
          if (instructions_completed >= instruction_count) alive = 0;

        end

    end

endmodule

// ___IF module___
module stage_IF(pcsrc, branch_pc, instruction, new_pc);

    input pcsrc;
    output instruction, new_pc;

    wire source_pc;   //pc depends on this depends on this
    wire current_pc;  //instr_mem,add depend on this

    mux mux_if(
        .if_active(branch_pc), //every module that depends on source_pc changes, if mux active
        .default_val(source_pc),
        .active(pcsrc),
        .output_val(current_pc)
    );

    pc main_pc(
        .in_pc(source_pc),
        .out_pc(current_pc)      //every module that depends on current_pc changes
    );

    adder add(
        .left(current_pc),
        .right(4),
        .res(source_pc)         //every module that depends on source_pc changes
    );

    instr_mem imem(
        .initial_pc(0), //TODO: change to be dynamic
        .pc(current_pc),
        .instruction(instruction)
    );

    // redundant
    assign new_pc = source_pc;

endmodule

module instr_mem(initial_pc, pc, instruction);

    input initial_pc;
    input pc;
    output instruction;

    integer M[31:0];

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
    end

    assign instruction = M[(pc - initial_pc) / 4];

endmodule


// ___ID module___
// @new_pc comes from add module of IF module
module stageID(instruction, new_pc, handle_rtype, wb_data, new_pc_carried, rd1, rd2, immediate, rs, rt);

    input[31:0] instruction;
    input new_pc, handle_rtype, wb_data;
    output reg new_pc_carried, rd1, rd2, immediate;

    // rs,rt are forwarded
    output reg rs;
    output reg rt;
    wire rd;

    wire write_reg;

    //        |25:21,20:15|
    //R-Type: |rs,rt|,rd; rd is the write register
    //I-type: |rs,rt|, imm; rt is the write register


    register rgstr(
        .readReg1(instruction[25:21]),
        .readReg2(instruction[20:16]),  //unused if I-type
        .writeReg(write_reg),
        .writeData(wb_data),  //i.e. data to write
        .readData1(rd1),
        .readData2(rd2)
    );

    left_sign_ext sx(
        .in_bits(instruction[15:0]),
        .to_length(32),
        .out_bits(immediate)
    );

    mux is_rtype(
        .if_active(instruction[20:16]), //i.e. if R-Type, write_reg=rd=20:16
        .default_val(instruction[15:11]),   //i.e. if I-type, write_reg=rt=15:11
        .active(handle_rtype),
        .output_val(write_reg)
    );

    always @(instruction, new_pc) begin
        //assign to outputs that aren't outputs in any of the modules above
        rs <= instruction[25:20];
        rt <= instruction[20:16];
        new_pc_carried <= new_pc;
    end

endmodule


// ___common modules___
module mux(if_active, default_val, active, output_val);
    input if_active, active, default_val;
    output reg output_val;

    always @(if_active, default_val, active) begin
        if (active) begin
            output_val <= if_active;
        end
        else begin
            output_val <= default_val;
        end
    end

endmodule


// no computation
// only send a signal to all modules that rely on output
module pc(in_pc, out_pc);
    input in_pc;
    output out_pc;
    assign out_pc = in_pc;
endmodule


module adder(left, right, res);
    input left, right;
    output res;
    assign res = left + right;
endmodule


module register(readReg1, readReg2, first_address, writeReg, writeData, readData1, readData2);

    input readReg1, readReg2, first_address, writeReg, writeData;
    output reg readData1, readData2;

    // no values until
    integer R[99:0];

    always @(readReg1, readReg2, writeReg, writeData) begin

        readData1 <= R[readReg1];
        readData2 <= R[readReg2];

        if (writeData) R[(writeReg - first_address) / 4] <= writeData;

    end

endmodule


module left_sign_ext(in_bits, to_length, out_bits);

    input in_bits, to_length;
    output out_bits;

    assign out_bits = in_bits | 32'h00; //TODO: change 32 to variable

endmodule


// will be more outputs when necessary
module control(opcode, regDest);

    input opcode;
    output reg regDest;

    always @(opcode) begin

        if (opcode == 6'b0) begin
            regDest <= 1;
        end

        // I-type; division by 2^n truncates n digits
        else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin
            regDest <= 0;
        end

    end

endmodule
