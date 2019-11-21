`timescale 1ns / 1ps

module main(clk);

    input clk;

    // ___lifecycle variables___
    reg alive = 1;
    integer instruction_count = 15;
    integer instructions_completed = 0;

    // ___stage wires___
    // each wire name prefix is a stage
    // value of wire comes after completion of stage suggested by prefix of wire
    // e.g. if_new_pc available after instruction fetch
    wire[31:0] if_instruction;
    wire[31:0] if_new_pc;

    wire[31:0] id_carried_pc;
    wire[31:0] id_read_data_1;
    wire[31:0] id_read_data_2;
    wire[31:0] id_immediate;
    wire[4:0] id_rs;
    wire[4:0] id_rt;
    wire mem_write_back;

    wire[3:0] alu_command;        //arbitrary size

    wire[31:0] exe_mem_address;
    wire[31:0] exe_branch_pc;

    // ___control wires___
    wire control_handle_rtype;

    stage_IF stage_if(
        .pcsrc(1),
        .instruction(if_instruction),
        .branch_pc(exe_branch_pc),
        .new_pc(if_new_pc)
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

    control ctrl(
        .opcode(if_instruction[31:26]),       //same as alu_op
        .regDest(control_handle_rtype)       //deciding regDest in stage 2
    );

    control_alu ctr_alu(
        .func(if_instruction[5:0]),
        .opcode(if_instruction[31:26]),
        .alu_command(alu_command)
    );

    stage_EXE stage_exe(

        .in_new_pc(id_carried_pc),
        .in_rd1(id_read_data_1),
        .in_rd2(id_read_data_2),
        .in_sx_imm(id_immediate),

        .ctrl_alu_src(!control_handle_rtype),
        .ctrl_alu_op(if_instruction[31:26]),

        .out_alu_res(exe_mem_address),
        .out_add_res(exe_branch_pc)
        //id_read_data_2 is carried through EXE

    );

    always @(posedge clk) begin

        if (alive) begin

          instructions_completed <= instructions_completed + 1;
          if (instructions_completed >= instruction_count) alive = 0;

        end

    end

endmodule

// ___IF module___
module stage_IF(pcsrc, branch_pc, instruction, new_pc);

    input pcsrc;
    input[31:0] branch_pc;   //will be supplied by branch_pc at one point
    output[31:0] instruction;
    output[31:0] new_pc;

    wire[31:0] current_pc;

    mux mux_branch(
        .if_active(branch_pc),
        .default_val(new_pc),
        .active(pcsrc),
        .output_val(current_pc)
    );

    pc main_pc(
        .in_pc(current_pc),
        .out_pc(current_pc)
    );

    adder add(
        .left(current_pc),
        .right(4),
        .res(new_pc)
    );

    instr_mem imem(
        .initial_pc(0),       //TODO: change to be dynamic
        .pc(current_pc),
        .instruction(instruction)
    );

endmodule

module instr_mem(initial_pc, pc, instruction);

    input[31:0] initial_pc;
    input[31:0] pc;
    output[31:0] instruction;

    integer M[14:0];

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
    input[31:0] new_pc;
    input handle_rtype;
    input[31:0] wb_data;  //an integer

    output reg[31:0] new_pc_carried;
    output reg[31:0] rd1;
    output reg[31:0] rd2;
    output reg[31:0] immediate;

    // rs,rt are forwarded
    output reg[4:0] rs;
    output reg[4:0] rt;
    wire[4:0] rd;

    wire[4:0] write_reg;  //takes rd if R-type, rt if I-type

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


// ___EXE modules___
module stage_EXE(
    in_new_pc,
    in_rd1,
    in_rd2,
    in_sx_imm,
    ctrl_alu_src,
    ctrl_alu_op,
    out_alu_res,
    out_add_res,
);

    input[31:0] in_new_pc;
    input[31:0] in_rd1;
    input[31:0] in_rd2;
    input[31:0] in_sx_imm;

    input ctrl_alu_src;
    input[3:0] ctrl_alu_op;          //instruction provided by alu control

    output[31:0] out_alu_res;
    output[31:0] out_add_res;

    wire[31:0] alu_oprnd_2;

    adder new_pc_adder(
        .left(in_new_pc),
        .right(in_sx_imm / 4),    //same as shift left 2
        .res(out_add_res)
    );

    mux alu_operand(
        .if_active(in_sx_imm),     //if i-type, use immediate
        .default_val(in_rd2),          //if not i-type, use value from R[readData2]
        .active(ctrl_alu_src),
        .output_val(alu_oprnd_2)
    );

    //note: reg dest was handled in stage 2, otherwise it would be a mux in this module

    alu exe_alu(
        .left(in_rd1),
        .right(alu_oprnd_2),
        .op_to_perform(ctrl_alu_op),  //the graph shows alu control as a separate module but it can be made a wire
        .res(out_alu_res)
    );

endmodule


// 2 options:
//  create a wire for each operation  (correct)
//  use a short protocol where operation is mapped to a number <- (using this to avoid maintaining wires)
module alu(left, right, op_to_perform, res);

    input[31:0] left;     //operand 1
    input[31:0] right;    //operand 2

    input[3:0] op_to_perform;    //defines what to do with operands, arbitrarily 2^4 options

    output reg[31:0] res;

    always @(left, right) begin

        case(op_to_perform)
            1: begin
                //add
                res <= left + right;
            end
            2: begin
                //sub
                res <= left - right;
            end
            3: begin
                //mult
                res <= left * right;
            end
            4: begin
                //div
                res <= left / right;
            end
        endcase

    end

endmodule


// ___common modules___
module mux(if_active, default_val, active, output_val);

    // at most 32 bit values
    input[31:0] if_active;
    input[31:0] default_val;
    input active;
    output reg[31:0] output_val;

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
    input[31:0] in_pc;
    output[31:0] out_pc;
    assign out_pc = in_pc;
endmodule


module adder(left, right, res);
    // does not handle results > 2^32
    input[31:0] left;
    input[31:0] right;
    output[31:0] res;
    assign res = left + right;
endmodule


module register(readReg1, readReg2, first_address, writeReg, writeData, readData1, readData2);

    input readReg1;
    input[4:0] readReg2;
    input[31:0] first_address;
    input[4:0] writeReg;
    input[31:0] writeData;

    output reg[31:0] readData1;
    output reg[31:0] readData2;

    // no values until
    integer R[99:0];

    always @(readReg1, readReg2, writeReg, writeData) begin

        readData1 <= R[readReg1];
        readData2 <= R[readReg2];

        if (writeData) R[(writeReg - first_address) / 4] <= writeData;

    end

endmodule


module left_sign_ext(in_bits, to_length, out_bits);

    input[31:0] in_bits;
    input[5:0] to_length; //most amount of bits to extend to is (2^6)-1, if (2^5)-1 then no 32
    output[31:0] out_bits;

    assign out_bits = in_bits | 32'h00; //TODO: change 32 to variable

endmodule


// will be more outputs when necessary
module control(opcode, regDest);

    input[5:0] opcode;
    output reg[4:0] regDest;

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

module control_alu(func, opcode, alu_command);

    input[4:0] func;
    input[5:0] opcode;

    output reg[3:0] alu_command;
    //output alu_branch;

    always @(func, opcode) begin

        // R-type
        if (opcode == 6'h0) begin

            if (func == 6'h20) begin
                alu_command <= 1; //+
            end

            else if (func == 6'h22) begin
                alu_command <= 2; //-
            end

            //TODO: codes for mult div
            else if (func == 6'h18) begin
                alu_command <= 3; //*
            end

            else if (func == 6'h1A) begin
                alu_command <= 4; //div
            end

        end

        // I-type
        else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin

            //I-types only ever add or do nothing

            // load word: *add* immediate to R[rr2]
            if (opcode == 6'b100011) alu_command <= 1;

            // store word: same as load word
            else if (opcode == 6'b101011) alu_command <= 1;

            //*i: nothing happens to the immediate
            else alu_command <= 0;

            // for now, no branching but the branch decision will be contributed to by this module
            // TODO: change to variable
            //alu_branch <= 0;

        end

    end

endmodule
