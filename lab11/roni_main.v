`timescale 1ns / 1ps

module main(clk);

    input clk;

    // README: read the comments that start with __*__

    // ___lifecycle variables___
    reg alive = 1;
    integer instruction_count = 15;
    integer instructions_completed = 0;

    // ___stage wires___
    // each wire name prefix is a stage
    // value of wire comes after completion of stage suggested by prefix of wire
    wire[31:0] if_instruction;  //available after IF stage
    wire[31:0] if_new_pc;

    wire[31:0] id_carried_pc;
    wire[31:0] id_read_data_1;
    wire[31:0] id_read_data_2;
    wire[31:0] id_immediate;

    wire[31:0] exe_mem_address;
    wire[31:0] exe_branch_pc;
    wire exe_zero;

    wire[31:0] mem_data;
    wire mem_pcsrc;

    wire[31:0] wb_data;

    // ___control wires___
    wire control_handle_rtype;
    wire control_mem_write;
    wire control_mem_read;
    wire control_mem_to_reg;
    wire control_will_branch;

    wire[3:0] alu_command;        //used to decide which operation the EXE alu will perform


    // IF follows the design of the graph with borders
    // all the other stages follow the graph without borders

    stage_IF stage_if(

        // input
        .pcsrc(mem_pcsrc),
        .branch_pc(exe_branch_pc),

        // output
        .instruction(if_instruction),
        .new_pc(if_new_pc)

    );

    stage_ID stage_id(

        //input
        .instruction(if_instruction),
        .new_pc(if_new_pc),
        .handle_rtype(control_handle_rtype),
        .wb_data(wb_data),

        //output
        .new_pc_carried(id_carried_pc),
        .rd1(id_read_data_1),
        .rd2(id_read_data_2),
        .immediate(id_immediate)

    );

    control ctrl(

        .opcode(if_instruction[31:26]),       //same as alu_op

        .regDest(control_handle_rtype),       //deciding regDest in stage 2

        // lab11
        .mem_write(control_mem_write),        //unlock write data in mem module
        .mem_to_reg(control_mem_to_reg),
        .mem_read(control_mem_read),          //unlock read data in mem module
        .will_branch(control_will_branch)     //use to decide if branching; only branch if (will_branch && !exe_zero)

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
        .out_add_res(exe_branch_pc),
        .out_zero(exe_zero)

    );

    // lab11
    stage_MEM stage_mem(

        .in_first_address(0),                 //TODO: change to first address in input
        .in_address(exe_mem_address),
        .in_write_data(id_read_data_2),

        .ctrl_mem_write(control_mem_write),
        .ctrl_mem_read(control_mem_read),

        .out_read_data(mem_data)

    );

    // lab11
    mux stage_wb(
        .if_active(mem_data),
        .default_val(exe_mem_address),
        .active(control_mem_to_reg),
        .output_val(wb_data)
    );

    always @(posedge clk) begin

        if (alive) begin

            //TODO: borders here, e.g. use will_branch to decide pcsrc at border EXE/MEM

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
        .initial_pc(0),       //TODO: change
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
module stage_ID(instruction, new_pc, handle_rtype, wb_data, new_pc_carried, rd1, rd2, immediate);

    input[31:0] instruction;
    input[31:0] new_pc;
    input handle_rtype;
    input[31:0] wb_data;  //an integer

    output[31:0] new_pc_carried;
    output reg[31:0] rd1;
    output reg[31:0] rd2;
    output reg[31:0] immediate;

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

    assign new_pc_carried = new_pc;

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
    out_zero
);

    input[31:0] in_new_pc;
    input[31:0] in_rd1;
    input[31:0] in_rd2;
    input[31:0] in_sx_imm;

    input ctrl_alu_src;
    input[3:0] ctrl_alu_op;          //instruction provided by alu control

    output[31:0] out_alu_res;
    output[31:0] out_add_res;
    output reg out_zero;

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

    // if alu result is 0, signal the "zero" wire to prevent branching
    always @(out_alu_res) begin
        if (out_alu_res == 0) out_zero <= 1;
    end

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

// ___MEM modules___
// lab11
module stage_mem(in_first_address, in_address, in_write_data, ctrl_mem_write, ctrl_mem_read, out_read_data);

  input[31:0] in_address;
  input[31:0] in_first_address;
  input[31:0] in_write_data;

  input ctrl_mem_write;
  input ctrl_mem_read;

  output reg[31:0] out_read_data;

  integer M[99:0];

  always @(in_address, in_write_data) begin

      if (ctrl_mem_write) begin
          M[(in_address - in_first_address) / 4] <= in_write_data;
      end
      else if (ctrl_mem_read) begin
          out_read_data <= M[(in_address - in_first_address) / 4];
      end

  end

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
module control(opcode, regDest, mem_write, mem_to_reg, mem_read, will_branch);

    input[5:0] opcode;

    output reg mem_read, mem_write, mem_to_reg, will_branch, regDest;

    always @(opcode) begin

        if (opcode == 6'b0) begin
            regDest <= 1;
        end

        else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin

            regDest <= 0;

            // lw
            if (opcode == 6'b100011) begin
                mem_to_reg <= 1;
                mem_read <= 1;
            end

            // sw
            else if (opcode == 6'b101011) begin
                mem_write <= 1;
            end

            // beq
            else if (opcode == 6'h4) will_branch <= 1;
            // bneq

            else if (opcode == 6'h5 ) will_branch <= 1;
            // bgt
            else if (opcode == 6'h7) will_branch <= 1;

        end

    end

endmodule

module control_alu(func, opcode, alu_command, alu_branch);

    input[4:0] func;
    input[5:0] opcode;

    output reg[3:0] alu_command;
    output reg alu_branch;

    always @(func, opcode) begin

        // R-type
        if (opcode == 6'h0) begin

            if (func == 6'h20) begin
                alu_command <= 1; //+
            end

            else if (func == 6'h22) begin
                alu_command <= 2; //-
            end

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

            // (lab11) the new branch pc is calculated earlier in EXE stage
            // (lab11) alu_branch is the same as pcsrc
            // beq
            else if (opcode == 6'h4) alu_branch <= 1;

            // bneq
            else if (opcode == 6'h5 ) alu_branch <= 1;

            // bgt
            else if (opcode == 6'h7) alu_branch <= 1;

            //*i: nothing happens to the immediate
            else begin
              alu_command <= 0;
              alu_branch <= 0;
            end

        end

    end

endmodule
