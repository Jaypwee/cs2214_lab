`timescale 1ns / 1ps

module main(clk);

    input clk;

    // ___lifecycle variables___
    reg alive = 1;
    integer instruction_count = 15;
    integer instructions_completed = 0;

    wire borderIF;

    stage_IF stageif(
        .pcsrc(1),
        .clk_if(clk),
        .instruction(borderIF)
    );

    always @(posedge clk) begin

        if (alive) begin

          $display("borderIF: %d", borderIF);

          instructions_completed = instructions_completed + 1;
          if (instructions_completed >= instruction_count) alive = 0;

        end

    end

endmodule


module stage_IF(pcsrc, clk_if, instruction);

    input clk_if;
    input pcsrc;
    output instruction;

    wire source_pc;   //pc depends on this depends on this
    wire current_pc;  //instr_mem,add depend on this

    mux mux_if(
        .in_value(pcsrc),
        .carry_value(source_pc) //every module that depends on source_pc changes, if mux active
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

endmodule


module mux(in_value, carry_value);
    input in_value;
    output reg carry_value;

    // protocol: any module that depends on a mux will have not update if it receives 0 from mux
    always @(in_value) begin
        if (!in_value) carry_value <= 0;
        else carry_value <= in_value;
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
