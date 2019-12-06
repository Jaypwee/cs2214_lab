`timescale 1ns / 1ps
// Lab 8

module main(clk);
  input clk;

  integer total_instr;
  integer instr_done;

  wire if_id; // Send instruction over


  initial begin
    total_instr = 15;
    instr_done = 0;
  end

  IF main_if(
    .pc_src(1),
    .instruction(if_id)
  );


  always @(posedge clk) begin
    instr_done = instr_done + 1;
  end


module IF(pc_src, instruction);
  input pc_src;
  output instruction;

  wire[0:0] mux_signal;
  wire[31:0] current_pc; //If this changes, it sends signal to Instruction memory
  wire[31:0] updated_pc;

  initial begin
    updated_pc = pc_src;
  end

  // Initials must be set by forcing constants

  mux mux_main(
    .mx_inp(mux_signal),
    .pc(updated_pc),
    .outp(current_pc)
  );

  adder adder_main(
    .pc(current_pc),
    .change_mux(mux_signal),
    .newpc(updated_pc)
  );

  instr_mem instr_mem_main(
    .pc_src(pc_src)
    .pc(current_pc),
    .instr(instruction)
  );

endmodule

module adder(pc, change_mux, newpc); //Adder is only updated when a new pc comes. After updating pc, it should give MUX a 0
  input pc;
  output change_mux;
  output newpc;

  assign newpc = pc + 4;
  assign change_mux = 0;
endmodule

module mux(mx_inp, pc, outp);
  input mx_inp;
  input pc;

  output reg outp;

  always @(mx_inp) begin
    if(mx_inp) outp <= pc; //Updates the PC that was added by the adder if mux signal is 1.
  end
endmodule

module instr_mem(pc_src, instr);
  input pc_src;
  output instr;

  integer init_pc;

  integer M[14:0];

  //roni: init_pc will be a big number so init_pc+x is likely larger than the size of M
  initial begin
    init_pc = pc_src //Snapshot the first pc
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

  assign instr = M[(pc - init_pc)/4];
endmodule
