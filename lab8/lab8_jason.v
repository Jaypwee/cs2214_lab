// Lab 8

module main(clk, init_pc);
  input clk;
  input init_pc;

  wire[0:0] mux_signal;
  wire current_pc; //If this changes, it sends signal to Instruction memory
  wire updated_pc;
  wire instruction;

  initial begin
    mux_signal = 0;
    current_pc = init_pc;
  end

  mux mux_main(
    .mx_inp(mux_signal),
    .pc(updated_pc)
    .outp(current_pc)
  );

  adder adder_main(
    .pc(current_pc),
    .change_mux(mux_signal),
    .newpc(updated_pc)
  );

  instr_mem instr_mem_main(
    .pc(current_pc),
    .instr(instruction)
  );

endmodule

module adder(pc, change_mux, newpc);
  input pc;
  input change_mux;
  output newpc;

  assign newpc = pc + 4;
endmodule

module mux(mx_inp, pc, outp);
  input mx_inp;
  input pc;

  output reg outp;

  always @(mx_inp) begin
    if(mx_inp) outp <= pc //Updates the PC that was added by the adder if mux signal is 1.
  end
endmodule

module instr_mem(init_pc, pc, instr);
  input init_pc;
  input pc;
  output instr;

  integer M[14:0];

  initial begin
    M[init_pc+0] = { 6'b000000, 5'b01000, 5'b01000, 5'b01001, 5'b00000, 6'b100000 };
    M[init_pc+4] = { 6'b000000, 5'b01000, 5'b01001, 5'b01010, 5'b00000, 6'b100010 };
    M[init_pc+8] = { 6'b000000, 5'b01010, 5'b01000, 5'b01010, 5'b00000, 6'b100100 };
    M[init_pc+12] = { 6'b000000, 5'b01001, 5'b01010, 5'b01011, 5'b00000, 6'b100101 };
    M[init_pc+16] = { 6'b000000, 5'b01011, 5'b01000, 5'b01100, 5'b00000, 6'b101010 };
    M[init_pc+20] = { 6'b100011, 5'b10000, 5'b01011, 5'b00000, 5'b00000, 6'b000100 };
    M[init_pc+24] = { 6'b100011, 5'b10000, 5'b01010, 5'b00000, 5'b00000, 6'b001000 };
    M[init_pc+28] = { 6'b100011, 5'b10000, 5'b10001, 5'b00000, 5'b00000, 6'b000000 };
    M[init_pc+32] = { 6'b100011, 5'b11110, 5'b10110, 5'b00000, 5'b00000, 6'b000000 };
    M[init_pc+36] = { 6'b100011, 5'b10000, 5'b11001, 5'b00000, 5'b00000, 6'b001100 };
    M[init_pc+40] = { 6'b101011, 5'b00101, 5'b11101, 5'b00000, 5'b00000, 6'b000100 };
    M[init_pc+44] = { 6'b101011, 5'b11000, 5'b00100, 5'b00000, 5'b00000, 6'b000000 };
    M[init_pc+48] = { 6'b101011, 5'b00011, 5'b11110, 5'b00000, 5'b00000, 6'b000100 };
    M[init_pc+52] = { 6'b101011, 5'b10000, 5'b10000, 5'b00000, 5'b00000, 6'b001000 };
    M[init_pc+56] = { 6'b101011, 5'b00100, 5'b00010, 5'b00000, 5'b00000, 6'b001100 };
  end

  assign instr = M[pc];
endmodule