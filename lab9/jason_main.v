`timescale 1ns / 1ps
// Lab 8

module main(clk);
  input clk;

  integer total_instr;
  integer instr_done;
  integer init_pc;
  integer alive;

  wire if_id; // Send instruction over

  initial begin
    init_pc = 0;
    alive = 1;
    total_instr = 15;
    instr_done = 0;
    
    $display("Start initialize");
  end

  IF main_if(
    .pc_src(1),
    .instruction(if_id)
  );


  always @(posedge clk) begin
  
    $display("CLK TICK");
    instr_done = instr_done + 1;
    if(instr_done > total_instr) alive = 0;
  end
  
endmodule



module IF(init_pc, clk, init_pc, pc_src, instruction);
  input pc_src, init_pc;
  input clk;
  output instruction;

  wire[0:0] mux_signal;
  wire[31:0] current_pc; //If this changes, it sends signal to Instruction memory
  wire[31:0] updated_pc;

  initial begin
    
  end

  // Initials must be set by forcing constants

  mux mux_main( //if pc_src is 0 output pc_0 else pc_1
    .mx_inp(pc_src),
    .pc_0(updated_pc),
    .pc_1(branched_pc), //This will come from later place
    .outp(current_pc)
  );

  adder adder_main(
    .pc(current_pc),
    .newpc(updated_pc)
  );

  instr_mem instr_mem_main(
    .init_pc(init_pc),
    .pc(current_pc),
    .instr(instruction)
  );

endmodule

module adder(pc, newpc); //Adder is only updated when a new pc comes. After updating pc, it should give MUX a 0
  input pc;
  output newpc;

  assign pc = pc + 4;
endmodule

module mux(mx_inp, pc_0, pc_1, outp);
  input mx_inp; //Selector
  input pc_0, pc_1;

  output reg outp;

  always @(mx_inp) begin
    if(mx_inp) outp <= pc_0; //Updates the PC that was added by the adder if mux signal is 1.
    else outp <= pc_1;
  end
endmodule

module instr_mem(pc, init_pc, instr);
  input pc;
  input init_pc;
  output instr;

  integer M[14:0];

  //roni: init_pc will be a big number so init_pc+x is likely larger than the size of M
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

  assign instr = M[(pc - init_pc)/4];
endmodule

module ID(instruction, pc, wb_data, pc_carry, rd1, rd2, immediate, rs, rt);
  input[31:0] instruction;
  input pc;

  output reg pc_carry, rs, rt;
  output wb_data, rd1, rd2, immediate;
  wire rd;
  wire write_reg;
  wire r; //rtype ? 1 : 0
  wire i; //itype ? 1 : 0
  wire j; //jtype ? 1 : 0

  register ID_reg(
    .read1(instruction[25:21]),
    .read2(instruction[20:16]),  //unused if I-type
    .reg_write(write_reg),
    .data_write(wb_data),  //i.e. data to write
    .readMem1(rd1),
    .readMem2(rd2)
  );

  control ID_control(
    .opcode(instruction[31:26]),
    .r(r),
    .i(i),
    .j(j)
  );

  lse util(
    .inp(instruction[15:0]),
    .len(32),
    .outp(immediate)
  );

  mux is_r( 
    .mx_inp(r),
    .pc_0(instruction[15:11]),
    .pc_1(instruction[20:16]),
    .outp(write_reg)
  );

  always @(instruction, pc) begin
      rs <= instruction[25:20];
      rt <= instruction[20:16];
      pc_carry <= pc;
  end

endmodule

module register(read1, read2, address, reg_write, data_write, readMem1, readMem2);
  input[5:0] read1;
  input[5:0] read2;
  input address, reg_write, data_write;

  output reg readMem1;
  output reg readMem2;

  integer R[31:0];

  always @(*) begin
    readMem1 <= R[read1];
    readMem2 <= R[read2];

    if (data_write) R[(reg_write - address)/4] <= data_write;
  end
endmodule

module lse(inp, len, outp);
  input inp, len;
  output outp;

  assign outp = inp | 32'h0; //Basically make it 16 -> 32 bit
endmodule

module control(opcode, r, i, j);
  input opcode;
  output reg r, i, j;

  always @(opcode) begin
    //Reset indicators
    r = 0;
    i = 0;
    j = 0;
    // All the control conditionals need to be here

    if (opcode == 6'b0) r <= 1;

        // I type
        else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin
            i <= 0;
        end

        else if(opcode / 2 == 5'b00001) j <= 1;
  end
endmodule