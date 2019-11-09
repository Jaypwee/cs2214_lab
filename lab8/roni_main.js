`timescale 1ns / 1ps

module main(clk, borderIF);

    input clk;

    // ___lifecycle variables___
    integer alive = 1;
    integer instruction_count = 15; // TODO: check if correct for each lab
    integer instructions_completed = 0;

    // memory variables
    integer curr_in;
    integer M[99:0];
    //integer pc_start = 32'h00400000; // TODO: set pc_start to memory address when needed
    integer pc_start = 0;
    integer pc_as_index; // will never be assigned to except at start of always block
    integer addr;
    integer mem_as_index;



    // ___IF variables___
    output reg[31:0] borderIF;

    // logic wires
    // naming: "w_<wire source>_<one of wire destinations>"
    wire w_pc_border;  // carries pc
    wire w_add_mux;  // carries incremented pc

    // control wires
    reg pcsrc = 0;

    mux if_mux(
      .in_value(pcsrc),
      .carry_value(w_add_mux)
    );

    pc if_pc(
      .value(w_add_mux),
      .out_value(w_pc_border)
    );

    adder add_pc(
      .left(w_pc_border),
      .right(4),
      .res(w_add_mux)
    );


    // ___ID variables (incomplete)___
    reg[5:0] opcode;

    // R-type variables
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] rd;
    reg[4:0] shamt;
    reg[5:0] func;

    // I-Type variables
    reg[15:0] immediate;

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

    always @(posedge clk) begin

      if (alive) begin

        // borderIF stores the latest instruction
        borderIF <= M[(w_pc_border - pc_start) / 4];

        // change the value that causes IF to occur (start at MUX)
        pcsrc <= pcsrc + 1;

        $display("address: %d", w_add_mux);
        $display("index: %d", (w_add_mux - pc_start) / 4);
        $display("borderIF: %h", borderIF);

        // ___ID___
        // // this is the only line that pc_as_index will be modified in
        // pc_as_index = (w_add_mux - pc_start) / 4;
        //
        // borderIF <= M[pc_as_index];
        // curr_in = M[pc_as_index];
        //
        // opcode = curr_in[31:26];
        //
        // $display("_________________________________________________________");
        // $display("Current Instruction:\t%b; instruction number:%d", curr_in, instructions_completed + 1);
        // $display("Opcode:\t%b", opcode);
        //
        // // R-type
        // if (opcode == 6'h0) begin
        //
        //     rs <= curr_in[25:21];
        //     rt <= curr_in[20:16];
        //     rd <= curr_in[15:11];
        //     shamt <= curr_in[10:6];
        //     func <= curr_in[5:0];
        //
        //     $display("Rs:\t%b", rs);
        //     $display("Rt:\t%b", rt);
        //     $display("Rd:\t%b", rd);
        //     $display("Shamt:\t%b", shamt);
        //     $display("Func:\t%b", func);
        //
        // end
        //
        // // I-type
        // // division by 2^n truncates n digits
        // else if (opcode != 6'b0 & opcode / 2 != 5'b00001 & opcode / 4 != 4'b0100) begin
        //
        //     // R[rs] contains memory address when used in I-type instructions
        //     rs <= curr_in[25:21];
        //     rt <= curr_in[20:16];
        //     immediate <= curr_in[15:0];
        //
        //     // (R[rs] - w_add_mux) / 4: memory address in R[rs] converted into an index in M
        //     // immediate / 4: offset from immediate converted to index offset in M
        //     //mem_as_index = ((R[rs] - pc_start) + immediate) / 4;
        //
        //     $display("Rs:\t%b", rs);
        //     $display("Rt:\t%b", rt);
        //     $display("Immediate:\t%b", immediate);
        //
        // end
        //
        // // Jtype
        // // no incrementing of pc occurs
        // else if (opcode / 2 == 5'b00001) begin
        //
        //     addr <= curr_in[25:0] * 4;
        //     $display("address (jump immediate):\t%b", addr);
        //
        // end

        instructions_completed = instructions_completed + 1;

        if (instructions_completed >= instruction_count) begin
          alive = 0;
        end

      end

    end

endmodule


// modules were defined after they were composed in main
// doing so helps in defining them

// memory is not a module, it is an array

module mux(in_value, carry_value);
  input in_value;
  output reg carry_value;

  always @(in_value) begin
    if (!in_value) carry_value = 0;
    else carry_value = in_value;
  end

endmodule

// does nothing as of now
module pc(value, out_value);
  input value;
  output out_value;
  assign out_value = value;
endmodule


module adder(left, right, res);
  input left, right;
  output res;
  assign res = left + right;
endmodule
