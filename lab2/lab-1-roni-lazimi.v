`timescale 1ns / 1ps
module main(clk);

integer alive = 1; // !alive is HLT

input clk;

integer idx_inp = 0; // index for input
integer idx_instr = 0; // index for instructions

integer input_size = 10;
integer input_list [9:0];
integer instruction_list [9:0];

// either an array of length 100, or 2d array of area 100, arbitrarily choosing a 2d array
integer memory [9:0][9:0];

// iterate over the memory array using n as row and m as column
// registered count will be used to iterate over the input list
integer n, m;
integer i;

// flow variables
integer accum, current_input, instruction, address, code;
integer row, col;
integer feedback;

initial begin

  input_list[0] = 9;
  input_list[1] = 5;
  input_list[2] = 7;
  input_list[3] = 12;
  input_list[4] = 13;
  input_list[5] = 72;
  input_list[6] = 24;
  input_list[7] = 39;
  input_list[8] = 51;
  input_list[9] = 20;

  instruction_list[0] = 901;
  instruction_list[1] = 399;
  instruction_list[2] = 901;
  instruction_list[3] = 199;
  instruction_list[4] = 902;
  instruction_list[5] = 0;
  instruction_list[6] = 0;
  instruction_list[7] = 0;
  instruction_list[8] = 0;
  instruction_list[9] = 0;

  // title, indent, list
  $write("Initialized input list:");
  $display("\t");

  for (idx_inp = 0; idx_inp < input_size; idx_inp = idx_inp + 1) begin
    $write("%d ", input_list[idx_inp]);
  end
  $display("\t");

  // title, indent, list
  $write("Initialized Instructions:");
  $display("\t");
  for (idx_instr = 0; idx_instr < input_size; idx_instr = idx_instr + 1) begin
   $write("%d ", instruction_list[idx_instr]);
  end
  $display("\t");

  // title, indent, list
  $write("Initialized Memory: ");
  $display("\t");

  // loop through each field in memory, and set the latest input from input_list into the 2d memory
  // i is used to reference latest input

  // TODO: Move to function if possible
  for (n = 0; n < input_size; n = n + 1) begin
    for (m = 0; m < input_size; m = m + 1) begin
        memory[n][m] = 0;
        if (n == 0) begin
            memory[n][m] = instruction_list[m];
        end
        $write("%d ", memory[n][m]);
    end
    $display("\t");
  end

  // reusing for always
  idx_inp = 0;
  idx_instr = 0;

end

always @(posedge clk) begin

    if (alive) begin

      $display("________________________________________________________________________________________________________________________");

      // approach:
      //   - parse actions
      //   - perform actions
      //   - output actions
      //   - output results

      /* parse actions */

      instruction = instruction_list[idx_instr];
      code = instruction / 100;

      // if using array, address is sufficient as an index
      address = instruction % 100;
      row = address / 10;
      col = address % 10;

      current_input = input_list[idx_inp];

      /* perform actions */
      // TODO: move to case statement
      if (code == 0) begin
          // Stop
          alive = 0;
      end

      if (code == 1) begin
          // Add the contents of the memory address to the Accumulator
          accum = accum + memory[row][col];
      end

      if (code == 2) begin
          // Subtract the contents of the memory address from the Accumulator
          accum = accum - memory[row][col];
      end

      if (code == 3) begin
          // Store the value in the Accumulator in the memory address given
          memory[row][col] = accum;
      end

      if (code == 5) begin
          // Load the Accumulator with the contents of the memory address given
          accum = memory[row][col];
      end

      if (code == 6) begin
        // Set the idx_instr
        idx_instr = col
      
      if (code == 7) begin
        // If accum is 0, set idx_instr
        if (accum == 0) begin
          idx_instr = col
        end

      if (code == 8) begin
        // If accum is greater than or eq to 0,
        if (accum >= 0) begin
          idx_instr = col
        end

      if (code == 9) begin
          // Input or Output. Take from Input if address is 1, copy to Output if address is 2
          if (address == 1) begin
              accum = current_input;
              idx_inp = idx_inp + 1;
          end else if (address == 2) begin
              feedback = accum;
          end
      end

      /* output actions */
      $display("Instruction:\t%d", instruction);

      $display("Accumulator:\t%d", accum);
      $display("inp_idx:\t%d", idx_inp);
      $display("Output: %d", feedback);

      /* output results */
      // TODO: Move to function if possible
      $display("Memory:");
      for (n = 0; n < input_size; n = n + 1) begin
          for (m = 0; m < input_size; m = m + 1) begin
              $write("%d ", memory[n][m]);
          end
          $display("\t");
      end

      // always increment instruction index
      idx_instr = idx_instr + 1;

    end

end

endmodule
