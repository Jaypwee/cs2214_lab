if (opcode / 2 == 5'b00001) begin
  #Jtype
  addr = curr_in[25:0]
  
  if(opcode == 6'b000010) begin
    #jump
    idx_instr = addr - start
  end 

  if (opcode == 6'b000011)begin
    #jal
    R[31] = idx_instr
    idx_instr = addr - start
  end
end

// Beq

if (opcode == 6'h4)begin
  if (rs == rt) pc = pc + label * 4 //We offset the pc by the label * 4. This needs to be additionally incremented by 4 at the end
end

if (opcode == 6'h5)begin
  if (rs != rt) pc = pc + label * 4
end

if(opcode == 6'h7)begin
  if (rs > rt) pc = pc + label * 4
end