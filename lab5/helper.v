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

