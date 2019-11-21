module execute(clk, pc, opcode, opfunc, immediate, rd1, rd2, aluop, alusrc);
  input clk, aluop, alusrc, pc;
  input rd1, rd2, control;

  output result;

  always @(clk) begin
    if (aluop) begin
      case (opfunc)
        6'b100000: result <= rd1 + rd2;
        6'b100100: result <= rd1 & rd2;
        // 6'b000110: result <= rd1 / rd2;
        6'b100010: result <= rd1 - rd2;
        6'b100101: result <= rd1 | rd2;
        6'b101010: begin
          if (rd1<rd2) result <= 1;
          else result <= 0;
        end
        // or multi 
        // There is so many fucking alu operations =_=
      endcase
    end

    else if (immediate & alusrc) begin // funcs such as addi, where we need ALU and not branching
      if (opcode == 6'b00100) begin
        result <= rd1 + immediate;
      end
    end

    else if (immediate) begin //sw lw
      case (opcode)
        6'b100011: begin //lw
          result <= rd1 + immediate // Memread control in ID
        end
        6'b101011: begin //sw
          result <= rd1 + immediate // Memwrite control in ID
          
        end
      endcase
    end
  end
endmodule