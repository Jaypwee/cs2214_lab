`timescale 1ns / 1ps
module main(clk, counter);

    input clk;

    output[31:0] counter;

    wire[31:0] looper;

    adder add(
      .left(looper),
      .right(4),
      .res(counter)
    );

    always @(posedge clk) begin
      $display("counter: %d", counter);
    end

endmodule


module adder(left, right, res);
    input left, right;
    output reg res;
    always @(left, right) begin
      $display("add; l: %d, r: %d", left, right);
      res <= left + right;
    end
endmodule
