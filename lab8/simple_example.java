`timescale 1ns / 1ps
module main(clk, counter);

    input clk;

    output counter;

    wire[31:0] looper;
    wire out_wire;

    adder add(
      .left(looper),
      .right(4),
      .res(out_wire)
    );

    assign counter = out_wire;

endmodule


module adder(left, right, res);
    input left, right;
    output res;
    assign res = left + right;
endmodule

// expected: when forcing to looper, out_wire changes and then counter changes
// does not run as expected

// problem: out_wire and counter have size 1 and can't be changed (define why)
