`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/09/10 04:23:44
// Design Name: 
// Module Name: main
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// Sung Wook Park
//////////////////////////////////////////////////////////////////////////////////


module main(clk);
input clk;
integer input_list [9:0];
integer instruction_list [9:0];
integer memory_list [99:0];
integer inp_idx;
integer accumulator;
integer out_put;
integer counter;
integer i;

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
instruction_list[3] =  199;
instruction_list[4] = 902;
instruction_list[5] = 0;
instruction_list[6] = 0;
instruction_list[7] = 0;
instruction_list[8] = 0;
instruction_list[9] = 0;

for(i=0;i<100;i=i+1)begin
    memory_list[i] = 0;
end

for(i=0;i<10;i=i+1)begin
    memory_list[i] = instruction_list[i];
end

inp_idx = 0;
counter = 0;
accumulator = 0;
$write("Initialized Input List: ");
for(i=0;i<10;i=i+1) begin
    $write("%d ", input_list[i]);
end
$display(" ");
$write("Initialized Instructions: ");
for(i=0;i<10;i=i+1) begin
    $write("%d ", instruction_list[i]);
end
$display(" ");
$write("Initialized memory: ");
for(i=0;i<100;i=i+1) begin
        $write("%d ", memory_list[i]);
end
$display(" ");

end

endmodule
