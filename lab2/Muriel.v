`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/09/2019 01:53:19 PM
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
// 
//////////////////////////////////////////////////////////////////////////////////


module main(clk);
    input clk;
    integer accum, inp, out, i;
    integer memory[99:0];
    integer inputList[9:0];
    integer instruction[9:0];
    
    initial begin
    
    $display("--- Beginning of Program ---");
    
    inputList[0] = 9;
    inputList[1] = 5;
    inputList[2] = 7;
    inputList[3] = 12;
    inputList[4] = 13;
    inputList[5] = 72;
    inputList[6] = 24;
    inputList[7] = 39;
    inputList[8] = 39;
    inputList[9] = 51;
    
    instruction[0] = 901;
    instruction[1] = 399;
    instruction[2] = 901;
    instruction[3] = 199;
    instruction[4] = 902;
    for (i=5; i<10; i=i+1)begin
        instruction[i] = 0;
    end
    
    memory[0] = 901;
    memory[1] = 399;
    memory[2] = 901;
    memory[3] = 199;
    memory[4] = 902;
    for (i=5; i<100; i=i+1)begin
        memory[i] = 0;
    end
    
    $write("Intialized Input List: ");
    for(i=0; i<10; i=i+1) begin
        $write("%d          ", inputList[i]);
    end
    $display();
    
    $write("Initialized Instructions: ");
    for(i=0; i<10; i=i+1) begin
        $write("%d          ", instruction[i]);
    end
    $display();
    
    $write("Initialized Memory: ");
    for(i=0; i<100; i=i+1) begin
        $write("%d          ", memory[i]);
    end
    $display();

    end

endmodule
