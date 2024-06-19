`timescale 1ns / 1ps



module Multiplier_Testbench();


//Module I/O 
reg Clock;
reg [15:0] num1;
reg [15:0] num2;
wire [15:0] res;

//Test unit
FloatMultiplier Float_mult(Clock, num1, num2, res);

initial begin

    //Register initialization
    Clock = 0;

    num1 =16'b1011000000011000;
    num2 =16'b0010000000000100;


    forever begin
    
        #1 Clock = ~Clock;
        
    end
end



endmodule
