`timescale 1ns / 1ps


module FloatMultiplier(

    input clock,
    input [15:0] num1,
    input [15:0] num2,
    output [15:0] res

    );
    
reg [7:0] ClockCount = 0;
reg [15:0] Out; // 0 00000 0000000000 <- sign, exponent, significant

always @(posedge clock)
begin
    
    if (ClockCount == 255) begin
        
        //Set the sign via XOR
        Out[15] <= num1[15] ^ num2[15];
        
        //Add the exponents
        Out[14:10] <= num1[14:10] + num2[14:10];
        
        //Multiply the significand
        Out[9:0] <= num1[9:0] * num2[9:0];
        
        
        ClockCount = 0;
        
    end else
        
        ClockCount <= ClockCount + 1;

end    

//Assign output
assign res = Out;
    
endmodule