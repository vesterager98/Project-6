`timescale 1ns / 1ps



module Controller(
    
    // Basic I/O
    input clock,
    output [7:0] out,
    input [15:0] in,
    
    //Controller multipliers
    input [15:0] K,
    input [15:0] KD,
    input [15:0] KI
    
    );



//Other
reg [7:0] ClockCount = 0;
reg Out;



    
always @(posedge clock)
begin
    
    if (ClockCount == 255) begin
    
        
        
        
        
    
    
        ClockCount = 0;
        
    end else
        
        ClockCount <= ClockCount + 1;
end 

//Assign output
assign out = Out;
    
endmodule
