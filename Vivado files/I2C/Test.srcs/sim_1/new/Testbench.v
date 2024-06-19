`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/11/2023 02:43:28 PM
// Design Name: 
// Module Name: Testbench
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


module PWM_Testbench;

//Inputs
reg Clock;
reg [15:0] Duty;

//outputs
wire out;

//Test unit
PWM PWM_mod(out, Clock, Duty);

initial begin

    Duty = 15'b000000010000000;
    Clock = 1;

    forever begin
    
       #1 Clock = ~Clock;
    
    end
end

endmodule
