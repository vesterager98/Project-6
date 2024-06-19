`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/11/2023 02:47:55 PM
// Design Name: 
// Module Name: Top
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


module Top(

    //General use signals
    input Clock,
    
    //PWM signals
    //input [7:0] DutySelect,
    //output PWM_Gen
    
    //I2C signals
    inout SDA,
    output SCL,
    input Send,
    input [6:0] Address,
    input [7:0] DataReg,
    input [7:0] Data,
    input RW

    );
    
    //Define PWM generator
    //PWM PWM_Module(PWM_Gen, Clock, DutySelect);
    
    //Define I2C module
    I2C I2C_Module(SDA, SCL, Clock, Send, Address, DataReg, Data, RW);
    
    
    
endmodule
