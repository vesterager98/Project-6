`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2023 01:53:11 PM
// Design Name: 
// Module Name: TestImpl
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


module TestImpl(

    input btn1,
    input btn2,
    output led1,
    output led2,
    input clock,
    inout SDA,
    output SCL

    );
    
    //I2C I/O
    wire send;
    reg [6:0] address = 7'b0110101;
    reg [7:0] datareg = 8'b01100110;
    reg [7:0] data = 8'b01000001;
    wire readwrite;
    
    wire [7:0] 
    dataRecReg1, 
    dataRecReg2, 
    dataRecReg3, 
    dataRecReg4,
    dataRecReg5,
    dataRecReg6,
    dataRecReg7,
    dataRecReg8,
    dataRecReg9,
    dataRecReg10,
    dataRecReg11,
    dataRecReg12;
    
    //Define module
    I2C I2C_mod(SDA, SCL, clock, send, address, datareg, readwrite, dataRecReg1, 
    dataRecReg1, 
    dataRecReg2, 
    dataRecReg3, 
    dataRecReg4,
    dataRecReg5,
    dataRecReg6,
    dataRecReg7,
    dataRecReg8,
    dataRecReg9,
    dataRecReg10,
    dataRecReg11,
    dataRecReg12);
    
    //Logic
    assign led1 = btn1;
    assign send = btn1;
    assign led2 = btn2;
    assign readwrite = btn2;
    
    //Read test //------------------------------------------------------------------
    /*
    
    always @(posedge clock) begin
        
        data <= dataRecReg1;
        
    end
    
    /*
    //Read test //-------------------------------------------------------------------
    
    
    //Write test //------------------------------------------------------------------------------------
    /*
    
    reg [22:0] Counter = 0;
    
    always @(posedge clock) begin
    
        if (Counter == 6000000/2) begin
        
            send <= 1;
        
        end 
        
        if (Counter == 6000000) begin
        
            send <= 0;
            data <= data + 1;
            Counter <= 0;
        
        end 
        
        Counter <= Counter + 1;
    
    end
    
    */
    //Write test //-------------------------------------------------------------------------------------
    
    
endmodule
