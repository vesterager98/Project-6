`timescale 1ns / 1ps


module ReadTest(

    input btn1,
    input btn2,

    input clock,
    inout SDA,
    output SCL

    );
    
    //I2C module I/O
    reg send;
    wire rec;
    reg [6:0] address = 7'b0110101;
    reg [7:0] dataReg = 8'b11100111;
    reg [7:0] dataIn = 8'b00000000;
    reg ReadWrite = 1;
    wire [2:0] ReadAmount = 1;
    
    wire [7:0] dataOut;
    
    //Test logic
    reg [22:0] Counter = 0; //Counter for 12 MHz -> 2 Hz
    reg start = 0;
    
    //Module def
    I2C I2C_mod(clock, SDA, SCL, send, rec, ReadWrite, ReadAmount, address, dataReg, dataIn, dataOut);
    
    always @(posedge clock) begin
    
        //Logic that allows the FPGA to be reset
        if (btn1) start <= 1;
        if (btn2) begin
            start <= 0;
            ReadWrite <= 1;
        end
        
        //Logic that allows the FPGA to bounce back messages
        if (start) begin 
    
            if (Counter == 6000000/2) begin
                send <= 1;
            end 
            
            if (Counter == 6000000) begin
                send <= 0;
                dataIn <= dataOut;
                ReadWrite <= ~ReadWrite;
                Counter <= 0;
            end 
        
            Counter <= Counter + 1;
        
        end
    
    end
    
endmodule
