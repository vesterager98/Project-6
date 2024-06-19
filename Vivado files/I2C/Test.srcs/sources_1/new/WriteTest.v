`timescale 1ns / 1ps


module WriteTest(

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
    reg [7:0] dataReg = 8'b11111111;
    reg [7:0] data = 8'b00000000;
    wire ReadWrite = 0;
    wire [2:0] ReadAmount = 1;
    
    //Module def
    I2C I2C_mod(clock, SDA, SCL, send, rec, ReadWrite, ReadAmount, address, dataReg, data);
    
    //Test logic
    reg [22:0] Counter = 0; //Counter for 12 MHz -> 2 Hz
    reg start = 0;
    
    always @(posedge clock) begin
        
        //Logic that allows the FPGA to be reset
        if (btn1) start <= 1;
        if (btn2) begin
            start <= 0;
            dataReg <= 8'b11111111;
            data <= 8'b00000000;
        end
        
        //Logic for writing at 2 Hz
        if (start) begin 
    
            if (Counter == 6000000/2) begin
                send <= 1;
            end 
            
            if (Counter == 6000000) begin
                send <= 0;
                data <= data + 1;
                dataReg <= dataReg - 1;
                Counter <= 0;
            end 
            
            Counter <= Counter + 1;
        
        end
    end
    
endmodule
