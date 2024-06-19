`timescale 1ns / 1ps


module I2C_Testbench;

//Module I/O 
reg Clock;

wire SCL;
wire SDA;

wire SDA_r;
reg SDA_w;
reg SDA_en = 0;

reg Send;
reg RW;
wire Rec;
reg ReadAmount;

reg [6:0] Address;
reg [7:0] DataReg;
reg [7:0] Data;

wire [7:0] 
    dataRecReg0, 
    dataRecReg1, 
    dataRecReg2, 
    dataRecReg3,
    dataRecReg4,
    dataRecReg5;

//Set up tri state driver
assign SDA = SDA_en ? SDA_w : 1'bZ;
assign SDA_r = SDA;

//Sender data
reg [7:0] SendData;

//Test unit
I2C I2C_mod(Clock, SDA, SCL, Send, Rec, RW, ReadAmount, Address, DataReg, Data,
    dataRecReg0, 
    dataRecReg1, 
    dataRecReg2, 
    dataRecReg3,
    dataRecReg4,
    dataRecReg5);

//Logic
reg updateData = 1;

initial begin

    //Register initialization
    Clock = 0;
    Address = 7'b1001011;
    DataReg = 8'b11111111;
    Data = 8'b00000000;
    Send = 1;
    RW = 1;
    ReadAmount = 1;

    SendData = 8'b10000000;

    forever begin
    
        #1 Clock = ~Clock;
        if (I2C_mod.SDA_en == 0) begin
            if (I2C_mod.State == 4) begin
                SDA_en <= 1;
                SDA_w <= 0;
            end else if (I2C_mod.State == 6) begin
                SDA_en <= 1;
                SDA_w <= SendData[I2C_mod.dataCounter];
            end
        end else begin
            SDA_en <= 0;
        end
        
        //Writes test bugfixing
        if (I2C_mod.State == 7) begin
            Send = 0;
            if (updateData) begin
                //DataReg <= DataReg - 1;
                //Data <= Data + 1;
                updateData <= 0;
            end
        end
        if (I2C_mod.State == 0) begin
            Send = 1;
            updateData <= 1;
        end
        //--------------------- 
        
    end
end

endmodule
