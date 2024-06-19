`timescale 1ns / 1ps



//I2C master module
module I2C(

    //Clock
    input wire clock_i,

    //Physical wires
    inout wire SDA_io,
    output wire SCL_o,
    
    //Control I/O
    input wire SendMSG_i,
    output wire RecMSG_o,
    input wire ReadWrite_i,
    input wire [2:0] readAmount_i,
    
    //Basic data
    input wire [6:0] address_i,
    input wire [7:0] datareg_i,
    
    //Payload I/O
    input wire [7:0] dataSenReg_i,

    output wire [7:0] dataRecReg0_o, 
    output wire [7:0] dataRecReg1_o, 
    output wire [7:0] dataRecReg2_o, 
    output wire [7:0] dataRecReg3_o,
    output wire [7:0] dataRecReg4_o,
    output wire [7:0] dataRecReg5_o

    ); //----------------------------------------------------------------------------------

//States
parameter [3:0] IDLE = 0;
parameter [3:0] START = 1;
parameter [3:0] ADDRESSING = 2;
parameter [3:0] READWRITE = 3;
parameter [3:0] ACK = 4;
parameter [3:0] DATAWRITE = 5;
parameter [3:0] DATAREAD = 6;
parameter [3:0] STOP = 7;
parameter [3:0] PRESTART = 8;

//Control regs
reg [3:0] State = 0;
reg Initial = 1;
reg ReadNow = 0;
reg SendMSG = 0;
reg RecMSG = 0;
reg SendLock = 0;

//Settings regs
reg ReadWrite = 1;
reg [2:0] ReadAmount = 1;
reg [6:0] addressPusher = 0;
reg [7:0] dataPusher = 0;

//Counter regs
reg [4:0] ClockCounter = 0;
reg [2:0] addressCounter = 0;
reg [3:0] dataCounter = 0;
reg [7:0] transCounter = 0;

// Recieving data storage regs
reg [7:0] 
    dataRecReg0 = 0, 
    dataRecReg1 = 0, 
    dataRecReg2 = 0, 
    dataRecReg3 = 0,
    dataRecReg4 = 0,
    dataRecReg5 = 0;

// Sending data storage regs
reg [7:0] dataSenReg = 0;

//IO Regs
reg SCL = 1;
reg SDA_w = 1;
wire SDA_r;
reg SDA_en = 1;

//Start state machine
always @(posedge clock_i)
begin

    //Sending lock
    if (SendMSG_i == 0) SendLock <= 0;
    SendMSG <= SendMSG_i;

    //If the clock triggers
    if (ClockCounter == 30) begin
        
        //State machine
        case (State)
        
            IDLE: begin //------------------------------------------------ 
                if (SendMSG == 1 & SendLock == 0) begin 
                    //Next stage + activate lock
                    State <= START;
                    SendMSG <= 0;
                    SendLock <= 1;
                    RecMSG <= 0;
                    //Reset control regs
                    Initial <= 1;
                    ReadNow <= 0;
                    //Load data
                    addressPusher <= address_i;
                    dataPusher <= datareg_i;
                    dataSenReg <= dataSenReg_i;
                    ReadWrite <= ReadWrite_i;
                    ReadAmount <= readAmount_i;
                    //Set SDA line to write just to be sure
                    SDA_en <= 1;
                end
            end
            
            START: begin //------------------------------------------------ 
                SDA_w <= 0;
            end
            
            ADDRESSING: begin //------------------------------------------------ 
                //Send next bit
                SDA_w <= addressPusher[6-addressCounter];
                addressCounter <= addressCounter + 1;
            end
            
            READWRITE: begin //------------------------------------------------
                if (Initial) begin
                    SDA_w <= 0;
                end else begin
                    SDA_w <= 1;
                end
            end
            
            ACK: begin //------------------------------------------------
                if (ReadNow) begin                          //If we're sending ACKS 
                    if (transCounter < ReadAmount) begin
                        SDA_w <= 0;
                        SDA_en <= 1;
                    end else begin
                        SDA_w <= 1;
                        SDA_en <= 1;
                    end
                end else begin                              //If we're recieving ACKS
                    SDA_en <= 0;
                end
            end
            
            DATAWRITE: begin //------------------------------------------------
                //Send next bit
                SDA_w <= dataPusher[7-dataCounter];
                dataCounter <= dataCounter + 1;
            end
            
            DATAREAD: begin //------------------------------------------------
                SDA_en <= 0;
            end
            
            STOP : begin //------------------------------------------------
                if (SCL == 1) begin
                    SDA_w <= 1;
                    State <= IDLE;
                    //Reset control regs
                    Initial <= 1;
                    ReadNow <= 0;
                    RecMSG <= 0;
                end
            end
            
            PRESTART : SDA_w <= 1; //------------------------------------------------
                
            default: State <= IDLE; //------------------------------------------------ 
        
        endcase
        
        //Reset ClockCounter
        ClockCounter <= 0;
    
    end 
    
    //If the clock is odd I 
    if (ClockCounter == (30/2-5)) begin
        
        case(State)
        
            START : begin //------------------------------------------------
                if (SDA_w == 0) begin
                    SCL <= 0;
                    State <= ADDRESSING;
                end
            end
            
            ADDRESSING : SCL <= 1; //------------------------------------------------
            
            READWRITE : SCL <= 1; //------------------------------------------------
            
            ACK : SCL <= 1; //------------------------------------------------
            
            DATAWRITE : SCL <= 1; //------------------------------------------------
            
            DATAREAD: SCL <= 1; //------------------------------------------------
             
            PRESTART : begin //------------------------------------------------
                if (SDA_w == 1) begin
                    SCL <= 1;
                    State <= START;
                end
            end
           
        endcase
    end
    
    //If the clock is odd II
    if (ClockCounter == (30/2+5)) begin
        
        case(State)
        
            ADDRESSING : begin //------------------------------------------------
                SCL <= 0;
                //When all address bits have been sent
                if (addressCounter == 7) begin
                    State <= READWRITE;
                    addressCounter <= 0;
                    transCounter <= transCounter + 1;
                end
            end
            
            READWRITE : begin //------------------------------------------------
                SCL <= 0;
                State <= ACK;
            end
            
            ACK : begin //------------------------------------------------
            
                SCL <= 0;                               //Update SCL
                
                if (~ReadNow) begin                         //If we're recieving ACKS
                    if (SDA_r == 0) begin                       //If ACK is recieved  
                        
                        if (Initial) begin                          //Initial stage
                            if (transCounter == 2) begin
                                Initial <= 0;
                                transCounter <= 0;
                                if (~ReadWrite) begin                   //Switch to write stage
                                    State <= DATAWRITE;
                                    dataPusher <= dataSenReg;
                                end else begin                          //Switch to read stage
                                    State <= PRESTART;
                                end
                            end else begin                          //Initial continues to write reg
                                State <= DATAWRITE;
                            end
                        end else if (~Initial & ~ReadWrite) begin       //Add Write stage
                                State <= STOP;
                                transCounter <= 0;
                        end else if (transCounter == 1) begin           //Add Read stage
                            ReadNow <= 1;
                            transCounter <= 0;
                            State <= DATAREAD;
                        end
                        
                        SDA_w <= 0;                                 //Base response
                        SDA_en <= 1;                                
                        
                    end else begin                              //If NACK is recieved
                    
                        State <= IDLE;
                        SDA_w <= 1;
                        SDA_en <= 1;  
                        
                    end 
                end else begin                              //If we're writing ACKS
                    if (transCounter < ReadAmount) begin
                        State <= DATAREAD;
                    end else begin
                        State <= STOP;
                        RecMSG <= 1;
                        SDA_w <= 0;
                        transCounter <= 0;
                    end
                end
            end    
            
            DATAWRITE : begin //------------------------------------------------
                SCL <= 0;
                //When all address bits have been sent
                if (dataCounter == 8) begin
                    State <= ACK;
                    dataCounter <= 0;
                    transCounter <= transCounter + 1;
                end
            end
            
            DATAREAD : begin //------------------------------------------------
                SCL <= 0;
                //Read bit
                dataPusher[7-dataCounter] <= SDA_r;
                dataCounter <= dataCounter + 1;
                //If a byte is fully read
                if (dataCounter == 8) begin
                    case(transCounter)
                        0: dataRecReg0 <= dataPusher;
                        1: dataRecReg1 <= dataPusher;
                        2: dataRecReg2 <= dataPusher;
                        3: dataRecReg3 <= dataPusher;
                        4: dataRecReg4 <= dataPusher;
                        5: dataRecReg5 <= dataPusher;
                    endcase
                    State <= ACK;
                    dataCounter <= 0;
                    transCounter <= transCounter + 1;
                end
            end
            
            STOP : SCL <= 1; //------------------------------------------------
           
        endcase  
    end
    
    //Increment clock
    ClockCounter <= ClockCounter + 1;  
      
end

//Assign outputs //----------------------------------------

//Tri state driver
assign SDA_io = SDA_en ? SDA_w : 1'bZ;
assign SDA_r = SDA_io;

// SCL
assign SCL_o = SCL;

// Data outs
assign dataRecReg0_o = dataRecReg0;
assign dataRecReg1_o = dataRecReg1;
assign dataRecReg2_o = dataRecReg2;
assign dataRecReg3_o = dataRecReg3;
assign dataRecReg4_o = dataRecReg4;
assign dataRecReg5_o = dataRecReg5;

//Message complete marker
assign RecMSG_o = RecMSG;

endmodule









