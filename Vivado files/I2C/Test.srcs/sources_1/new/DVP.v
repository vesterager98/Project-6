`timescale 1ns / 1ps


module DVP(

    input HSync_i,
    input VSync_i,
    input PCLK_i,
    input [9:0] Data_i

    );

    reg [3:0] pixl = 4'b0000;
    
    always @(posedge PCLK_i) begin
    
        pixl[0] <= Data_i[0];
        pixl[1] <= Data_i[1];
        pixl[2] <= Data_i[2];
        pixl[3] <= Data_i[3];
    
    end
    
endmodule
