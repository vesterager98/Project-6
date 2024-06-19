`timescale 1ns / 1ps


module PWM(

    output wire out_o,
    input wire clock,
    input wire [15:0] duty

    );

    reg [17:0] ClockCount = 0;
    reg Out;

always @(posedge clock)
begin

    if (ClockCount >= duty)
        Out <= 0;
    else
        Out <= 1;
    
    if (ClockCount == 240000)
        ClockCount = 0;
    else
        ClockCount <= ClockCount + 1;
    
end

assign out_o = Out;

endmodule
