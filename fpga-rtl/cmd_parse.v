`timescale 1ns/1ps
// Atomic signed-decimal C<x>,<y> command; legacy uint16 two's complement allowed.
module cmd_parse #(
    parameter integer C_X_DEF=409,C_Y_DEF=-165
)(input wire clk,rst_n,input wire [7:0] rx_byte,input wire rx_valid,
  output reg [15:0] c_x,c_y,output reg cfg_valid);
    reg state;
    reg active,negative;
    reg [16:0] accumulator;
    reg [2:0] digits;
    reg [15:0] pending_x;
    wire digit=rx_byte>="0" && rx_byte<="9";
    wire [15:0] value=negative ? -accumulator[15:0]:accumulator[15:0];
    wire signed [15:0] signed_value=value;
    wire allowed=digits>0 && accumulator<=65535 &&
                 (!negative || accumulator<=2048) && signed_value>=-2048 && signed_value<=2047;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            c_x<=C_X_DEF;c_y<=C_Y_DEF;cfg_valid<=0;state<=0;active<=0;
            accumulator<=0;digits<=0;negative<=0;pending_x<=0;
        end else begin
            cfg_valid<=0;
            if(rx_valid) begin
                if(rx_byte=="C" || rx_byte=="c") begin
                    active<=1;state<=0;accumulator<=0;digits<=0;negative<=0;
                end else if(active) begin
                    if((rx_byte=="-" || rx_byte=="+") && digits==0 && !negative) begin
                        negative<=rx_byte=="-";
                    end else if(digit && digits<5) begin
                        accumulator<=accumulator*10+(rx_byte-"0");digits<=digits+1;
                    end else if(rx_byte=="," && !state && allowed) begin
                        pending_x<=value;state<=1;accumulator<=0;digits<=0;negative<=0;
                    end else if((rx_byte==10 || rx_byte==13) && state && allowed) begin
                        c_x<=pending_x;c_y<=value;cfg_valid<=1;active<=0;
                    end else active<=0;
                end
            end
        end
    end
endmodule
