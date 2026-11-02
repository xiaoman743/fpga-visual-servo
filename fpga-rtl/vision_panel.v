`timescale 1ns/1ps
// Four compact seven-segment-style fields: signed X, signed Y, signed gap, run.
// Rendered directly on the existing LCD stream; no framebuffer/font ROM needed.
module vision_panel(
    input wire clk,rst_n,
    input wire [10:0] x,y,
    input wire [15:0] rgb,
    input wire signed [15:0] pole_x,pole_y,gap,
    input wire running,
    input wire [1:0] selected,
    output reg [15:0] out_rgb
);
    // Stable bundled values sampled only at display frame origin.
    reg signed [15:0] px,py,g;
    reg run;
    reg [1:0] sel;
    reg [3:0] digits[0:11];
    reg [2:0] signs;
    reg [15:0] magnitude;
    reg [3:0] conversion;
    reg [1:0] field;
    reg [3:0] digit;
    reg [6:0] segments;
    reg ink;
    wire in_panel=x<208 && y<60;
    wire [1:0] row=y/20;
    wire [4:0] sx=x%20;
    wire [4:0] sy=y%20;
    wire [3:0] column=x/20;
    function [6:0] seg;
        input [3:0] n;
        begin case(n)
          0:seg=7'b1111110;1:seg=7'b0110000;2:seg=7'b1101101;
          3:seg=7'b1111001;4:seg=7'b0110011;5:seg=7'b1011011;
          6:seg=7'b1011111;7:seg=7'b1110000;8:seg=7'b1111111;
          9:seg=7'b1111011;default:seg=0;
        endcase end
    endfunction
    // Decimal conversion is sequential and outside the pixel combinational path.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            px<=409;py<=-165;g<=0;run<=0;sel<=0;conversion<=0;field<=0;
            magnitude<=0;signs<=0;
            digits[0]<=0;digits[1]<=4;digits[2]<=0;digits[3]<=9;
            digits[4]<=0;digits[5]<=1;digits[6]<=6;digits[7]<=5;
            digits[8]<=0;digits[9]<=0;digits[10]<=0;digits[11]<=0;
        end else begin
            if(x==0 && y==0) begin
                px<=pole_x;py<=pole_y;g<=gap;run<=running;sel<=selected;
                signs<={gap[15],pole_y[15],pole_x[15]};field<=0;conversion<=1;
            end else if(conversion==1) begin
                case(field)
                  0:magnitude<=px[15] ? -px:px;
                  1:magnitude<=py[15] ? -py:py;
                  2:magnitude<=g[15] ? -g:g;
                endcase
                conversion<=2;
            end else if(conversion>=2 && conversion<=5) begin
                digits[field*4+5-conversion]<=magnitude%10;
                magnitude<=magnitude/10;
                if(conversion==5) begin
                    if(field==2) conversion<=0;
                    else begin field<=field+1;conversion<=1;end
                end else conversion<=conversion+1;
            end
        end
    end
    always @(*) begin
        digit=0;segments=0;ink=0;
        if(row<3) begin
            if(column>=2 && column<6) begin digit=digits[row*4+column-2];segments=seg(digit);end
            if(column==1 && signs[row]) segments=7'b0000001;
            // Field IDs 1/2/3; selected field is yellow, others white.
            if(column==0) segments=seg({2'd0,row}+1);
            ink=(segments[6] && sy>=2 && sy<=3 && sx>=4 && sx<=12) ||
                (segments[5] && sx>=12 && sx<=13 && sy>=3 && sy<=8) ||
                (segments[4] && sx>=12 && sx<=13 && sy>=10 && sy<=15) ||
                (segments[3] && sy>=15 && sy<=16 && sx>=4 && sx<=12) ||
                (segments[2] && sx>=3 && sx<=4 && sy>=10 && sy<=15) ||
                (segments[1] && sx>=3 && sx<=4 && sy>=3 && sy<=8) ||
                (segments[0] && sy>=8 && sy<=9 && sx>=4 && sx<=12);
        end
        if(in_panel) begin
            if(x>=145 && x<195) out_rgb=run ? 16'h07e0:16'hf800;
            else out_rgb=ink ? (row==sel ? 16'hffe0:16'hffff):16'h0000;
        end else out_rgb=rgb;
    end
endmodule
