`timescale 1ns/1ps
// goal = blue - gap * unit(blue-pole). Sequential sqrt and division.
// A positive gap leaves the marker nearer the base than the object.
module vision_goal(
    input wire clk,rst_n,start,
    input wire [15:0] bx,by,
    input wire signed [15:0] pole_x,pole_y,gap,
    output reg done,valid,
    output reg [15:0] gx,gy
);
    reg [2:0] state;
    reg signed [31:0] dx,dy;
    reg [15:0] xb,yb;
    reg signed [15:0] g;
    reg [63:0] radicand,root_rem;
    reg [31:0] root;
    reg [5:0] count;
    reg [31:0] numerator_x,numerator_y,quot_x,quot_y;
    reg [32:0] rem_x,rem_y;
    reg [31:0] divisor;
    reg sign_x,sign_y;
    wire [63:0] trial_rem=(root_rem<<2)|{62'd0,radicand[63:62]};
    wire [63:0] trial_root=({32'd0,root}<<2)|64'd1;
    wire [31:0] squared=dx*dx+dy*dy;
    wire [32:0] div_rem_x={rem_x[31:0],numerator_x[31]};
    wire [32:0] div_rem_y={rem_y[31:0],numerator_y[31]};
    wire signed [31:0] result_x=$signed({16'd0,xb})-(sign_x ? -$signed(quot_x):$signed(quot_x));
    wire signed [31:0] result_y=$signed({16'd0,yb})-(sign_y ? -$signed(quot_y):$signed(quot_y));
    wire [31:0] absdx=dx[31] ? -dx:dx;
    wire [31:0] absdy=dy[31] ? -dy:dy;
    wire [15:0] absg=g[15] ? -g:g;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=0;done<=0;valid<=0;gx<=0;gy<=0;
            dx<=0;dy<=0;xb<=0;yb<=0;g<=0;radicand<=0;root<=0;root_rem<=0;
            count<=0;numerator_x<=0;numerator_y<=0;quot_x<=0;quot_y<=0;
            rem_x<=0;rem_y<=0;divisor<=0;sign_x<=0;sign_y<=0;
        end else begin
            done<=0;
            case(state)
                0: if(start) begin
                    xb<=bx;yb<=by;g<=gap;
                    dx<=$signed({1'b0,bx})-$signed(pole_x);
                    dy<=$signed({1'b0,by})-$signed(pole_y);
                    state<=1;valid<=0;
                end
                1: begin
                    radicand<={16'd0,squared,16'd0};root<=0;root_rem<=0;count<=32;state<=2;
                end
                2: begin
                    radicand<=radicand<<2;
                    if(trial_rem>=trial_root) begin root_rem<=trial_rem-trial_root;root<=(root<<1)|1;end
                    else begin root_rem<=trial_rem;root<=root<<1;end
                    count<=count-1;
                    if(count==1) state<=3;
                end
                3: begin
                    // Pole too close to the object, or gap beyond the pole: reject.
                    if(root<2048 || (g>=0 && {8'd0,absg,8'd0}>=root)) begin
                        gx<=xb;gy<=yb;valid<=0;done<=1;state<=0;
                    end else begin
                        numerator_x<=(absdx*absg)<<8;numerator_y<=(absdy*absg)<<8;
                        sign_x<=dx[31]^g[15];sign_y<=dy[31]^g[15];
                        divisor<=root;rem_x<=0;rem_y<=0;quot_x<=0;quot_y<=0;
                        count<=32;state<=4;
                    end
                end
                4: begin
                    numerator_x<=numerator_x<<1;numerator_y<=numerator_y<<1;
                    if(div_rem_x>={1'b0,divisor}) begin
                        rem_x<=div_rem_x-divisor;quot_x<=(quot_x<<1)|1;
                    end else begin rem_x<=div_rem_x;quot_x<=quot_x<<1;end
                    if(div_rem_y>={1'b0,divisor}) begin
                        rem_y<=div_rem_y-divisor;quot_y<=(quot_y<<1)|1;
                    end else begin rem_y<=div_rem_y;quot_y<=quot_y<<1;end
                    count<=count-1;
                    if(count==1) state<=5;
                end
                5: begin
                    valid<=result_x>=0 && result_x<800 && result_y>=0 && result_y<480;
                    gx<=result_x>=0 && result_x<800 ? result_x[15:0]:xb;
                    gy<=result_y>=0 && result_y<480 ? result_y[15:0]:yb;
                    done<=1;state<=0;
                end
            endcase
        end
    end
endmodule
