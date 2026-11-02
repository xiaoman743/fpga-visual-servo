`timescale 1ns/1ps
// Physical keys, active low. Changes always disarm visual motion.
module vision_config #(
    parameter integer DEBOUNCE = 1000000,
    parameter integer REPEAT_DELAY = 25000000,
    parameter integer REPEAT_RATE = 5000000
)(
    input wire clk, rst_n,
    input wire [3:0] key_n,
    input wire [7:0] rx_byte,
    input wire rx_valid,
    output reg signed [15:0] pole_x, pole_y, gap,
    output reg running,
    output reg [1:0] selected,
    output reg [1:0] key_events,
    output wire [3:0] led_n
);
    (* ASYNC_REG="TRUE" *) reg [3:0] ks1, ks2;
    reg [3:0] stable;
    reg [31:0] debounce[0:3];
    reg [3:0] press;
    integer i;
    assign led_n = ~{running, selected==2, selected==1, selected==0};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ks1<=4'hf; ks2<=4'hf; stable<=4'hf; press<=0;
            for(i=0;i<4;i=i+1) begin debounce[i]<=0; end
        end else begin
            ks1<=key_n; ks2<=ks1; press<=0;
            for(i=0;i<4;i=i+1) begin
                if(ks2[i] != stable[i]) begin
                    if(debounce[i] == DEBOUNCE-1) begin
                        stable[i]<=ks2[i]; debounce[i]<=0;
                        if(!ks2[i]) press[i]<=1;
                    end else debounce[i]<=debounce[i]+1;
                end else begin
                    debounce[i]<=0;
                end
            end
        end
    end
    // UART is an optional exact-value alternative to physical keys.
    wire [15:0] ux,uy;
    wire uv;
    cmd_parse parser(.clk(clk),.rst_n(rst_n),.rx_byte(rx_byte),.rx_valid(rx_valid),
                     .c_x(ux),.c_y(uy),.cfg_valid(uv));
    // C supports signed decimal; V0/V1 and G<signed pixels> are parsed here.
    reg [1:0] mode;
    reg neg;
    reg [15:0] accum;
    reg [3:0] digits;
    wire digit = rx_byte>="0" && rx_byte<="9";
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pole_x<=409; pole_y<=-165; gap<=0; running<=0; selected<=0;key_events<=0;
            mode<=0; neg<=0; accum<=0; digits<=0;
        end else begin
            // KEY0 is intentionally unassigned. KEY1/2 are one event per press.
            if(press[1]) begin key_events[0]<=~key_events[0];running<=0;end
            if(press[2]) begin key_events[1]<=~key_events[1];running<=0;end
            if(press[3]&&!press[1]&&!press[2]) running<=~running;
            if(uv) begin pole_x<=ux; pole_y<=uy; running<=0; end
            if(rx_valid) begin
                if(rx_byte=="V") mode<=1;
                else if(rx_byte=="G") begin mode<=2; accum<=0; digits<=0; neg<=0; end
                else if(mode==1) begin
                    if(rx_byte=="0") running<=0;
                    else if(rx_byte=="1") running<=1;
                    mode<=0;
                end else if(mode==2) begin
                    if(rx_byte=="-" && digits==0 && !neg) neg<=1;
                    else if(digit && digits<3) begin
                        accum<=accum*10+(rx_byte-"0"); digits<=digits+1;
                    end else if((rx_byte==10 || rx_byte==13) && digits>0 && accum<=200) begin
                        gap<=neg ? -$signed(accum) : $signed(accum); running<=0; mode<=0;
                    end else mode<=0;
                end
            end
        end
    end
endmodule
