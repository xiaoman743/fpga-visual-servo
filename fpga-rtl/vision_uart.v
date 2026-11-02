`timescale 1ns/1ps
// V2: '@' + 28 bytes as 56 uppercase hex digits + CR LF (59 bytes).
// 26 payload bytes + CRC16-CCITT (init FFFF, poly 1021, high byte first).
module vision_uart #(
    parameter integer CLK_FREQ=50000000,BAUD=115200
)(
    input wire clk,rst_n,valid,
    input wire [207:0] payload,
    output wire txd,txd2
);
    reg [223:0] packet;
    reg [207:0] crc_data;
    reg [15:0] crc;
    reg [8:0] bits_left;
    reg [6:0] index;
    reg [1:0] state;
    reg start;
    reg [7:0] data;
    wire busy;
    wire feedback=crc[15]^crc_data[207];
    wire [15:0] crc_next=(crc<<1)^(feedback ? 16'h1021:16'h0000);
    wire [3:0] nibble=packet[223:220];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            packet<=0;crc_data<=0;crc<=16'hffff;bits_left<=0;index<=0;
            state<=0;start<=0;data<=0;
        end else begin
            start<=0;
            case(state)
                0: if(valid && !busy) begin
                    packet<={payload,16'd0};crc_data<=payload;crc<=16'hffff;
                    bits_left<=208;state<=1;
                end
                1: begin
                    crc<=crc_next;crc_data<=crc_data<<1;bits_left<=bits_left-1;
                    if(bits_left==1) begin packet[15:0]<=crc_next;index<=0;state<=2;end
                end
                2: if(!busy && !start) begin
                    start<=1;
                    if(index==0) data<="@";
                    else if(index<=56) begin
                        data<=nibble<10 ? (8'h30+{4'd0,nibble}):(8'h41+{4'd0,nibble}-10);
                        packet<=packet<<4;
                    end else if(index==57) data<=13;
                    else begin data<=10;state<=0;end
                    index<=index+1;
                end
            endcase
        end
    end
    uart_tx #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) u_tx(
        .clk(clk),.rst_n(rst_n),.start(start),.data(data),.busy(busy),.txd(txd));
    assign txd2=txd;
endmodule
