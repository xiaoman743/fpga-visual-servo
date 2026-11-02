`timescale 1ns/1ps
// Independent vision demo: configuration, target lock, radial clearance,
// servo and checked telemetry. No joint or gripper control is hidden here.
module vision_demo #(
    parameter integer UART_CLK_FREQ=50000000,UART_BAUD=115200
)(
    input wire clk,cam_clk,rst_n,
    input wire [3:0] key_n,
    input wire [7:0] rx_byte,
    input wire rx_valid,feat_done,
    input wire [15:0] red_x,red_y,blue_x,blue_y,
    input wire found_red,found_blue,
    output wire [3:0] led_n,
    output wire txd,txd2,
    output wire signed [15:0] cfg_x,cfg_y,cfg_gap,
    output wire cfg_run,
    output wire [1:0] selected,
    output wire [15:0] goal_x,goal_y,
    output wire goal_found
);
    wire [1:0] key_events;
    vision_config config_keys(.clk(clk),.rst_n(rst_n),.key_n(key_n),
        .rx_byte(rx_byte),.rx_valid(rx_valid),.pole_x(cfg_x),.pole_y(cfg_y),
        .gap(cfg_gap),.running(cfg_run),.selected(selected),.key_events(key_events),.led_n(led_n));

    // Bundled-data handshake: source holds the entire configuration until ack.
    reg [50:0] mailbox;
    reg req;
    (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,req_sync;
    reg ack;
    wire [50:0] current={key_events,cfg_x,cfg_y,cfg_gap,cfg_run};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin mailbox<=0;req<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],ack};
            if(req==ack_sync[1] && mailbox!=current) begin mailbox<=current;req<=~req;end
        end
    end
    reg signed [15:0] px,py,gap;
    reg run;
    reg [50:0] pending;
    reg [1:0] cam_key_events;
    reg pending_valid;
    wire config_wait=(req_sync[1]!=ack) || pending_valid;
    always @(posedge cam_clk or negedge rst_n) begin
        if(!rst_n) begin
            req_sync<=0;ack<=0;px<=409;py<=-165;gap<=0;run<=0;
            pending<=0;pending_valid<=0;cam_key_events<=0;
        end else begin
            req_sync<={req_sync[0],req};
            if(req_sync[1]!=ack) begin pending<=mailbox;pending_valid<=1;ack<=req_sync[1];end
            else if(feat_done && pending_valid) begin
                {cam_key_events,px,py,gap,run}<=pending;pending_valid<=0;
            end
        end
    end
    wire [15:0] locked_x,locked_y;
    wire locked_found;
    target_gate gate(.clk(cam_clk),.rst_n(rst_n),.feat_done(feat_done),
        .cx_b(blue_x),.cy_b(blue_y),.found_b(found_blue),
        .gx_b(locked_x),.gy_b(locked_y),.gfound_b(locked_found),
        .o_reacquired(),.o_gated());
    reg stage,goal_start,servo_start,uart_stage;
    reg [15:0] rx,ry,bx,by,seq;
    reg fr,fb,active;
    reg [15:0] frame_px,frame_py,frame_gap;
    wire goal_done,goal_valid;
    wire [15:0] gx,gy;
    vision_goal goal(.clk(cam_clk),.rst_n(rst_n),.start(goal_start),
        .bx(locked_x),.by(locked_y),.pole_x(px),.pole_y(py),.gap(gap),
        .done(goal_done),.valid(goal_valid),.gx(gx),.gy(gy));
    wire signed [15:0] pan,reach;
    wire servo_valid,hold;
    servo_polar #(.K_THETA_Q10(4),.K_R_Q10(4),.DEADZONE(6),.LIMIT(4),.HOLD_N(8)) servo(
        .clk(cam_clk),.rst_n(rst_n),.feat_done(servo_start),
        .cx_r(rx),.cy_r(ry),.cx_b(gx),.cy_b(gy),
        .found_r(fr && active && goal_valid),.found_b(fb && locked_found && active && goal_valid),
        .cfg_cx(frame_px),.cfg_cy(frame_py),.d_pan(pan),.d_reach(reach),.o_valid(servo_valid),.o_hold(hold));
    reg [207:0] frame_payload;
    reg frame_toggle;
    wire [7:0] flags={1'b1,cam_key_events,goal_valid,(hold && active),active,fb,fr};
    always @(posedge cam_clk or negedge rst_n) begin
        if(!rst_n) begin
            stage<=0;goal_start<=0;servo_start<=0;uart_stage<=0;
            rx<=0;ry<=0;bx<=0;by<=0;seq<=0;fr<=0;fb<=0;active<=0;
            frame_px<=409;frame_py<=-165;frame_gap<=0;frame_payload<=0;frame_toggle<=0;
        end else begin
            stage<=feat_done;goal_start<=stage;servo_start<=goal_done;
            if(feat_done) begin
                rx<=red_x;ry<=red_y;bx<=blue_x;by<=blue_y;
                fr<=found_red;fb<=found_blue;
                active<=run && !config_wait;
                frame_px<=px;frame_py<=py;frame_gap<=gap;
            end
            if(servo_valid) begin
                frame_payload<={8'd2,flags,seq,rx,ry,bx,by,gx,gy,frame_px,frame_py,frame_gap,pan,reach};
                frame_toggle<=~frame_toggle;seq<=seq+1;
            end
        end
    end
    (* ASYNC_REG="TRUE" *) reg [2:0] frame_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) frame_sync<=0;else frame_sync<={frame_sync[1:0],frame_toggle};
    end
    vision_uart #(.CLK_FREQ(UART_CLK_FREQ),.BAUD(UART_BAUD)) uart(.clk(clk),.rst_n(rst_n),.valid(frame_sync[2]^frame_sync[1]),
                     .payload(frame_payload),.txd(txd),.txd2(txd2));
    assign goal_x=gx;
    assign goal_y=gy;
    assign goal_found=goal_valid && fr && fb;
endmodule
