//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved         
//----------------------------------------------------------------------------------------
// File name:           ov5640_lcd
// Created by:          正点原子
// Created date:        2025年11月010日14:20:02
// Version:             V1.0
// Descriptions:        OV5640摄像头LCD显示
//                      
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module ov5640_lcd(    
    input                 sys_clk      ,  //系统时钟
    input                 sys_rst_n    ,  //系统复位，低电平有效
    //摄像头接口                       
    input                 cam_pclk     ,  //CMOS 数据像素时钟
    input                 cam_vsync    ,  //CMOS 场同步信号
    input                 cam_href     ,  //CMOS 行同步信号
    input   [7:0]         cam_data     ,  //CMOS 数据
    output                cam_rst_n    ,  //CMOS 复位信号，低电平有效
    output                cam_pwdn     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl      ,  //CMOS SCCB_SCL线
    inout                 cam_sda      ,  //CMOS SCCB_SDA线       
    // DDR3                            
    inout   [31:0]        ddr3_dq      ,  //DDR3 数据
    inout   [3:0]         ddr3_dqs_n   ,  //DDR3 dqs负
    inout   [3:0]         ddr3_dqs_p   ,  //DDR3 dqs正  
    output  [13:0]        ddr3_addr    ,  //DDR3 地址   
    output  [2:0]         ddr3_ba      ,  //DDR3 bank 选择
    output                ddr3_ras_n   ,  //DDR3 行选择
    output                ddr3_cas_n   ,  //DDR3 列选择
    output                ddr3_we_n    ,  //DDR3 读写选择
    output                ddr3_reset_n ,  //DDR3 复位
    output  [0:0]         ddr3_ck_p    ,  //DDR3 时钟正
    output  [0:0]         ddr3_ck_n    ,  //DDR3 时钟负
    output  [0:0]         ddr3_cke     ,  //DDR3 时钟使能
    output  [0:0]         ddr3_cs_n    ,  //DDR3 片选
    output  [3:0]         ddr3_dm      ,  
    output  [0:0]         ddr3_odt     ,                                         
    //lcd接口                           
    output                lcd_hs       ,  //LCD 行同步信号
    output                lcd_vs       ,  //LCD 场同步信号
    output                lcd_de       ,  //LCD 数据输入使能
    inout       [23:0]    lcd_rgb      ,  //LCD 颜色数据
    output                lcd_bl       ,  //LCD 背光控制信号
    output                lcd_rst      ,  //LCD 复位信号
    output                lcd_pclk        ,          //LCD 采样时钟
    input                 uart_rxd           ,  // uart rx : pole command (PC -> FPGA)
    output                uart_txd           ,  // uart tx : centroid output (CH340)
    input [3:0]           key_n,
    output [3:0]          led_n,
    output                uart_txd2             // uart tx #2 : ATK MODULE D16 -> ESP32
    );                                 
                                                                   
//wire define                          
wire         clk_50m                   ;  //50MHz时钟,提供给lcd驱动时钟
wire         locked                    ;  //时钟锁定信号
wire         rst_n                     ;  //全局复位                                                                            
wire  [15:0] wrdata                    ;  //DDR3控制器模块写数据
wire         rdata_req                 ;  //DDR3控制器模块读请求
wire  [15:0] rddata                    ;  //DDR3控制器模块读数据
wire [15:0] cx_r, cy_r, cx_b, cy_b;
wire [15:0] cx_b_raw, cy_b_raw;  // blue centroid straight from vision_features (pre-gate)
wire        found_r, found_b, feat_done;
wire        found_b_raw;         // pre-gate found_blue
wire [15:0] panel_rgb;
wire [15:0] rddata_osd;                  // pixel after osd overlay
wire [10:0] lcd_xpos, lcd_ypos;          // lcd pixel coordinates
wire signed [15:0] d_pan, d_reach;       // servo commands (ticks)
wire [7:0]  rx_byte;                     // uart rx byte
wire        rx_valid;                    // uart rx strobe
wire [15:0] cfg_cx, cfg_cy;              // runtime pole c
wire [1:0]  d_mode;                  // delay injector mode  (delay_cfg)
wire [3:0]  d_depth;                 // delay injector depth in frames
wire        feat_done_d;             // delayed frame strobe
wire [15:0] dcx_r, dcy_r, dcx_b, dcy_b;
wire        dfound_r, dfound_b;

wire         cmos_frame_valid          ;  //数据有效使能信号
wire         init_calib_complete       ;  //DDR3初始化完成信号
wire         sys_init_done             ;  //系统初始化完成
wire         clk_200m                  ;  //DDR3参考时钟
wire         cmos_frame_vsync          ;  //输出帧有效场同步信号
wire         lcd_clk                   ;  //分频产生的LCD 采样时钟
wire  [12:0] h_disp                    ;  //LCD屏水平分辨率
wire  [12:0] v_disp                    ;  //LCD屏垂直分辨率           
wire  [12:0] y_addr_st                 ; 
wire  [12:0] y_addr_end                ;
wire  [15:0] lcd_id                    ;  //LCD屏的ID号
wire  [27:0] ddr3_addr_max             ;  //存入DDR3的最大读写地址               
wire  [12:0] total_h_pixel             ;  //水平总像素大小 
wire  [12:0] total_v_pixel             ;  //垂直总像素大小
//*****************************************************
//**                    main code
//*****************************************************
//待时钟锁定后产生复位结束信号
assign  rst_n = sys_rst_n & locked;

//系统初始化完成：DDR3初始化完成
assign  sys_init_done = init_calib_complete;

//摄像头图像分辨率设置模块
picture_size u_picture_size (
    .rst_n                 (rst_n              ),
    .clk                   (clk_50m            ),    
    .lcd_id                (lcd_id             ),  //LCD的器件ID
                        
    .cmos_h_pixel          (h_disp             ),  //摄像头水平分辨率
    .cmos_v_pixel          (v_disp             ),  //摄像头垂直分辨率  
    .total_h_pixel         (total_h_pixel      ),  //水平总像素大小
    .total_v_pixel         (total_v_pixel      ),  //垂直总像素大小
    .y_addr_st             (y_addr_st          ), 
    .y_addr_end            (y_addr_end         ),
    .ddr3_addr_max         (ddr3_addr_max      )   //DDR3最大读写地址
    );

//ov5640 驱动
ov5640_dri u_ov5640_dri(
    .clk                   (clk_50m            ),
    .rst_n                 (rst_n              ),

    .cam_pclk              (cam_pclk           ),
    .cam_vsync             (cam_vsync          ),
    .cam_href              (cam_href           ),
    .cam_data              (cam_data           ),
    .cam_rst_n             (cam_rst_n          ),
    .cam_pwdn              (cam_pwdn           ),
    .cam_scl               (cam_scl            ),
    .cam_sda               (cam_sda            ),
    
    .capture_start         (init_calib_complete),
    .cmos_h_pixel          (h_disp             ),
    .cmos_v_pixel          (v_disp             ),
    .total_h_pixel         (total_h_pixel      ),
    .total_v_pixel         (total_v_pixel      ),
    .y_addr_st             (y_addr_st          ), 
    .y_addr_end            (y_addr_end         ),
    .cmos_frame_vsync      (cmos_frame_vsync   ),
    .cmos_frame_href       (                   ),
    .cmos_frame_valid      (cmos_frame_valid   ),
    .cmos_frame_data       (wrdata             )
    );   

ddr3_top u_ddr3_top(
    .rst_n                 (rst_n              ),  //复位,低有效
    .init_calib_complete   (init_calib_complete),  //DDR3初始化完成信号    
    //ddr3接口信号         
    .app_addr_rd_min       (28'd0              ),  //读DDR3的起始地址
    .app_addr_rd_max       (ddr3_addr_max[27:1]),  //读DDR3的结束地址
    .rd_bust_len           (h_disp[10:4]       ),  //从DDR3中读数据时的突发长度
    .app_addr_wr_min       (28'd0              ),  //写DDR3的起始地址
    .app_addr_wr_max       (ddr3_addr_max[27:1]),  //写DDR3的结束地址
    .wr_bust_len           (h_disp[10:4]       ),  //从DDR3中写数据时的突发长度
    // DDR3 IO接口                
    .ddr3_dq               (ddr3_dq            ),  //DDR3 数据
    .ddr3_dqs_n            (ddr3_dqs_n         ),  //DDR3 dqs负
    .ddr3_dqs_p            (ddr3_dqs_p         ),  //DDR3 dqs正  
    .ddr3_addr             (ddr3_addr          ),  //DDR3 地址   
    .ddr3_ba               (ddr3_ba            ),  //DDR3 bank 选择
    .ddr3_ras_n            (ddr3_ras_n         ),  //DDR3 行选择
    .ddr3_cas_n            (ddr3_cas_n         ),  //DDR3 列选择
    .ddr3_we_n             (ddr3_we_n          ),  //DDR3 读写选择
    .ddr3_reset_n          (ddr3_reset_n       ),  //DDR3 复位
    .ddr3_ck_p             (ddr3_ck_p          ),  //DDR3 时钟正
    .ddr3_ck_n             (ddr3_ck_n          ),  //DDR3 时钟负  
    .ddr3_cke              (ddr3_cke           ),  //DDR3 时钟使能
    .ddr3_cs_n             (ddr3_cs_n          ),  //DDR3 片选
    .ddr3_dm               (ddr3_dm            ),  //DDR3_dm
    .ddr3_odt              (ddr3_odt           ),  //DDR3_odt
    // System Clock Ports                            
    .sys_clk_i             (clk_200m           ),   
    // Reference Clock Ports                         
    .clk_ref_i             (clk_200m           ),    
    //用户
    .ddr3_read_valid       (1'b1               ),  //DDR3 读使能
    .ddr3_pingpang_en      (1'b1               ),  //DDR3 乒乓操作使能
    .wr_clk                (cam_pclk           ),  //写时钟
    .wr_load               (cmos_frame_vsync   ),  //输入源更新信号   
    .wr_en                 (cmos_frame_valid   ),  //数据有效使能信号
    .wrdata                (wrdata             ),  //写有效数据 
    .rd_clk                (lcd_clk            ),  //读时钟 
    .rd_load               (rd_vsync           ),  //输出源更新信号    
    .rddata                (rddata             ),  //rfifo输出数据
    .rdata_req             (rdata_req          )   //请求数据输入     
     );                

 clk_wiz_0 u_clk_wiz_0
   (
    // Clock out ports
    .clk_out1              (clk_200m           ),     
    .clk_out2              (clk_50m            ),
    // Status and control signals
    .reset                 (~sys_rst_n         ), 
    .locked                (locked             ),       
   // Clock in ports
    .clk_in1               (sys_clk            )
    );     

//LCD驱动显示模块
// ---------------- self-developed : dual-feature centroid ----------------
vision_features #(
    .H_PIXEL (800),
    .V_PIXEL (480)
) u_vision (
    .clk          (cam_pclk),
    .rst_n        (rst_n),
    .i_vsync      (cmos_frame_vsync),
    .i_valid      (cmos_frame_valid),
    .i_rgb        (wrdata),
    .o_valid      (feat_done),
    .o_x_red      (cx_r),
    .o_y_red      (cy_r),
    .o_found_red  (found_r),
    .o_x_blue     (cx_b_raw),
    .o_y_blue     (cy_b_raw),
    .o_found_blue (found_b_raw)
);

// ---------------- self-developed : delay injector (P5) ----------------
delay_inject #(
    .D_MAX     (8),
    .D_FIXED   (2),
    .D_MAX_JIT (8),
    .LFSR_SEED (16'hACE1),
    .JIT_STEP  (8),
    .MODE      (2'd0)
) u_delay (
    .clk       (cam_pclk),
    .rst_n     (rst_n),
    .feat_done (feat_done),
    .cx_r      (cx_r),
    .cy_r      (cy_r),
    .cx_b      (cx_b_raw),
    .cy_b      (cy_b_raw),
    .found_r   (found_r),
    .found_b   (found_b_raw),
    .d_mode    (d_mode),
    .d_depth   (d_depth),
    .o_valid   (feat_done_d),
    .o_cx_r    (dcx_r),
    .o_cy_r    (dcy_r),
    .o_cx_b    (dcx_b),
    .o_cy_b    (dcy_b),
    .o_found_r (dfound_r),
    .o_found_b (dfound_b),
    .o_delay   ()
);

// "D<mode>,<depth>\n" on the same 115200 byte stream as the C command.
delay_cfg u_delay_cfg (
    .clk      (clk_50m),
    .rst_n    (rst_n),
    .rx_byte  (rx_byte),
    .rx_valid (rx_valid),
    .d_mode   (d_mode),
    .d_depth  (d_depth),
    .d_valid  ()
);

// Independent visual demo, with board keys and a checked V2 UART stream.
uart_rx #(.CLK_FREQ(50000000),.BAUD(115200)) u_rx (
    .clk(clk_50m),.rst_n(rst_n),.rxd(uart_rxd),.o_data(rx_byte),.o_valid(rx_valid));
wire signed [15:0] demo_x,demo_y,demo_gap;
wire demo_run,demo_goal_found;
wire [1:0] demo_selected;
wire [15:0] demo_goal_x,demo_goal_y;
vision_demo u_demo (
    .clk(clk_50m),.cam_clk(cam_pclk),.rst_n(rst_n),.key_n(key_n),
    .rx_byte(rx_byte),.rx_valid(rx_valid),.feat_done(feat_done_d),
    .red_x(dcx_r),.red_y(dcy_r),.blue_x(dcx_b),.blue_y(dcy_b),
    .found_red(dfound_r),.found_blue(dfound_b),.led_n(led_n),
    .txd(uart_txd),.txd2(uart_txd2),.cfg_x(demo_x),.cfg_y(demo_y),
    .cfg_gap(demo_gap),.cfg_run(demo_run),.selected(demo_selected),
    .goal_x(demo_goal_x),.goal_y(demo_goal_y),.goal_found(demo_goal_found));
assign cx_b=demo_goal_x;
assign cy_b=demo_goal_y;
assign found_b=demo_goal_found;

vision_panel u_panel(.clk(lcd_clk),.rst_n(rst_n),.x(lcd_xpos),.y(lcd_ypos),
    .rgb(rddata),.pole_x(demo_x),.pole_y(demo_y),.gap(demo_gap),
    .running(demo_run),.selected(demo_selected),.out_rgb(panel_rgb));

// ---------------- self-developed : osd crosshair ----------------
osd_cross #(
    .ARM        (10),
    .THICK      (1),
    .COL_MARKER (16'hFFFF),
    .COL_OBJECT (16'h07E0)
) u_osd (
    .clk      (lcd_clk),
    .rst_n    (rst_n),
    .xpos     (lcd_xpos),
    .ypos     (lcd_ypos),
    .rgb_in   (panel_rgb),
    .rgb_out  (rddata_osd),
    .mx       (cx_r),
    .my       (cy_r),
    .m_found  (found_r),
    .ox       (cx_b),
    .oy       (cy_b),
    .o_found  (found_b)
);
lcd_rgb_top  u_lcd_rgb_top(
    .sys_clk               (clk_50m            ),
    .sys_rst_n             (rst_n              ),
    .sys_init_done         (sys_init_done      ),     
                           
    //lcd接口                            
    .lcd_id                (lcd_id             ),  //LCD屏的ID号 
    .lcd_hs                (lcd_hs             ),  //LCD 行同步信号
    .lcd_vs                (lcd_vs             ),  //LCD 场同步信号
    .lcd_de                (lcd_de             ),  //LCD 数据输入使能
    .lcd_rgb               (lcd_rgb            ),  //LCD 颜色数据
    .lcd_bl                (lcd_bl             ),  //LCD 背光控制信号
    .lcd_rst               (lcd_rst            ),  //LCD 复位信号
    .lcd_pclk              (lcd_pclk           ),  //LCD 采样时钟
    .lcd_clk               (lcd_clk            ),  //LCD 驱动时钟
    //用户接口                     
    .out_vsync             (rd_vsync           ),  //LCD场信号
    .h_disp                (                   ),  //行分辨率  
    .v_disp                (                   ),  //场分辨率  
    .pixel_xpos            (lcd_xpos),
    .pixel_ypos            (lcd_ypos),       
    .data_in               (rddata_osd),  //rfifo输出数据
    .data_req              (rdata_req          )   //请求数据输入
    );   
    
endmodule
