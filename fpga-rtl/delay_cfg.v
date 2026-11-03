`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// delay_cfg : parse "D<mode>,<depth>\n" from the shared 115200 byte stream.
//   mode  0 = bypass, 1 = fixed depth, 2 = jitter ceiling
//   depth 0..15, saturated (the injector clamps to D_MAX again)
// Non-D bytes are ignored; a malformed D command is dropped atomically.
// Reset value is bypass, so a fresh bitstream behaves like the old one.
// -----------------------------------------------------------------------------
module delay_cfg (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] rx_byte,
    input  wire       rx_valid,
    output reg  [1:0] d_mode,
    output reg  [3:0] d_depth,
    output reg        d_valid
);

    localparam [1:0] MODE_DEF  = 2'd0;
    localparam [3:0] DEPTH_DEF = 4'd0;

    reg       active;
    reg       field;      // 0 = mode, 1 = depth
    reg [7:0] acc;
    reg [2:0] nd;
    reg [1:0] mode_tmp;

    wire is_digit = (rx_byte >= 8'h30) && (rx_byte <= 8'h39);
    wire is_comma = (rx_byte == 8'h2C);
    wire is_eol   = (rx_byte == 8'h0A) || (rx_byte == 8'h0D);

    // saturate at 99 so three digits cannot overflow
    wire [7:0] acc_x10 = (acc > 8'd9) ? 8'd99 : (acc * 8'd10);
    wire [7:0] acc_nxt = acc_x10 + {5'd0, (rx_byte - 8'h30)};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active   <= 1'b0;
            field    <= 1'b0;
            acc      <= 8'd0;
            nd       <= 3'd0;
            mode_tmp <= 2'd0;
            d_mode   <= MODE_DEF;
            d_depth  <= DEPTH_DEF;
            d_valid  <= 1'b0;
        end else begin
            d_valid <= 1'b0;
            if (rx_valid) begin
                if (rx_byte == 8'h44 || rx_byte == 8'h64) begin          // 'D' / 'd'
                    active <= 1'b1;
                    field  <= 1'b0;
                    acc    <= 8'd0;
                    nd     <= 3'd0;
                end else if (active) begin
                    if (is_digit && nd < 3'd3) begin
                        acc <= acc_nxt;
                        nd  <= nd + 3'd1;
                    end else if (is_comma && !field && nd > 0) begin
                        mode_tmp <= (acc <= 8'd2) ? acc[1:0] : 2'd3;
                        field    <= 1'b1;
                        acc      <= 8'd0;
                        nd       <= 3'd0;
                    end else if (is_eol && field && nd > 0) begin
                        if (mode_tmp <= 2'd2) begin
                            d_mode  <= mode_tmp;
                            d_depth <= (acc > 8'd15) ? 4'd15 : acc[3:0];
                            d_valid <= 1'b1;
                        end
                        active <= 1'b0;
                    end else begin
                        active <= 1'b0;                                  // malformed: drop
                    end
                end
            end
        end
    end

endmodule
