// rexta_usb_top.sv
// rexta project — Tang Nano 1K USB HID Host controller
//
// Interfaces:
//   - Two USB low-speed HID ports via nand2mario usb_hid_host core
//       Port A: usb_a_dp / usb_a_dm  (either keyboard or mouse)
//       Port B: usb_b_dp / usb_b_dm  (either keyboard or mouse)
//   - SPI slave (Mode 0, 8-byte fixed transaction) → PicoRV32 / Cyclone IV
//   - Single IRQ active-low output → CPU
//
// Device routing is determined at runtime from each port's typ output:
//   typ=01 → keyboard fields updated from that port
//   typ=10 → mouse fields updated from that port
//   If both ports report the same type, last report wins.
//
// Clock: 27 MHz crystal → rPLL → 24 MHz → ÷2 → 12 MHz for USB cores
// See design document §5.1 for PLL verification steps.

`default_nettype none
`timescale 1ns / 1ps

module rexta_usb_top (
    input  wire   clk27,        // 27 MHz crystal
    input  wire   rst_n,        // active-low reset (button or power-on)

    // USB port A — plug in keyboard or mouse
    // 30Ω series resistors and 14.7kΩ pull-downs to GND required on board.
    inout  wire   usb_a_dp,
    inout  wire   usb_a_dm,

    // USB port B — plug in keyboard or mouse
    inout  wire   usb_b_dp,
    inout  wire   usb_b_dm,

    // SPI slave (to PicoRV32 / Cyclone IV)
    input  wire   spi_cs_n,     // chip-select, active low
    input  wire   spi_sck,
    input  wire   spi_mosi,
    output logic  spi_miso,

    // IRQ to CPU, active low — asserted when any port has a pending event
    output logic  irq_n,

    // Debug LEDs — single onboard RGB LED, each channel active low
    output logic  led_red_n,    // unused — held off
    output logic  led_green_n,  // lit when a mouse is connected
    output logic  led_blue_n    // lit when a keyboard is connected
);

// ─────────────────────────────────────────────────────────────────────────────
// 1. Clock generation
//    rPLL parameters from Gowin IDE wizard (GW1NZ-1, 27 MHz → 12 MHz):
//      IDIV=8 (÷9), FBDIV=3 (×4), ODIV=64 → VCO=768 MHz, FOUT=12 MHz
// ─────────────────────────────────────────────────────────────────────────────
wire clk12;

rPLL #(
    .FCLKIN          ("27"),
    .IDIV_SEL        (8),
    .FBDIV_SEL       (3),
    .ODIV_SEL        (64),
    .DYN_IDIV_SEL    ("false"),
    .DYN_FBDIV_SEL   ("false"),
    .DYN_ODIV_SEL    ("false"),
    .PSDA_SEL        ("0000"),
    .DYN_DA_EN       ("true"),
    .DUTYDA_SEL      ("1000"),
    .CLKOUT_FT_DIR   (1'b1),
    .CLKOUTP_FT_DIR  (1'b1),
    .CLKOUT_DLY_STEP (0),
    .CLKOUTP_DLY_STEP(0),
    .CLKFB_SEL       ("internal"),
    .CLKOUT_BYPASS   ("false"),
    .CLKOUTP_BYPASS  ("false"),
    .CLKOUTD_BYPASS  ("false"),
    .DYN_SDIV_SEL    (2),
    .CLKOUTD_SRC     ("CLKOUT"),
    .CLKOUTD3_SRC    ("CLKOUT"),
    .DEVICE          ("GW1NZ-1")
) u_pll (
    .CLKOUT  (clk12),
    .LOCK    (),
    .CLKIN   (clk27),
    .CLKFB   (clk12),
    .RESET   (~rst_n),
    .RESET_P (1'b0),
    .FBDSEL  (6'b0),
    .IDSEL   (6'b0),
    .ODSEL   (6'b0),
    .PSDA    (4'b0),
    .DUTYDA  (4'b0),
    .FDLY    (4'b0),
    .CLKOUTP (),
    .CLKOUTD (),
    .CLKOUTD3()
);

// ─────────────────────────────────────────────────────────────────────────────
// 2. USB HID Host cores — one per port, neutral A/B naming
// ─────────────────────────────────────────────────────────────────────────────

// Port A
logic        a_report;
logic [1:0]  a_typ;
logic [7:0]  a_key_modifiers;
logic [7:0]  a_key1, a_key2, a_key3, a_key4;
logic [7:0]  a_mouse_btn;
logic [7:0]  a_mouse_dx, a_mouse_dy;

usb_hid_host u_hid_a (
    .usbclk        (clk12),
    .usbrst_n      (rst_n),
    .usb_dm        (usb_a_dm),
    .usb_dp        (usb_a_dp),
    .report        (a_report),
    .typ           (a_typ),
    .conerr        (),
    .key_modifiers (a_key_modifiers),
    .key1          (a_key1),
    .key2          (a_key2),
    .key3          (a_key3),
    .key4          (a_key4),
    .mouse_btn     (a_mouse_btn),
    .mouse_dx      (a_mouse_dx),
    .mouse_dy      (a_mouse_dy),
    .game_l(), .game_r(), .game_u(), .game_d(),
    .game_a(), .game_b(), .game_x(), .game_y(),
    .game_sel(), .game_sta(),
    .dbg_hid_report()
);

// Port B
logic        b_report;
logic [1:0]  b_typ;
logic [7:0]  b_key_modifiers;
logic [7:0]  b_key1, b_key2, b_key3, b_key4;
logic [7:0]  b_mouse_btn;
logic [7:0]  b_mouse_dx, b_mouse_dy;

usb_hid_host u_hid_b (
    .usbclk        (clk12),
    .usbrst_n      (rst_n),
    .usb_dm        (usb_b_dm),
    .usb_dp        (usb_b_dp),
    .report        (b_report),
    .typ           (b_typ),
    .conerr        (),
    .key_modifiers (b_key_modifiers),
    .key1          (b_key1),
    .key2          (b_key2),
    .key3          (b_key3),
    .key4          (b_key4),
    .mouse_btn     (b_mouse_btn),
    .mouse_dx      (b_mouse_dx),
    .mouse_dy      (b_mouse_dy),
    .game_l(), .game_r(), .game_u(), .game_d(),
    .game_a(), .game_b(), .game_x(), .game_y(),
    .game_sel(), .game_sta(),
    .dbg_hid_report()
);

// ─────────────────────────────────────────────────────────────────────────────
// 3. Runtime device routing
//    typ is stable between reports — safe to use combinatorially for routing.
//    kbd_connected / mse_connected: true if either port has that device type.
//    If both ports report the same type, both contribute (last report wins
//    for the shadow fields, which is fine in practice).
// ─────────────────────────────────────────────────────────────────────────────
localparam TYP_KBD = 2'b01;
localparam TYP_MSE = 2'b10;

logic kbd_connected, mse_connected;
assign kbd_connected = (a_typ == TYP_KBD) | (b_typ == TYP_KBD);
assign mse_connected = (a_typ == TYP_MSE) | (b_typ == TYP_MSE);

// Keyboard report: fires when whichever port has a keyboard sends a report
logic kbd_report;
logic [7:0] kbd_key_modifiers;
logic [7:0] kbd_key1, kbd_key2, kbd_key3, kbd_key4;

assign kbd_report        = (a_report & (a_typ == TYP_KBD)) |
                           (b_report & (b_typ == TYP_KBD));
assign kbd_key_modifiers = (a_typ == TYP_KBD) ? a_key_modifiers : b_key_modifiers;
assign kbd_key1          = (a_typ == TYP_KBD) ? a_key1          : b_key1;
assign kbd_key2          = (a_typ == TYP_KBD) ? a_key2          : b_key2;
assign kbd_key3          = (a_typ == TYP_KBD) ? a_key3          : b_key3;
assign kbd_key4          = (a_typ == TYP_KBD) ? a_key4          : b_key4;

// Mouse report: fires when whichever port has a mouse sends a report
logic mse_report;
logic [7:0] mse_mouse_btn;
logic [7:0] mse_mouse_dx, mse_mouse_dy;

assign mse_report    = (a_report & (a_typ == TYP_MSE)) |
                       (b_report & (b_typ == TYP_MSE));
assign mse_mouse_btn = (a_typ == TYP_MSE) ? a_mouse_btn : b_mouse_btn;
assign mse_mouse_dx  = (a_typ == TYP_MSE) ? a_mouse_dx  : b_mouse_dx;
assign mse_mouse_dy  = (a_typ == TYP_MSE) ? a_mouse_dy  : b_mouse_dy;

// ─────────────────────────────────────────────────────────────────────────────
// 4. Event shadow register
//    Keyboard and mouse fields updated independently.
//
//    Byte 0  STATUS
//              [7] irq_pending     — kbd_pending | mse_pending
//              [6] kbd_pending     — keyboard has an unread event
//              [5] mse_pending     — mouse has an unread event
//              [4] kbd_connected   — a keyboard is enumerated on either port
//              [3] mse_connected   — a mouse is enumerated on either port
//              [2] mouse_btn_mid
//              [1] mouse_btn_right
//              [0] mouse_btn_left
//    Byte 1  KEY_MOD
//    Byte 2  KEY1
//    Byte 3  KEY2
//    Byte 4  KEY3
//    Byte 5  KEY4
//    Byte 6  MOUSE_DX  (signed)
//    Byte 7  MOUSE_DY  (signed)
// ─────────────────────────────────────────────────────────────────────────────
logic [7:0] shadow [0:7];
logic       kbd_pending;
logic       mse_pending;
logic       ack_clear;

always_ff @(posedge clk12 or negedge rst_n) begin
    if (!rst_n) begin
        kbd_pending <= 1'b0;
        mse_pending <= 1'b0;
        for (int i = 0; i < 8; i++) shadow[i] <= 8'h00;
    end else begin

        if (kbd_report) begin
            kbd_pending <= 1'b1;
            shadow[1]   <= kbd_key_modifiers;
            shadow[2]   <= kbd_key1;
            shadow[3]   <= kbd_key2;
            shadow[4]   <= kbd_key3;
            shadow[5]   <= kbd_key4;
        end

        if (mse_report) begin
            mse_pending <= 1'b1;
            shadow[6]   <= mse_mouse_dx;
            shadow[7]   <= mse_mouse_dy;
        end

        // STATUS rebuilt every cycle from live state
        shadow[0] <= {kbd_pending | mse_pending,  // [7] irq_pending
                      kbd_pending,                // [6] kbd_pending
                      mse_pending,                // [5] mse_pending
                      kbd_connected,              // [4] kbd_connected
                      mse_connected,              // [3] mse_connected
                      mse_mouse_btn[2:0]};        // [2:0] {mid, right, left}

        if (ack_clear) begin
            kbd_pending <= 1'b0;
            mse_pending <= 1'b0;
        end
    end
end

// ─────────────────────────────────────────────────────────────────────────────
// 5. IRQ output
// ─────────────────────────────────────────────────────────────────────────────
assign irq_n = ~(kbd_pending | mse_pending);

// ─────────────────────────────────────────────────────────────────────────────
// 5. Debug LEDs (active low)
// ─────────────────────────────────────────────────────────────────────────────
assign led_red_n   = 1'b1;        // off — unused
assign led_blue_n  = ~kbd_connected;  // blue  — keyboard connected
assign led_green_n = ~mse_connected;  // green — mouse connected

// ─────────────────────────────────────────────────────────────────────────────
// 7. SPI Slave — Mode 0 (CPOL=0, CPHA=0), MSB first, fixed 8-byte transaction
//
//    All logic shares clk12 — no CDC required.
//    Keep SCK ≤ 4 MHz so clk12 sampling is reliable (see design doc §5.4).
//
//    MOSI byte 0 = CMD:  bit[1]=ACK (clears both pending flags)
//    MISO bytes 0–7 = shadow register
// ─────────────────────────────────────────────────────────────────────────────
logic [5:0] bit_cnt;
logic [7:0] miso_shift;
logic [7:0] mosi_shift;
logic [7:0] cmd_byte;

logic [2:0] byte_idx;
assign byte_idx = bit_cnt[5:3];

logic sck_prev;
always_ff @(posedge clk12) sck_prev <= spi_sck;
logic sck_rise, sck_fall;
assign sck_rise =  spi_sck & ~sck_prev;
assign sck_fall = ~spi_sck &  sck_prev;

logic cs_prev;
always_ff @(posedge clk12) cs_prev <= spi_cs_n;
logic cs_assert, cs_deassert;
assign cs_assert   = ~spi_cs_n &  cs_prev;
assign cs_deassert =  spi_cs_n & ~cs_prev;

always_ff @(posedge clk12 or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt    <= '0;
        miso_shift <= '0;
        mosi_shift <= '0;
        cmd_byte   <= '0;
        ack_clear  <= 1'b0;
    end else begin
        ack_clear <= 1'b0;

        if (cs_assert) begin
            bit_cnt    <= '0;
            miso_shift <= shadow[0];
            mosi_shift <= '0;
        end else if (!spi_cs_n) begin
            if (sck_fall) begin
                if (bit_cnt[2:0] == 3'd7 && byte_idx < 3'd7)
                    miso_shift <= shadow[byte_idx + 1'b1];
                else
                    miso_shift <= {miso_shift[6:0], 1'b0};
            end
            if (sck_rise) begin
                mosi_shift <= {mosi_shift[6:0], spi_mosi};
                if (bit_cnt == 6'd7)
                    cmd_byte <= {mosi_shift[6:0], spi_mosi};
                bit_cnt <= bit_cnt + 1'b1;
            end
        end

        if (cs_deassert) begin
            if (cmd_byte[1])
                ack_clear <= 1'b1;
            bit_cnt <= '0;
        end
    end
end

assign spi_miso = spi_cs_n ? 1'bz : miso_shift[7];

endmodule