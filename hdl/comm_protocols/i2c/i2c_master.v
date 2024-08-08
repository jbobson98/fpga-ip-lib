/*




*/
`timescale 1ns / 1ps


module i2c_master #(
    parameter SYS_CLOCK_FREQ_HZ = 100_000_000,  // (Hz) Frequency of clk_i
    parameter FAST_MODE_ENABLED = 0             // set to 1 for i2c fast mode, 0 for standard
)(
    input wire       i_clk,
    input wire       i_rst,
    input wire [9:0] i_slave_addr,     // Slave address to read/write to. For 7-bit addr mode, use bits [6:0]
    input wire [7:0] i_byte_cnt,       // Number of bytes to read from or write to slave
    input wire [3:0] i_control_reg,    // [1'bstart_transaction 1'bClear_status ... unused]
    input wire [3:0] i_mode_reg,       // [1'bAddr_mode 1'bRW_mode .. unused] 
    input wire [7:0] i_tx_data,        // byte to be writen to slave
    
    output reg       o_rx_data_valid,
    output reg       o_tx_data_needed, // Pulses high when next byte to send to slave is needed (allows for fifo)
    output reg [7:0] o_rx_data,
    output reg [4:0] o_status_reg,     // [1'bBUSY 1'bTX_DONE 1'bRX_DONE 1'bTX_ERR 1'bRX_ERROR] 

    inout wire io_sda,
    inout wire io_scl

);

/* Generate Serial Clock Signals */
localparam integer I2C_STANDARD_MODE_FREQ_HZ = 100_000;
localparam integer I2C_FAST_MODE_FREQ_HZ     = 400_000;
localparam integer MODE_FREQ  = (FAST_MODE_ENABLED) ? I2C_FAST_MODE_FREQ_HZ : I2C_STANDARD_MODE_FREQ_HZ;
localparam integer TICK_MAX   = (SYS_CLOCK_FREQ_HZ / (4*MODE_FREQ)) - 1;
localparam integer TICK_WIDTH = $clog2(TICK_MAX + 1);
wire tick;
wire [TICK_WIDTH-1 : 0] tick_generator_count;
flex_counter #(.MAX_COUNT(TICK_MAX), .WIDTH(TICK_WIDTH)) tick_generator 
    (.clk(clk_i), .rst(rst_i), .cen(1'b1), .maxcnt(tick), .count(tick_generator_count));

/* I2C Signals and Filtering */
reg sda = 1'b1;
reg scl = 1'b1;
reg sda_out_en, scl_out_en;
wire sda_in, scl_in, sda_flt, scl_flt;
assign sda_in = io_sda;
assign scl_in = io_scl;
i2c_filter sda_filter (.i_clk(i_clk), .i_rst(i_rst), .i_in(sda_in), .o_out(sda_flt));
i2c_filter scl_filter (.i_clk(i_clk), .i_rst(i_rst), .i_in(scl_in), .o_out(scl_flt));

/* Control Signals */
wire start_trans, clear_status, addr_mode, rw_mode;
assign start_trans  = i_control_reg[3];
assign clear_status = i_control_reg[2];
assign addr_mode    = i_mode_reg[3];      // 0 = 7bit mode,  1 = 10bit mode
assign rw_mode      = i_mode_reg[2];      // 0 = write mode, 1 = read mode 

/* Status register bit masks */
localparam [4:0] BUSY_MASK     = 5'b10000;
localparam [4:0] TX_DONE_MASK  = 5'b01000;
localparam [4:0] RX_DONE_MASK  = 5'b00100;
localparam [4:0] TX_ERROR_MASK = 5'b00010;
localparam [4:0] RX_ERROR_MASK = 5'b00001;

/* Internal Registers */
// these should be registered in when we get the start signal
reg [9:0] slave_addr;
reg [7:0] byte_cnt;
reg [7:0] tx_data; // registered to i_tx_data
reg [7:0] transmit_byte; // byte to tx
reg [1:0] tick_cnt;

/* Slave Address Parsing */ // this is wrong, first byte should always have rw_mode set to 1'b0 see docs
wire [7:0] addr7_byte1, addr10_byte1, addr10_byte2;
assign addr7_byte1  = {slave_addr[6:0], rw_mode};
assign addr10_byte1 = {5'b11110, slave_addr[9:8], rw_mode};
assign addr10_byte2 = slave_addr[7:0];

/* TX Controller */
wire tx_ctrl_sda_out, tx_ctrl_scl_out;
wire tx_ctrl_sda_disable, tx_ctrl_scl_disable;
wire tx_ctrl_done, tx_ctrl_error;
wire tx_ctrl_start;
i2c_tx_byte_controller tx_controller (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_tick(tick),
    .i_tx_start(tx_ctrl_start),
    .i_tx_data(tx_data),
    .i_scl(scl_flt),
    .i_sda(sda_flt),
    .o_tx_done(tx_ctrl_done),
    .o_tx_error(tx_ctrl_error),
    .o_sda_disable(tx_ctrl_sda_disable),
    .o_scl_disable(tx_ctrl_scl_disable),
    .o_sda(tx_ctrl_sda_out),
    .o_scl(tx_ctrl_scl_out)
);











/* Tri-State Output */
assign io_sda  = (sda_out_en && ~tx_ctrl_sda_disable) ? sda : 1'bz;
assign io_scl  = (scl_out_en && ~tx_ctrl_scl_disable) ? scl : 1'bz;
//sda/scl <= tx_ctrl_sda_out when we are in the transmit state
// if we have tx_ctrl_error, then 
// TODO: logic for when SDA/SCL_OUT_EN high or low



localparam IDLE       = 8'd0;
localparam START      = 8'd1;
localparam ADDR_1     = 8'd2;
localparam ADDR_2     = 8'd3;
localparam ADDR_3     = 8'd4;



reg [7:0] state;


always @(posedge i_clk or posedge i_rst) begin
    if(i_rst) begin
        state <= IDLE;
    end else begin
        case(state)
            IDLE:   begin
                        if(start_trans) begin
                            slave_addr <= i_slave_addr;
                            byte_cnt <= i_byte_cnt;
                            tx_data <= i_tx_data;

                        end
                    end
            START: 









        endcase
    end
end


/* STATES

    IDLE
    SEND_START
    SEND_SLAVE_ADDR_1 (return to here a second time for 10bit mode but)
    SEND_SLAVE_ADDR_2




*/














endmodule