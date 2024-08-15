/*  ------------------------------------------------
    Module:     i2c_master
    Developer:  Jacob Bobson
    Date:       8/10/2024
    ------------------------------------------------

    Details:
        - supports 7 and 10 bit addr modes
        - supports standard or fast mode
        - see config regs below

    Note: this currently only works for 7 bit addressing
*/

/*
    JMB TODO: should also have clock stretching timeout timer on both rx and tx modules
*/

`timescale 1ns / 1ps


module i2c_master #(
    parameter SYS_CLOCK_FREQ_HZ = 100_000_000,  // (Hz) Frequency of i_clk
    parameter FAST_MODE_ENABLED = 0             // set to 1 for i2c fast mode, 0 for standard
)(
    input wire       i_clk,
    input wire       i_rst,
    input wire [9:0] i_slave_addr,       // Slave address to read/write to. For 7-bit addr mode, use bits [6:0]
    input wire [7:0] i_byte_cnt,         // Number of bytes to read from or write to slave
    input wire [3:0] i_control_reg,      // [1'bstart_transaction 1'bClear_status ... unused]
    input wire [3:0] i_mode_reg,         // [1'bAddr_mode 1'bRW_mode 1'braw_read unused] 
    input wire [7:0] i_tx_data,          // byte to be writen to slave
      
    output reg        o_rx_data_valid,
    output reg        o_tx_data_needed,  // Pulses high when next byte to send to slave is needed (allows for fifo)
    output wire [7:0] o_rx_data,
    output reg  [4:0] o_status_reg,      // [1'bBUSY 1'bTX_DONE 1'bRX_DONE 1'bTX_ERR 1'bRX_ERROR] 

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
flex_counter #(.MAX_COUNT(TICK_MAX), .WIDTH(TICK_WIDTH)) tick_generator 
    (.clk(i_clk), .rst(i_rst), .cen(1'b1), .maxcnt(tick), .count());

/* I2C Signals and Filtering */
reg sda        = 1'b1;
reg scl        = 1'b1;
reg sda_out_en = 1'b0;
reg scl_out_en = 1'b0;
wire sda_in, scl_in, sda_flt, scl_flt;
assign sda_in = io_sda;
assign scl_in = io_scl;
i2c_filter sda_filter (.i_clk(i_clk), .i_rst(i_rst), .i_in(sda_in), .o_out(sda_flt));
i2c_filter scl_filter (.i_clk(i_clk), .i_rst(i_rst), .i_in(scl_in), .o_out(scl_flt));

/* Status register bit masks */
localparam [4:0] BUSY_MASK     = 5'b10000;
localparam [4:0] TX_DONE_MASK  = 5'b01000;
localparam [4:0] RX_DONE_MASK  = 5'b00100;
localparam [4:0] TX_ERROR_MASK = 5'b00010;
localparam [4:0] RX_ERROR_MASK = 5'b00001;

/* STATES: Shared */
localparam IDLE  = 8'd0;
localparam START = 8'd1;
localparam STOP  = 8'd2;

/* STATES: Write Operation, 7-Bit Address Mode */
localparam OP_WRITE_7BIT_START    = 8'd3;
localparam OP_WRITE_7BIT_ADDR     = 8'd4;
localparam OP_WRITE_7BIT_DATA     = 8'd5;
localparam OP_WRITE_7BIT_REQ_DATA = 8'd6;

/* STATES: Read Operation, 7-Bit Address Mode */
localparam OP_READ_7BIT_START        = 8'd7;
localparam OP_READ_7BIT_DEV_ADDR_1   = 8'd8;
localparam OP_READ_7BIT_DEV_ADDR_2   = 8'd9;
localparam OP_READ_7BIT_REG_ADDR     = 8'd10;
localparam OP_READ_7BIT_REPEAT_START = 8'd11;
localparam OP_READ_7BIT_DATA_1       = 8'd12;
localparam OP_READ_7BIT_DATA_2       = 8'd13;

/* Internal Registers */
reg [9:0] slave_addr = 0;
reg [7:0] byte_cnt   = 0;
reg [7:0] tx_data    = 0;
reg [7:0] state      = IDLE;
reg [3:0] mode_reg   = 0;

/* Control Signals */
wire start_trans, clear_status, addr_10bit_mode, read_mode;
assign start_trans     = i_control_reg[3];  // Start i2c transaction  (active high)
assign clear_status    = i_control_reg[2];  // Clear status registers (active high)
assign addr_10bit_mode = mode_reg[3];       // 0 = 7bit mode,  1 = 10bit mode
assign read_mode       = mode_reg[2];       // 0 = write mode, 1 = read mode 
assign raw_read        = mode_reg[1];       // 0 = write address of slave reg then read, 1 = raw read command

/* Slave Address Parsing */
wire [7:0] addr10_byte1, addr10_byte2, addr10_read;
assign addr10_byte1 = {5'b11110, slave_addr[9:8], 1'b0};
assign addr10_byte2 = slave_addr[7:0];
assign addr10_read  = {5'b11110, slave_addr[9:8], read_mode};

/* TX Controller */
wire tx_ctrl_sda_out, tx_ctrl_scl_out;
wire tx_ctrl_sda_disable, tx_ctrl_scl_disable;
wire tx_ctrl_done, tx_ctrl_error;
reg tx_ctrl_start          = 1'b0;
reg [7:0] tx_ctrl_tx_data  = 0;
i2c_tx_byte_controller tx_controller (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_tick(tick),
    .i_tx_start(tx_ctrl_start),
    .i_tx_data(tx_ctrl_tx_data),
    .i_scl(scl_flt),
    .i_sda(sda_flt),
    .o_tx_done(tx_ctrl_done),
    .o_tx_error(tx_ctrl_error),
    .o_sda_disable(tx_ctrl_sda_disable),
    .o_scl_disable(tx_ctrl_scl_disable),
    .o_sda(tx_ctrl_sda_out),
    .o_scl(tx_ctrl_scl_out)
);

/* RX Controller */
wire rx_ctrl_sda_out, rx_ctrl_scl_out;
wire rx_ctrl_sda_disable, rx_ctrl_scl_disable;
wire rx_ctrl_done, rx_ctrl_error;
reg rx_ctrl_start     = 1'b0;
reg rx_ctrl_send_nack = 1'b0;
i2c_rx_byte_controller rx_controller (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_tick(tick),
    .i_rx_start(rx_ctrl_start),
    .i_scl(scl_flt),
    .i_sda(sda_flt),
    .i_send_nack(rx_ctrl_send_nack),
    .o_rx_done(rx_ctrl_done),
    .o_rx_error(rx_ctrl_error),
    .o_sda_disable(rx_ctrl_sda_disable),
    .o_scl_disable(rx_ctrl_scl_disable),
    .o_sda(rx_ctrl_sda_out),
    .o_scl(rx_ctrl_scl_out),
    .o_rx_data(o_rx_data)
);

/* Start Generator */
reg start_gen_start;
wire start_gen_sda, start_gen_scl, start_gen_done;
i2c_start_generator start_gen (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_tick(tick),
    .i_start(start_gen_start),
    .o_sda(start_gen_sda),
    .o_scl(start_gen_scl),
    .o_done(start_gen_done)
);

/* Stop Generator */
reg stop_gen_start;
wire stop_gen_sda, stop_gen_scl, stop_gen_done;
i2c_stop_generator stop_gen (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_tick(tick),
    .i_start(stop_gen_start),
    .o_sda(stop_gen_sda),
    .o_scl(stop_gen_scl),
    .o_done(stop_gen_done)
);

/* Tri-State Output */
wire sda_disable_on_read, scl_disable_on_read;
assign sda_disable_on_read = ((state == READ || state == OP_READ_7BIT_DATA_1) && rx_ctrl_sda_disable);
assign scl_disable_on_read = ((state == READ || state == OP_READ_7BIT_DATA_1) && rx_ctrl_scl_disable);
assign io_sda  = ((state != IDLE) && ~tx_ctrl_sda_disable && ~sda_disable_on_read) ? sda : 1'bz;
assign io_scl  = ((state != IDLE) && ~tx_ctrl_scl_disable && ~scl_disable_on_read) ? scl : 1'bz;

/* SDA / SCL Control */
always @(*) begin
    sda = 1'b1;
    scl = 1'b1;

    case(state)

        OP_WRITE_7BIT_START,
        OP_READ_7BIT_DATA_2,
        OP_READ_7BIT_START: 
        begin
            sda = 1'b0;
            scl = 1'b0;
        end

        START,
        OP_READ_7BIT_REPEAT_START:
        begin
            sda = start_gen_sda;
            scl = start_gen_scl;
        end

        OP_WRITE_7BIT_ADDR,
        OP_WRITE_7BIT_DATA,
        OP_WRITE_7BIT_REQ_DATA,
        OP_READ_7BIT_DEV_ADDR_1,
        OP_READ_7BIT_DEV_ADDR_2,
        OP_READ_7BIT_REG_ADDR:
        begin
            sda = tx_ctrl_sda_out;
            scl = tx_ctrl_scl_out;
        end
        
        OP_READ_7BIT_DATA_1:
        begin
            sda = rx_ctrl_sda_out;
            scl = rx_ctrl_scl_out;
        end

        STOP:   
        begin
            sda = stop_gen_sda;
            scl = stop_gen_scl;
        end
        default: ;
    endcase
end

/* Data registers that do not require a reset */
always @(posedge i_clk) begin
    if(state == IDLE && start_trans) begin
        mode_reg   <= i_mode_reg;
        slave_addr <= i_slave_addr;
        tx_data    <= i_tx_data;
    end
end

/* Control FSM ---------------------------------------------------------------------------------------------------------
    General Notes:
        - If master receives NACK from slave, master will initiate the stop sequence.

    Comment Reference Key:
        A/N = ACK/NACK Bit
        R/W = READ/WRITE Bit
        s2m = slave to master
        m2s = master to slave
        Op  = Operation
*/
always @(posedge i_clk) begin
    if(i_rst) begin
        state <= IDLE;
        o_tx_data_needed  <= 1'b0;
        o_rx_data_valid   <= 1'b0;
        start_gen_start   <= 1'b0;
        stop_gen_start    <= 1'b0;
        byte_cnt          <= 0;  
        tx_ctrl_start     <= 1'b0;
        tx_ctrl_tx_data   <= 0;
        o_status_reg      <= 5'b00000;
        rx_ctrl_send_nack <= 1'b0;
        rx_ctrl_start     <= 1'b0;
    end else begin
        case(state)

            /* IDLE --------------------------------------------------------------------- */
            IDLE: begin
                o_status_reg[4:2] <= 3'b000;
                o_tx_data_needed <= 1'b0;
                o_rx_data_valid <= 1'b0;
                if(start_trans) begin
                    byte_cnt <= i_byte_cnt;
                    state <= START;
                    start_gen_start <= 1'b1;
                    o_status_reg <= (o_status_reg | BUSY_MASK);
                end
            end

            /* START -------------------------------------------------------------------- */
            START: begin
                start_gen_start <= 1'b0;
                if(start_gen_done) begin
                    case({addr_10bit_mode, read_mode})

                        2'b00: state <= OP_WRITE_7BIT_START;  // Write operation, 7 bit address
                        2'b01: state <= OP_READ_7BIT_START;   // Read operation, 7 bit address



                        default: state <= IDLE;
                    endcase
                end
            end

            /* STOP --------------------------------------------------------------------- */
            STOP: begin
                stop_gen_start <= 1'b0;
                if(stop_gen_done) begin
                    state <= IDLE;
                end
            end


/* 7-BIT ADDRESS: Write Operation --------------------------------------------------------------------------------------
    Op:    [Start] [Slave Addr] [R/W = 0] [A/N_s2m] [Data Byte] [A/N_s2m] ... [Data Byte] [A/N_s2m] [Stop]
    Bits:               7           1         1         8           1              8          1  
*/
            /* Start */
            OP_WRITE_7BIT_START: begin // scl and sda low for this state
                tx_ctrl_start   <= 1'b1;
                tx_ctrl_tx_data <= {slave_addr[6:0], 1'b0};
                state           <= OP_WRITE_7BIT_ADDR;
            end
            
            /* Send Addr and R/W Bit */
            OP_WRITE_7BIT_ADDR: begin
                tx_ctrl_start <= 1'b0;
                if(tx_ctrl_done) begin
                    tx_ctrl_start <= 1'b1;
                    tx_ctrl_tx_data <= tx_data;
                    byte_cnt <= byte_cnt - 1;
                    state <= WRITE;
                end

                if(tx_ctrl_error) begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | TX_ERROR_MASK;
                end
            end

            /* Send Data */
            OP_WRITE_7BIT_DATA: begin
                tx_ctrl_start <= 1'b0;
                if(tx_ctrl_done) begin
                    if(byte_cnt != 0) begin
                        o_tx_data_needed <= 1'b1;
                        state <= OP_WRITE_7BIT_REQ_DATA;
                    end else begin
                        state <= STOP;
                        stop_gen_start <= 1'b1;
                        o_status_reg <= o_status_reg | TX_DONE_MASK;
                    end
                end

                if(tx_ctrl_error) begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | TX_ERROR_MASK;
                end
            end

            /* Request Data to Send */
            OP_WRITE_7BIT_REQ_DATA: begin
                o_tx_data_needed <= 1'b0;
                tx_ctrl_tx_data <= i_tx_data;
                tx_ctrl_start <= 1'b1;
                byte_cnt <= byte_cnt - 1;
                state <= OP_WRITE_7BIT_DATA;
            end


/* 7-BIT ADDRESS: Read Operation ---------------------------------------------------------------------------------------
    
    RAW READ:
    Op:    [Start] [Slave Addr] [R/W = 1] [A/N s2m] [Data Byte s2m] [A/N m2s] ... [Data Byte s2m] [NACK m2s] [Stop]
    Bits:               7           1        1             8           1                 8            1

    READ FROM DEVICE REGISTER:
    Op:    [Start] [Slave Addr] [R/W = 0] [A/N s2m] [Data Byte m2s (reg addr)] [A/N s2m] [Repeat Start] [Slave Addr] ->
                    [R/W = 1] [A/N s2m] [Data Byte s2m] [A/N m2s] ... [Data Byte s2m] [NACK m2s] [Stop]
*/
            /* Start */
            OP_READ_7BIT_START: begin
                tx_ctrl_start <= 1'b1;
                if(raw_read) begin
                    tx_ctrl_tx_data <= {slave_addr[6:0], 1'b1};
                end else begin
                    tx_ctrl_tx_data <= {slave_addr[6:0], 1'b0};
                end
                state <= OP_READ_7BIT_DEV_ADDR_1;
            end

            /* Send Device Addr and R/W Bit */
            OP_READ_7BIT_DEV_ADDR_1: begin
                tx_ctrl_start <= 1'b0;
                if(tx_ctrl_done) begin
                    if(raw_read) begin
                        if(byte_cnt == 1) begin
                            rx_ctrl_send_nack <= 1'b1;
                        end else begin
                            rx_ctrl_send_nack <= 1'b0;
                        end
                        state <= OP_READ_7BIT_DATA_1;
                        rx_ctrl_start <= 1'b1;
                        byte_cnt <= byte_cnt - 1;
                    end else begin
                        state <= OP_READ_7BIT_REG_ADDR;
                        tx_ctrl_start <= 1'b1;
                        tx_ctrl_tx_data <= tx_data;
                    end
                end

                if(tx_ctrl_error) begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | TX_ERROR_MASK;
                end
            end

            /* Send Register Addr and R/W Bit */
            OP_READ_7BIT_REG_ADDR: begin
                tx_ctrl_start <= 1'b0;
                if(tx_ctrl_done) begin
                    state <= OP_READ_7BIT_REPEAT_START;
                    start_gen_start <= 1'b1;
                end

                if(tx_ctrl_error) begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | TX_ERROR_MASK;
                end
            end

            /* Repeat Start Condition */
            OP_READ_7BIT_REPEAT_START: begin
                start_gen_start <= 1'b0;
                if(start_gen_done) begin
                    tx_ctrl_start <= 1'b1;
                    tx_ctrl_tx_data <= {slave_addr[6:0], 1'b1};
                    state <= OP_READ_7BIT_DEV_ADDR_2;
                end
            end

            /* Send slave addr again with R/W bit set to read */
            OP_READ_7BIT_DEV_ADDR_2: begin
                tx_ctrl_start <= 1'b0;
                if(tx_ctrl_done) begin
                    if(byte_cnt == 1) begin
                        rx_ctrl_send_nack <= 1'b1;
                    end else begin
                        rx_ctrl_send_nack <= 1'b0;
                    end
                    state <= OP_READ_7BIT_DATA_1;
                    rx_ctrl_start <= 1'b1;
                    byte_cnt <= byte_cnt - 1;
                end

                if(tx_ctrl_error) begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | TX_ERROR_MASK;
                end
            end

            /* Read Data */
            OP_READ_7BIT_DATA_1: begin
                rx_ctrl_start <= 1'b0;
                if(rx_ctrl_done) begin
                    o_rx_data_valid <= 1'b1;
                    state <= OP_READ_7BIT_DATA_2;
                end

                if(rx_ctrl_error) begin
                    state <= STOP;
                    o_rx_data_valid <= 1'b0;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | RX_ERROR_MASK;
                end
            end

            /* Read Data */
            OP_READ_7BIT_DATA_2: begin
                o_rx_data_valid <= 1'b0;
                byte_cnt <= byte_cnt - 1;
                if(byte_cnt != 0) begin
                    rx_ctrl_start <= 1'b1;
                    state <= OP_READ_7BIT_DATA_1;
                    if(byte_cnt == 1) begin
                        rx_ctrl_send_nack <= 1'b1;
                    end else begin
                        rx_ctrl_send_nack <= 1'b0;
                    end
                end else begin
                    state <= STOP;
                    stop_gen_start <= 1'b1;
                    o_status_reg <= o_status_reg | RX_DONE_MASK;
                end
            end

            default: state <= IDLE;
        endcase
    end
end



endmodule