/*  ------------------------------------------------
    Module:     basic_i2c_master
    Developer:  Jacob Bobson
    Date:       8/6/2024
    ------------------------------------------------

    I2C Notes:
        Speeds:
            standard = 100kbps
            fast = 400kbps
            high speed = 3.4 Mbps
            ultra fast = 5 Mbps

    I2C Message:
        [ START | ADDRESS | READ/WRITE | ACK/NACK | DATAFRAME1 | ACK/NACK | DATAFRAME2 | ACK/NACK | STOP ]

        START      = SDA switches from hight to low before the SCL line switches from high to low
        ADDRESS    = 7 or 10 bit sequence unique to each slave (slave id)
        READ/WRITE = Single bit that indicates if master is reading(1) from or writing(0) to the slave
        ACK/NACK   = receiving device will notify sender that a frame has been received (ACK=SDA pulled low)
        STOP       = SDA switches from low to high after SCL switches from low to high
        DATAFRAME  = 8 bits long

    ADDRESS DATA:
        - Sent MSB first

    NORMAL DATA:
        - each byte is sent MSB first
        - usually if data being written is more than one byte, the most significant byte is sent first

    Misc:
        - should debounce (glitch filtering) SDA/SCL see https://support.xilinx.com/s/article/61861?language=en_US
        - SDA must be pulled up to VCC with a 5.6-kilo-ohm resistor and SCL also needs to be pulled up with the same 
            resistor only if there are multiple masters in the system or when the slave will perform clock stretching 
            as a flow control measure to synchronise with the master.
        - If ack not received, send stop command
        - Connect this with a data fifo that stores commands (same on the read side)

    Write Procedure States:
        IDLE
        SEND_START
        SEND_DEV_ADDR
        SEND_RW_BIT (0)
        CHECK_FOR_ACK
        SEND_REGISTER_ADDR
        CHECK_FOR_ACK
        SEND_DATA_BYTE_1
        CHECK_FOR_ACK
              .
              .
              .
        SEND_DATA_BYE_N
        CHECK_FOR_ACK
        SEND_STOP

    Read Procedure States
        IDLE
        SEND_START
        SEND_DEV_ADDR
        SEND_RW_BIT (0)
        CHECK_FOR_ACK
        SEND_REGISTER_ADDR
        CHECK_FOR_ACK
        SEND_RESTART (basically repeat START condition)
        SEND_DEV_ADDR
        SEND_RW_BIT (1)
        CHECK_FOR_ACK
        READ_REGISTER_1
        CHECK_FOR_ACK
              .
              .
              .    
        READ_REGISTER_N
        CHECK_FOR_ACK
        SEND_STOP
*/

module basic_i2c_master #(
    parameter SYS_CLOCK_FREQ     = 100_000_000,  // (Hz) Frequency of clk_i 
    parameter SCL_FREQ           = 100_000,      // (Hz) Frequency of I2C SCL, 110-150 KHz to achieve standard speed
    parameter DEV_ADDR_WIDTH     = 7,            // Number of bits used for device address (7 or 10)
    parameter DEV_REG_ADDR_WIDTH = 8,            // Width of device register address
    parameter DATA_WIDTH         = 8             // Width of data to write at address (8/16)
)(

    /* Control Signals */
    input wire clk_i,
    input wire rst_i,
    input wire start_trans_i,               // Start a transaction if not currently busy
    input wire read_i,                      // '1' = read from address, '0' = write to address (FIRST TEST WRITES, THEN MOVE ON TO READS)
    input wire [DEV_ADDR_WIDTH-1:0] dev_addr_i,
    input wire [DEV_REG_ADDR_WIDTH-1:0] dev_reg_addr_i,
    input wire [DATA_WIDTH-1:0] wr_data_i,

    /* Output Status Signals */
    output reg [DATA_WIDTH-1:0] read_data_o,
    output reg busy_o,

    /* External I2C Signals */
    inout wire i2c_serial_data,
    inout wire i2c_serial_clk
);

/* Generate Serial Clock Signals */
localparam integer TICK_GENERATOR_MAX   = (SYS_CLOCK_FREQ / (4*SCL_FREQ)) - 1;
localparam integer TICK_GENERATOR_WIDTH = $clog2(TICK_GENERATOR_MAX + 1);
wire tick, tick_en;
assign tick_en = 1'b1; // JMB: might want to change this later
wire [TICK_GENERATOR_WIDTH-1 : 0] tick_generator_count;
flex_counter #(.MAX_COUNT(TICK_GENERATOR_MAX), .WIDTH(TICK_GENERATOR_WIDTH)) tick_generator 
    (.clk(clk_i), .rst(rst_i), .cen(tick_en), .maxcnt(tick), .count(tick_generator_count));

/* Handle Tri-State of I2C Signals */
reg serial_data_out_en, serial_clk_out_en;
reg scl = 1'b1;
reg sda = 1'b1;
assign i2c_serial_data = (serial_data_out_en) ? sda : 1'bz;
assign i2c_serial_clk  = (serial_clk_out_en) ? scl : 1'bz;

/* Address and Data Registers */
reg [DEV_ADDR_WIDTH:0]          dev_addr  = 0; // bit 0 will be read/write bit
reg [DEV_REG_ADDR_WIDTH-1:0]    reg_addr  = 0;
reg [DATA_WIDTH-1:0]            wr_data   = 0;


localparam IDLE               = 4'd0;
localparam SEND_START         = 4'd1;
localparam SEND_DEV_ADDR      = 4'd2;
localparam CHECK_FOR_ACK      = 4'd3;
localparam SEND_REGISTER_ADDR = 4'd4;
localparam SEND_DATA_BYTE     = 4'd5;
localparam SEND_STOP          = 4'd6;
localparam SEND_RESTART       = 4'd7;
localparam READ_REGISTER      = 4'd8;
localparam SEND_NACK          = 4'd9;


/* Control Registers */
reg [3:0] state   = IDLE; 
reg [1:0] step    = 0;
reg [3:0] bit_cnt = 0;
reg ack_recv      = 1'b0;

/* I2C Line Control */
always @(*) begin
    if(state != IDLE && state != CHECK_FOR_ACK && state != READ_REGISTER) begin
        serial_data_out_en = 1'b1;
    end else begin
        serial_data_out_en = 1'b0;
    end

    if(state != IDLE && step != 1 && step != 2) begin
        serial_clk_out_en = 1'b1;
    end else begin
        serial_clk_out_en = 1'b0;
    end
end

/* I2C Controller State Machine */
always @(posedge clk_i or posedge rst_i) begin

    if(rst_i) begin
        state <= IDLE;
        step <= 0;
    end else begin
        
        case(state)

            /* IDLE --------------------------------------------------------------------- */
            IDLE:   begin
                        step <= 0;
                        read_data_o <= 0;
                        if(start_trans_i) begin
                            state <= SEND_START;
                            dev_addr <= {dev_addr_i, read_i};
                            reg_addr <= dev_reg_addr_i;
                            wr_data <= wr_data_i;
                            busy_o <= 1'b1;
                        end else begin
                            busy_o <= 1'b0;
                        end
                    end

            /* SEND_START --------------------------------------------------------------- */
            SEND_START: begin
                            if(tick) begin
                                case(step)
                                    2'd0:   ;
                                    2'd1:   sda <= 1'b0;       
                                    2'd2:   bit_cnt <= 0;
                                    2'd3:   begin
                                                scl <= 1'b0;
                                                sda <= dev_addr[DEV_ADDR_WIDTH];
                                                state <= SEND_DEV_ADDR;
                                            end
                                    default: ;
                                endcase
                                step <= step + 1;
                            end
                        end

            /* SEND_DEV_ADDR ------------------------------------------------------------ */
            SEND_DEV_ADDR:  begin
                                if(tick) begin
                                    case(step)
                                        2'd0:   begin
                                                    scl <= 1'b1;
                                                    step <= step + 1;
                                                end
                                        2'd1:   if(i2c_serial_clk) step <= step + 1; // handle clock stretching
                                        2'd2:   begin
                                                    scl <= 1'b0;
                                                    bit_cnt <= bit_cnt + 1;
                                                    step <= step + 1;
                                                end
                                        2'd3:   begin
                                                    if(bit_cnt == DEV_ADDR_WIDTH + 1) begin
                                                        state <= CHECK_FOR_ACK;
                                                    end else begin
                                                        sda <= dev_addr[DEV_ADDR_WIDTH - bit_cnt];
                                                    end
                                                    step <= step + 1;
                                                end
                                        default: ;
                                    endcase
                                end
                            end

            /* CHECK_FOR_ACK ------------------------------------------------------------ */
            CHECK_FOR_ACK:  begin
                                if(tick) begin
                                    case(step)
                                        2'd0:   begin
                                                    scl <= 1'b1;
                                                    step <= step + 1;
                                                end
                                        2'd1:   begin
                                                    if(i2c_serial_clk) step <= step + 1; // handle clock stretching
                                                    ack_recv <= 1'b0;
                                                end
                                        2'd2:   begin
                                                    if(~i2c_serial_data) ack_recv <= 1'b1;
                                                    step <= step + 1;
                                                    scl <= 1'b0;
                                                end
                                        2'd3:   begin
                                                    step <= step + 1;
                                                    if(ack_recv) begin
                                                        sda <= 1'b1;
                                                        state <= SEND_REGISTER_ADDR;
                                                    end else begin
                                                        state <= SEND_STOP;
                                                    end
                                                end
                                        default: ;
                                    endcase
                                end
                            end

            /* SEND_REGISTER_ADDR ------------------------------------------------------- */
            /*
            SEND_REGISTER_ADDR: begin
                                    if(tick) begin
                                        case(step)
                                            2'd0:   begin
                                                        scl <= 1'b1;
                                                        step <= step + 1;
                                                    end
                                            2'd1:   if(i2c_serial_clk) step <= step + 1; // handle clock stretching
                                            2'd2:   begin
                                                        scl <= 1'b0;
                                                        bit_cnt <= bit_cnt + 1;
                                                        step <= step + 1;
                                                    end
                                            2'd3:   begin
                                                        if(bit_cnt == DEV_ADDR_WIDTH + 1) begin
                                                            state <= CHECK_FOR_ACK;
                                                        end else begin
                                                            sda <= dev_addr[DEV_ADDR_WIDTH - bit_cnt];
                                                        end
                                                        step <= step + 1;
                                                    end
                                            default: ;
                                        endcase
                                    end
                                end
                */

            default: state <= IDLE;
        endcase
    end
end







endmodule
