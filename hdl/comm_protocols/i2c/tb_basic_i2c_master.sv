/* time_unit = 1ns, time_precision = 100ps */
`timescale 1ns / 1ps

module tb_basic_i2c_master ();

/* Clock Parameters */
localparam CLOCK_FREQ_HZ = 100_000_000;
localparam CLK_PERIOD_NS = (1_000_000_000 / CLOCK_FREQ_HZ);
localparam FF_SETUP_TIME = 0.190;
localparam FF_HOLD_TIME  = 0.100;
localparam CHECK_DELAY   = (CLK_PERIOD_NS - FF_SETUP_TIME); // Check right before the setup time starts

/* I2C Parameters */
localparam I2C_SCL_FREQ_HZ        = 10_000_000;
localparam I2C_DEV_ADDR_WDITH     = 7;
localparam I2C_DEV_REG_ADDR_WIDTH = 8;
localparam I2C_DATA_WIDTH         = 8; 


/* Test Bench Signals */
integer tb_test_num;
string tb_test_case;
  
/* DUT Input Port Signals */
reg tb_clk;
reg tb_rst;
reg tb_start_trans;
reg tb_read;
reg [I2C_DEV_ADDR_WDITH-1:0] tb_dev_addr;
reg [I2C_DEV_REG_ADDR_WIDTH-1:0] tb_dev_reg_addr;
reg [I2C_DATA_WIDTH-1:0] tb_wr_data;

/* DUT Output Port Signals */
reg [I2C_DATA_WIDTH-1:0] tb_read_data;
reg tb_busy;

/* DUT I2C Port Signals */
wire tb_i2c_sda;
reg i2c_sda_driver;
wire tb_i2c_scl;
reg i2c_scl_driver;
//assign tb_i2c_sda = i2c_sda_driver;
//assign tb_i2c_scl = i2c_scl_driver;
pullup(tb_i2c_sda);
pullup(tb_i2c_scl);

/* Instantiate DUT */
basic_i2c_master #(
    .SYS_CLOCK_FREQ(CLOCK_FREQ_HZ),
    .SCL_FREQ(I2C_SCL_FREQ_HZ),
    .DEV_ADDR_WIDTH(I2C_DEV_ADDR_WDITH),
    .DEV_REG_ADDR_WIDTH(I2C_DEV_REG_ADDR_WIDTH),
    .DATA_WIDTH(I2C_DATA_WIDTH)
) DUT (
    .clk_i(tb_clk),
    .rst_i(tb_rst),
    .start_trans_i(tb_start_trans),
    .read_i(tb_read),
    .dev_addr_i(tb_dev_addr),
    .dev_reg_addr_i(tb_dev_reg_addr),
    .wr_data_i(tb_wr_data),
    .read_data_o(tb_read_data),
    .busy_o(tb_busy),
    .i2c_serial_data(tb_i2c_sda),
    .i2c_serial_clk(tb_i2c_scl)
);

i2c_slave_dummy slave_dummy (
    .RST(tb_rst),
    .SCL(tb_i2c_scl),
    .SDA(tb_i2c_sda)
);


/* Generate DUT Clock */
always begin
    tb_clk = 1'b0;  // Start clock low to avoid false rising edge events at t=0
    #(CLK_PERIOD_NS/2.0); // half of clock period for 50% duty cycle
    tb_clk = 1'b1;
    #(CLK_PERIOD_NS/2.0);
end

/* Task: Resut DUT */
task reset_dut;
    begin
        tb_rst = 1'b1; // active the reset
        @(posedge tb_clk);
        @(posedge tb_clk); // assert reset for two positive clock edges
        @(negedge tb_clk); // Wait until safely away from rising edge of the clock before releasing
        tb_rst = 1'b0;
        @(negedge tb_clk);
        @(negedge tb_clk);  // Leave out of reset for a couple cycles before allowing other stimulus
    end
endtask

/* Task: Idle (all inputs 0) */
task i2c_idle;
    begin
        tb_start_trans = 1'b0;
        tb_dev_addr = 0;
        tb_dev_reg_addr = 0;
        tb_read = 0;
        tb_wr_data = 0;
        i2c_sda_driver = 1'bz;
        i2c_scl_driver = 1'b1;
    end
endtask


/* Test Bench Main Process */
initial begin

    /* Init DUT Inputs ---------------------------------------------------------- */
    $info("INIT TESTBENCH");
    tb_rst = 1'b0;
    i2c_idle();
    tb_test_num = 0;
    tb_test_case = "Test bench initializaton";
    #(0.1);

    /* Power-On Reset ----------------------------------------------------------- */
    $info("TEST: Power-on reset");
    tb_test_num = tb_test_num + 1;
    tb_test_case = "TEST: Power-on reset";
    #(0.1);
    tb_rst = 1'b1;
    #(CLK_PERIOD_NS * 0.5);
    // check signals here
    @(posedge tb_clk);
    #(2 * FF_HOLD_TIME);
    tb_rst = 1'b0;
    #(CHECK_DELAY);
    //check Signals here
    //assert(tb_rx_done == 1'b1);
    //assert(tb_rx_data_o == 8'b00000000);
    
    /* Basic Write -------------------------------------------------------------- */
    tb_test_num = tb_test_num + 1;
    @(posedge tb_clk);
    tb_dev_addr = 7'h55;
    tb_dev_reg_addr = 8'b10101010;
    tb_wr_data = 8'b11111111;
    tb_read = 1'b0;
    tb_start_trans = 1'b1;
    @(posedge tb_clk);
    tb_start_trans = 1'b0;




end


endmodule