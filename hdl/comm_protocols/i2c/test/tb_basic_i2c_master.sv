/* time_unit = 1ns, time_precision = 100ps */
`timescale 1ns / 1ps

module tb_basic_i2c_master ();

/* Clock Parameters */
localparam CLOCK_FREQ_HZ = 100_000_000;
localparam CLK_PERIOD_NS = (1_000_000_000 / CLOCK_FREQ_HZ);
localparam FF_SETUP_TIME = 0.190;
localparam FF_HOLD_TIME  = 0.100;
localparam CHECK_DELAY   = (CLK_PERIOD_NS - FF_SETUP_TIME); // Check right before the setup time starts

/* Test Bench Signals */
integer tb_test_num;
string tb_test_case;
  
/* DUT Input Port Signals */
reg tb_clk;
reg tb_rst;
reg tb_read;
reg [9:0] tb_dev_addr;
reg [7:0] tb_wr_data;
reg [7:0] tb_byte_cnt;
reg [3:0] tb_control_reg;
reg [3:0] tb_mode_reg;
reg tb_rx_data_valid, tb_tx_data_needed;
reg [7:0] tb_rx_data;
reg [4:0] tb_status_reg;


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
i2c_master #(
    .SYS_CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),
    .FAST_MODE_ENABLED(0)
) DUT (
    .i_clk(tb_clk),
    .i_rst(tb_rst),
    .i_slave_addr(tb_dev_addr),
    .i_byte_cnt(tb_byte_cnt),
    .i_control_reg(tb_control_reg),
    .i_mode_reg(tb_mode_reg),
    .i_tx_data(tb_wr_data),
    .o_rx_data_valid(tb_rx_data_valid),
    .o_tx_data_needed(tb_tx_data_needed),
    .o_rx_data(tb_rx_data),
    .o_status_reg(tb_status_reg),
    .io_sda(tb_i2c_sda),
    .io_scl(tb_i2c_scl)
);

i2c_slave_dummy slave (
    .SCL(tb_i2c_scl),
    .SDA(tb_i2c_sda),
    .RST(tb_rst)
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
        tb_control_reg = 4'b0000;
        tb_mode_reg = 4'b0000;
        tb_dev_addr = 0;
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

    
    /* Basic Write -------------------------------------------------------------- */
    tb_test_num = tb_test_num + 1;
    @(posedge tb_clk);
    tb_dev_addr = 10'h55;
    tb_wr_data = 8'h03;
    tb_read = 1'b0;
    tb_byte_cnt = 8'd2;
    tb_control_reg = 4'b1000;
    @(posedge tb_clk);
    tb_control_reg = 4'b0000;
    @(posedge tb_tx_data_needed);
    tb_wr_data = 8'h57;




end


endmodule