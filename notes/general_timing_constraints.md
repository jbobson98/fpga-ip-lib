# General Timing Constraints #

### Creating a Clock in XDC File ###
Below is an example of creating a 100MHz clock in the contraint file. All timing contraints in Vivado xdc files are in nanoseconds. (Note: use Vivado Language Templates for examples of all xdc contraints)

`set_property PACKAGE_PIN E3 [get_ports {clk}]`\
`create_clock -name sysclk -period 10 [get_ports {clk}]`\

Use `set_clock_groups` to define clocks that are synchronous to eachother. Synchronous clock are clocks that have a known phase relationship, usually derived from the same primary clock. See documentation for full usage.

### Input/Output Delay Constraints ###
For asynchronous IO (slow IO relative to FPGA clock) like LEDs, buttons, SPI, and I2C the IO delay does not really matter. Therefore, the constraint on these pins can be ignored (`set_false_path`).

This is different for synchronous ouputs like when an external IC expects data valid 2ns before rising clock edge. When both the clock and data are output from the FPGA, it is called "source synchronous". Use the output delay to make sure the FPGA outputs the data early enough so that it arrives at the external IC > 2 ns before the next clock edge arrives (`set_output_delay`).

With a synchronous input, you may be outputting the clock, some time later that arrives at the external IC, which triggers a data transmission, so some time later that data arrives back at the FPGA. The FPGA needs to know how long that process takes, so that it can make sure that the delay internal to the FPGA is small enough so that the data gets to the register before the next clock edge (`set_input_delay`).

If the I/O is synchronous and is slower than 1MHz, you can probably get away with not having the constraints. Adding the constraints will guarentee the design will work so it's best practice to add them.

