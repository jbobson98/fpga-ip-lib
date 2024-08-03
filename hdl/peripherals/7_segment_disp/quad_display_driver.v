/* Quad Display Driver
    Drives 4 seven segment displays at a user definable refresh rate.
    Input number is parsed to get individual digits
    [Note: For deployment on BASYS3 development board]
*/

module quad_display #(
    parameter CLOCK_FREQ_HZ = 100_000_000,
    parameter REFRESH_RATE_HZ = 240 // 60Hz to 1KHz recommended
)(
    input wire clk_i,
    input wire rst_i,
    input wire disp_en_i,
    input wire [3:0] dot_enables_i,
    input wire [13:0] disp_num_i, // number to display (up to 9999)
    output reg [3:0] anodes_o,
    output reg [7:0] cathodes_o
);

localparam integer REFRESH_COUNTER_MAX = ((CLOCK_FREQ_HZ / REFRESH_RATE_HZ) - 1) / 4;
localparam integer COUNTER_BIT_WIDTH = $clog2(REFRESH_COUNTER_MAX + 1);

// Instantiate the refresh counter
wire next_display; // turn on leds for next display in cycle
wire [COUNTER_BIT_WIDTH-1 : 0] refresh_counter_val;
flex_counter #(.MAX_COUNT(REFRESH_COUNTER_MAX), .WIDTH(COUNTER_BIT_WIDTH)) refresh_counter 
    (.clk(clk_i), .rst(rst_i), .cen(disp_en_i), .maxcnt(next_display), .count(refresh_counter_val));

// Instantiate driver for each display
wire [3:0] digits [3:0]; // disp 3 = left, disp 0 = right
wire [7:0] cathodes [3:0];
seven_segment_disp_driver disp3 
    (.clk(clk_i), .rst(rst_i), .digit(digits[3]), .dot_en(dot_enables_i[3]), .mapping(cathodes[3]));
seven_segment_disp_driver disp2 
    (.clk(clk_i), .rst(rst_i), .digit(digits[2]), .dot_en(dot_enables_i[2]), .mapping(cathodes[2]));
seven_segment_disp_driver disp1 
    (.clk(clk_i), .rst(rst_i), .digit(digits[1]), .dot_en(dot_enables_i[1]), .mapping(cathodes[1]));
seven_segment_disp_driver disp0 
    (.clk(clk_i), .rst(rst_i), .digit(digits[0]), .dot_en(dot_enables_i[0]), .mapping(cathodes[0]));

// Calculate digits for segment inputs
wire bcd_done;
reg start_bcd;
reg [13:0] disp_num;
decimal_to_bcd #(.INPUT_WIDTH(14), .DECIMAL_DIGITS(4)) bcd_converter 
    (.clk_i(clk_i), .rst_i(rst_i), .start_i(start_bcd), .binary_i(disp_num), 
     .bcd_o({digits[3], digits[2], digits[1], digits[0]}), .done_o(bcd_done));

// Decimal to BCD, double double algorithm
reg converter_ready;
always @(posedge clk_i or posedge rst_i) begin
    if(rst_i) begin
        disp_num <= 0;
        converter_ready <= 1'b1;
        start_bcd <= 1'b1;
    end else begin
        disp_num <= disp_num_i;
        start_bcd <= 1'b0;

        if(disp_num != disp_num_i && converter_ready) begin
            start_bcd <= 1'b1;
            converter_ready <= 1'b0;
        end else if(bcd_done) begin
            converter_ready <= 1'b1;
        end
    end
end

// Handle refreshing
reg [1:0] cur_disp;
always @(posedge clk_i or posedge rst_i) begin
    if(rst_i) begin
        anodes_o <= 4'b1111;
        cathodes_o <= 8'b11111111;
        cur_disp <= 0;
    end else begin
        if(~disp_en_i || ~converter_ready) begin
            anodes_o <= 4'b1111;
            cathodes_o <= 8'b11111111;
            cur_disp <= 0;
        end else if(next_display) begin
            cur_disp <= cur_disp + 1;
            cathodes_o <= cathodes[cur_disp];
            anodes_o <= ~(4'b0001 << cur_disp);
        end
    end
end

endmodule