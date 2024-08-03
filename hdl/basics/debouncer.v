/*
    Debounce an input button to avoid glitches.
    This also synchronizes the input pin (2FF).

    Size of N-bit Counter = ceil(log2(Tbounce/Tclk))
*/

module debouncer #(
    parameter CLK_FREQ = 100_000_000,    // Freq in Hz (default 100 MHz)
    parameter DEBOUNCE_MS = 20          // Debounce time in ms (Tbounce)
)(
    input wire clk,
    input wire rst,
    input wire btn_in,
    output reg btn_out
);


localparam integer DEBOUNCE_CYCLES = (CLK_FREQ / 1000) * DEBOUNCE_MS;
localparam integer COUNTER_WIDTH = $clog2(DEBOUNCE_CYCLES + 1);
reg [COUNTER_WIDTH-1 : 0] count;

// Synchronize the button input
wire synced_btn_in;
synchronizer sync (.clk(clk), .rst(rst), .async_in(btn_in), .sync_out(synced_btn_in));

always @(posedge clk or posedge rst) begin


    if(rst) begin
        count <= 0;
        btn_out <= 1'b0;
    end else begin
        if(synced_btn_in == btn_out) begin
            count <= 0;
        end else begin
            if(count == DEBOUNCE_CYCLES-1) begin
                btn_out <= synced_btn_in;
                count <= 0;
            end else begin
                count <= count + 1;
            end
        end
    end
end


endmodule