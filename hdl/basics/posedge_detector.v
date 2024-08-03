module posedge_detector (
    input wire clk,
    input wire signal,
    output wire edge_pulse
);

reg sigdelay;
always @(posedge clk) begin
    sigdelay <= signal;
end

assign edge_pulse = signal & ~sigdelay;

endmodule