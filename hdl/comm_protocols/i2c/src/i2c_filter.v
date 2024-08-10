`timescale 1ns / 1ps

module i2c_filter (
    input wire i_clk,
    input wire i_rst,
    input wire i_in,
    output wire o_out
);

reg ff1,ff2,ff3,out_inv;
assign o_out = ~out_inv;

always @(posedge i_clk) begin
    if(i_rst) begin
        ff1 <= 1'b0;
        ff2 <= 1'b0;
        ff3 <= 1'b0;
    end else begin
        ff1 <= ~i_in;
        ff2 <= ff1;
        ff3 <= ff2;
    end
end

always @(posedge i_clk) begin
    if(i_rst) begin
        out_inv <= 1'b0;
    end else begin
        if(ff3 == ff2) out_inv <= ff3;
    end
end

endmodule