/*
    Classic 2 D flip-flop synchronizer.
*/

module synchronizer (
    input wire clk,
    input wire rst,
    input wire async_in,
    output wire sync_out
);

reg ff1, ff2;

assign sync_out = ff2;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ff1 <= 1'b0;
        ff2 <= 1'b0;
    end else begin
        ff1 <= async_in;
        ff2 <= ff1;
    end
end


endmodule