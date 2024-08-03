module flex_counter #(
    parameter MAX_COUNT = 255,
    parameter WIDTH = 8 
)(
    input wire clk,
    input wire rst,
    input wire cen,    // count enable
    output reg maxcnt, // MAX_COUNT reached
    output reg [WIDTH-1 : 0] count
);

always @(posedge clk or posedge rst) begin

    if(rst) begin
        count <= 0;
        maxcnt <= 1'b0;
    end else begin
        if(cen) begin
            if(count < MAX_COUNT) begin
                count <= count + 1;
                maxcnt <= 1'b0;
            end else begin
                count <= 0;
                maxcnt <= 1'b1;
            end
        end
    end
end

endmodule