/* Seven Segment Display Driver
    Maintains the state (cathode outputs) of single seven segment display.

    Cathods: CA-CG, DP
     _______________________
    |    |     CA     |    |
    |    |____________|    |
    | CF |            | CB |
    |    |            |    |
    |____|____________|____|
         |     CG     |
     ____|____________|_____
    |    |            |    |
    |    |            |    |    _______
    | CE |____________| CC |   |      |
    |    |     CD     |    |   |  DP  |
    |____|____________|____|   |______| 
    
    Note: Drive cathodes low for segment to illuminate.
    Output Vector Mapping: [CA, CB, CC, CD, CE, CF, CG, DP]
*/

module seven_segment_disp_driver (
    input wire clk,
    input wire rst,
    input wire [3:0] digit,
    input wire dot_en,
    output reg [7:0] mapping
);

always @(posedge clk or posedge rst) begin
    if(rst) begin
        mapping <= 8'b11111111;
    end else begin
        mapping[0] <= ~dot_en;
        case(digit)
            4'd0: mapping[7:1] <= 7'b0000001;
            4'd1: mapping[7:1] <= 7'b1001111; 
            4'd2: mapping[7:1] <= 7'b0010010;
            4'd3: mapping[7:1] <= 7'b0000110;
            4'd4: mapping[7:1] <= 7'b1001100;
            4'd5: mapping[7:1] <= 7'b0100100;
            4'd6: mapping[7:1] <= 7'b0100000;
            4'd7: mapping[7:1] <= 7'b0001111;
            4'd8: mapping[7:1] <= 7'b0000000;
            4'd9: mapping[7:1] <= 7'b0000100;
            default: mapping[7:1] <= 7'b1111111;
        endcase
    end
end
endmodule