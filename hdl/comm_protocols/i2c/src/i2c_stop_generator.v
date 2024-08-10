`timescale 1ns / 1ps

module i2c_stop_generator (
    input wire i_clk,
    input wire i_rst,
    input wire i_tick,
    input wire i_start,
    output reg o_sda,
    output reg o_scl,
    output reg o_done
);

localparam IDLE  = 3'b000;     
localparam STEP1 = 3'b001;     
localparam STEP2 = 3'b010;
localparam STEP3 = 3'b011;
localparam STEP4 = 3'b100;
localparam STEP5 = 3'b101;
reg[2:0] state;

always @(*) begin
    o_done = 1'b0;
    o_scl  = 1'b0;
    o_sda  = 1'b0;
    case(state)
        STEP2:  begin
                    o_sda = 1'b0;
                    o_scl = 1'b0;
                end
        STEP3:  begin
                    o_sda = 1'b0;
                    o_scl = 1'b1;
                end
        STEP4:  begin
                    o_sda  = 1'b1;
                    o_scl  = 1'b1;
                    o_done = 1'b1;
                end
        STEP5:  begin
                    o_done = 1'b0;
                end
        default: ;
    endcase
end

always @(posedge i_clk) begin
    if(i_rst) begin
        state  <= IDLE;
    end else begin
        case(state)
            IDLE:   if(i_start) state <= STEP1;
            STEP1:  if(i_tick) state <= STEP2;
            STEP2:  if(i_tick) state <= STEP3;
            STEP3:  if(i_tick) state <= STEP4;
            STEP4:  state <= STEP5;
            STEP5:  state <= IDLE;
            default: state <= IDLE;
        endcase
    end
end

endmodule