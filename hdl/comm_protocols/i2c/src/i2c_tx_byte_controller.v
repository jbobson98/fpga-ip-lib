`timescale 1ns / 1ps

module i2c_tx_byte_controller (
    input wire i_clk,
    input wire i_rst,
    input wire i_tick,
    input wire i_tx_start,
    input wire [7:0] i_tx_data,
    input wire i_scl,
    input wire i_sda,
    output reg o_tx_done,
    output reg o_tx_error,    // we don't get ACK
    output reg o_sda_disable, // only enable we are driving SDA or SCL, other check for clock stretching and ACK/NACK 
    output reg o_scl_disable,
    output reg o_sda,
    output reg o_scl
);

// check for clock stretching whenever master expects SCL line to be high

/* Internal Registers */
localparam integer TOTAL_BITS = 8;
reg [1:0] step    = 0;
reg [3:0] state   = 0;  // 0 = idle, 1-8 tx bits, 9 = ACK check
reg [7:0] tx_data = 0;
reg ack_recv      = 1'b0;

/* Handle SDA and SCL Disable */
always @(*) begin
    if(step == 1 || step == 2) begin
        o_scl_disable = 1'b1;
    end else begin
        o_scl_disable = 1'b0;
    end

    if(state == 9) begin
        o_sda_disable = 1'b1;
    end else begin
        o_sda_disable = 1'b0;
    end
end

/* Transmit State Machine */
always @(posedge i_clk) begin
    if(i_rst) begin
        step          <= 0;
        state         <= 0;
        o_sda         <= 1'b0;
        o_scl         <= 1'b0;
        o_tx_done     <= 1'b0;
        o_tx_error    <= 1'b0;
        ack_recv      <= 1'b0;
        tx_data       <= 1'b0;
    end else begin
        if(state == 0) begin
            o_tx_done     <= 1'b0;
            o_tx_error    <= 1'b0;
            o_sda         <= 1'b1;
            o_scl         <= 1'b0;
            ack_recv      <= 1'b0;
            if(i_tx_start) begin
                step     <= 0;
                tx_data  <= i_tx_data;
                state    <= state + 1;
                o_sda    <= i_tx_data[TOTAL_BITS-1];
            end
        end else if(state >= 1 && state <= 8) begin
            if(i_tick) begin
                case(step)
                    2'd0:   begin
                                o_scl <= 1'b1;
                                step <= step + 1;
                            end
                    2'd1:   if(i_scl) step <= step + 1; // Check for clock stretching
                    2'd2:   begin
                                o_scl <= 1'b0;
                                step <= step + 1;
                            end
                    2'd3:   begin
                                state <= state + 1;
                                step <= step + 1;
                                if(state < 8) o_sda <= tx_data[(TOTAL_BITS - state - 1) & 3'b111];
                            end
                    default: ;
                endcase
            end
        end else if(state == 9) begin
            if(i_tick) begin
                case(step)
                    2'd0:   begin
                                o_scl <= 1'b1;
                                step <= step + 1;
                            end
                    2'd1:   begin
                                if(i_scl) step <= step + 1; // Check for clock stretching
                            end
                    2'd2:   begin
                                if(~i_sda) ack_recv <= 1'b1;
                                o_scl <= 1'b0;
                                step <= step + 1;
                            end
                    2'd3:   begin
                                state <= 0;
                                step <= step + 1;
                                if(ack_recv) begin
                                    o_sda <= 1'b1;
                                    o_tx_done <= 1'b1;
                                end else begin
                                    o_tx_error <= 1'b1;
                                end
                            end
                    default: ;
                endcase
            end
        end else begin
            state <= 0;
        end
    end
end


endmodule
