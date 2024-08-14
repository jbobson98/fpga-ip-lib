`timescale 1ns / 1ps

module i2c_rx_byte_controller (
    input wire i_clk,
    input wire i_rst,
    input wire i_tick,
    input wire i_rx_start, // might want to register this
    input wire i_scl,
    input wire i_sda,
    input wire i_send_nack, // if we are receiving the last byte send a nack to the slave
    output reg o_rx_done,   // rx data valid
    output reg o_rx_error,    
    output reg o_sda_disable, // only enable sda when we are sending ack/nack to slave
    output reg o_scl_disable,
    output reg o_sda,
    output reg o_scl,
    output reg [7:0] o_rx_data
);

/* Internal Registers */
localparam integer TOTAL_BITS = 8;
reg [1:0] step    = 0;
reg [3:0] state   = 0;  // 0 = idle, 1-8 rx bits, 9 = send ACK/NACK

/* Handle SDA and SCL Disable */
always @(*) begin
    if(step == 1 || step == 2) begin
        o_scl_disable = 1'b1;
    end else begin
        o_scl_disable = 1'b0;
    end

    if(state == 9) begin
        o_sda_disable = 1'b0;
    end else begin
        o_sda_disable = 1'b1;
    end
end

/* Receive State Machine */
always @(posedge i_clk) begin
    if(i_rst) begin
        step          <= 0;
        state         <= 0;
        o_sda         <= 1'b0;
        o_scl         <= 1'b0;
        o_rx_done     <= 1'b0;
        o_rx_error    <= 1'b0;
        o_rx_data     <= 0;
    end else begin
        if(state == 0) begin
            o_rx_done     <= 1'b0;
            o_rx_error    <= 1'b0;
            o_sda         <= 1'b1;
            o_scl         <= 1'b0;
            if(i_rx_start) begin
                step     <= 0;
                state    <= state + 1;
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
                                o_rx_data <= {o_rx_data[6:0], i_sda};
                                o_scl <= 1'b0;
                                step <= step + 1;
                            end
                    2'd3:   begin
                                state <= state + 1;
                                step <= step + 1;
                            end
                    default: ;
                endcase
            end
        end else if(state == 9) begin // Send ACK/NACK
            if(i_tick) begin
                case(step)
                    2'd0:   begin
                                o_scl <= 1'b1;
                                if(i_send_nack) begin
                                    o_sda <= 1'b1;
                                end else begin
                                    o_sda <= 1'b0;
                                end
                                step <= step + 1;
                            end
                    2'd1:   begin
                                if(i_scl) step <= step + 1; // Check for clock stretching
                            end
                    2'd2:   begin
                                o_scl <= 1'b0;
                                step <= step + 1;
                            end
                    2'd3:   begin
                                state <= 0;
                                step <= step + 1;
                                //o_sda <= 1'b1; // might not need this
                                o_rx_done <= 1'b1;
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