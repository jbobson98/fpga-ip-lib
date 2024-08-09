module i2c_start_generator (
    input wire i_clk,
    input wire i_rst,
    input wire i_tick,
    input wire i_start,
    input wire i_scl,
    input wire i_sda,
    output reg o_start_done,
    output reg o_sda,
    output reg o_scl
);


localparam IDLE = 1'b0;
localparam GEN  = 1'b1;

reg[1:0] step = 0;
reg state = IDLE;


always @(posedge i_clk or posedge i_rst) begin
    if(i_rst) begin
        state <= IDLE;
    end else begin
        if(state == IDLE) begin
            o_sda <= 1'b1;
            o_scl <= 1'b1;
            o_start_done <= 1'b0;
            if(i_start) state <= GEN;
        end else if(state == GEN && i_tick) begin
            case(step)
                2'd0:   ;
                2'd1:   o_sda <= 1'b0;       
                2'd2:   ;
                2'd3:   begin
                            o_scl <= 1'b0;
                            state <= IDLE;
                            o_start_done <= 1'b1;
                        end
                default: ;
            endcase
            step <= step + 1;
        end else begin
            state <= IDLE;
        end
    end
end


endmodule