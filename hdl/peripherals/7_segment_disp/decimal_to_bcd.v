/* Decimal to BCD Converter
    Uses the Double Dabble algorithm to convert decimal values
    into BCD digits.
*/

module decimal_to_bcd #(
    parameter INPUT_WIDTH = 8,
    parameter DECIMAL_DIGITS = 3
)(
    input wire clk_i,
    input wire rst_i,
    input wire start_i,
    input wire [INPUT_WIDTH-1 : 0] binary_i,
    output reg [DECIMAL_DIGITS*4-1 : 0] bcd_o,
    output reg done_o
);

// Define states
localparam IDLE              = 3'b000;
localparam SHIFT             = 3'b001;
localparam CHECK_SHIFT_IDX   = 3'b010;
localparam ADD               = 3'b011;
localparam CHECK_DIGIT_IDX   = 3'b100;
localparam DONE              = 3'b101;

// State regs
localparam integer LOOP_COUNTER_WIDTH = $clog2(INPUT_WIDTH + 1);  //num loops = input width
localparam integer DIGIT_COUNTER_WIDTH = $clog2(DECIMAL_DIGITS + 1);
reg [2:0] cur_state = IDLE; 
reg [INPUT_WIDTH-1 : 0] binary_val = 0;
reg [DIGIT_COUNTER_WIDTH-1 : 0] digit_idx = 0;
reg [LOOP_COUNTER_WIDTH-1 : 0] loop_count = 0;


always @(posedge clk_i or posedge rst_i) begin
    if(rst_i) begin
        cur_state <= IDLE;
        bcd_o <= 0;
        done_o <= 1'b0;
        binary_val <= 0;
        digit_idx <= 0;
        loop_count <= 0;
    end else begin
        case(cur_state)

            IDLE: 
                begin
                    done_o <= 1'b0;
                    if(start_i == 1'b1) begin
                        binary_val <= binary_i;
                        bcd_o      <= 0;
                        cur_state  <= SHIFT;
                    end 
                end
      
            SHIFT: // shift msb of binary input into bcd lsb, shift all bits of binary input through
                begin
                    bcd_o      <= bcd_o << 1; 
                    bcd_o[0]   <= binary_val[INPUT_WIDTH-1];
                    binary_val <= binary_val << 1;
                    cur_state  <= CHECK_SHIFT_IDX;
                end

            CHECK_SHIFT_IDX:
                begin
                    if(loop_count == INPUT_WIDTH-1) begin
                        loop_count <= 0;
                        cur_state  <= DONE;
                    end else begin
                        loop_count <= loop_count + 1;
                        cur_state  <= ADD;
                    end
                end
            
            ADD:
                begin
                    if(bcd_o[digit_idx*4 +: 4] > 4) begin
                        bcd_o[(digit_idx*4) +: 4] <= bcd_o[digit_idx*4 +: 4] + 3;
                    end
                    cur_state <= CHECK_DIGIT_IDX;
                end

            CHECK_DIGIT_IDX:
                begin
                    if(digit_idx == DECIMAL_DIGITS-1) begin
                        digit_idx <= 0;
                        cur_state <= SHIFT;
                    end else begin
                        digit_idx <= digit_idx + 1;
                        cur_state <= ADD;
                    end
                end
            
            DONE:
                begin
                    done_o    <= 1'b1;
                    cur_state <= IDLE;
                end
            
            default: cur_state <= IDLE;

        endcase
    end
end




endmodule