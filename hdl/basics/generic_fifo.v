/*  Generic FIFO:
    - FIFO with configurable width and depth
    - If depth exceeds 32-word consider using BRAM
    - almost_full and almost_empty configurable based on percentage of fifo size
    - prefer inference fifo
    - First Word Fall Through: 
    - Almost_empty/full good for reading and writing in bursts
*/

module generic_fifo #(
    parameter WIDTH = 8, // width in bits of each element of the fifo
    parameter DEPTH = 8  // number of elements in the fifo
)(
    // Reset
    input wire rst_i,

    // Write Side
    input wire wr_clk_i,
    input wire wr_dv_i,  // write data valid
    input wire [WIDTH-1:0] wr_data_i,
    output reg wr_full_o,
    output reg wr_almost_full_o,

    // Read Side
    input wire rd_clk_i,
    input wire rd_en_i,
    output reg [WIDTH-1:0] rd_data_o,
    output reg rd_empty_o,
    output reg rd_almost_empty_o
);









endmodule