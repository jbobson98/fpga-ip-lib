/*
    Handles the generation of required signals for a VGA display.
*/

module vga_core (
    input wire clk,
    input wire rst,
    input wire [11:0] color_in,     // [4'bRED, 4'bGreen, 4'bBlue] 
    output wire hsync,
    output wire vsync,
    output wire [11:0] color_o,     // [4'bRED, 4'bGreen, 4'bBlue]
    output wire [15:0] x_coord,     // current pixel x coordinate (16bits should cover all valid vga resolutions)
    output wire [15:0] y_coord      // current pixel y coordinate
);


// VGA resolution parameters (1280x720 @60Hz, requres 75MHz clock)
localparam integer HORZ_PIXELS = 1280;
localparam integer HORZ_FRONT_PORCH = 110;
localparam integer HORZ_BACK_PORCH = 220;
localparam integer HORZ_SYNC = 40;
localparam integer TOTAL_WIDTH = HORZ_PIXELS + HORZ_FRONT_PORCH + HORZ_BACK_PORCH + HORZ_SYNC;
localparam integer WIDTH_BITS = $clog2(TOTAL_WIDTH + 1);

localparam integer VERT_PIXELS = 720;
localparam integer VERT_FRONT_PORCH = 5;
localparam integer VERT_BACK_PORCH = 20;
localparam integer VERT_SYNC = 5;
localparam integer TOTAL_HEIGHT = VERT_PIXELS + VERT_FRONT_PORCH + VERT_BACK_PORCH + VERT_SYNC;
localparam integer HEIGHT_BITS = $clog2(TOTAL_HEIGHT + 1);


// Instantiate vga_sync_generator
wire [WIDTH_BITS-1 : 0] x_loc;
wire [HEIGHT_BITS-1 : 0] y_loc;
wire video_active;
vga_sync_generator #(
    .HORZ_PIXELS(HORZ_PIXELS),
    .HORZ_FRONT_PORCH(HORZ_FRONT_PORCH),
    .HORZ_BACK_PORCH(HORZ_BACK_PORCH),
    .HORZ_SYNC(HORZ_SYNC),
    .VERT_PIXELS(VERT_PIXELS),
    .VERT_FRONT_PORCH(VERT_FRONT_PORCH),
    .VERT_BACK_PORCH(VERT_BACK_PORCH),
    .VERT_SYNC(VERT_SYNC)
) sync_gen (
    .clk(clk), 
    .rst(rst), 
    .hsync(hsync), 
    .vsync(vsync),
    .video_active(video_active),
    .x_loc(x_loc),
    .y_loc(y_loc)
);

assign color_o = (video_active) ? color_in : 12'b000000000000;
assign x_coord = x_loc;
assign y_coord = y_loc;



endmodule