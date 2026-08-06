`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.08.2026 13:33:45
// Design Name: 
// Module Name: pi_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module pi_controller#(
    // Boundary values for wind up
    parameter signed [23:0] MAX_OUTPUT = 24'sh7FFFFF, // +2^23 - 1
    parameter signed [23:0] MIN_OUTPUT = 24'sh800000   // -2^23
)
(
    input clk, reset,
    input signed [11:0] reference,
    input signed [11:0] feedback,
    output reg signed [23:0] control_signal
);

wire signed [11:0] error       = reference - feedback;
wire signed [11:0] kp_constant = (error > 12'sd500 || error < -12'sd500) ? 12'sd128 : 12'sd32;
wire signed [11:0] ki_constant = (error > 12'sd500 || error < -12'sd500) ? 12'sd4   : 12'sd1;
wire signed [23:0] p           = error * kp_constant;
wire signed [23:0] delta_i     = error * ki_constant;
reg signed [23:0] accumulator;
wire signed [24:0] control_sum = accumulator + p;




always @(posedge clk) begin
    if (reset) begin
       accumulator    <= 24'sd0; end
    else begin
        // Output
        if (control_sum > MAX_OUTPUT) begin
                control_signal <= MAX_OUTPUT;
                accumulator    <= MAX_OUTPUT - p;
        end else if (control_sum < MIN_OUTPUT) begin
            control_signal <= MIN_OUTPUT;
            accumulator    <= MIN_OUTPUT - p;
        end else begin
            control_signal <= control_sum[23:0];
            accumulator    <= accumulator + delta_i;
        end
end
end
endmodule
