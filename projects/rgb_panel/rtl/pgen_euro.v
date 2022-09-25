/*
 * pgen.v
 *
 * vim: ts=4 sw=4
 *
 * Copyright (C) 2019  Sylvain Munaut <tnt@246tNt.com>
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`default_nettype none

module bram (
        input clk_r, clk_w, wen, ren,
        input [11:0] waddr,
        input [11:0] raddr,
        input [7:0] wdata,
        output reg [7:0] rdata
);
        reg [7:0] mem [0:4095];
        initial mem[0] <= 255;
        always @(posedge clk_w) begin
                if (wen)
                        mem[waddr] <= wdata;
        end
        always @(posedge clk_r) begin
                if (ren)
                        rdata <= mem[raddr];
        end
endmodule

module pgen_euro #(
	parameter integer N_ROWS   = 64,	// # of rows (must be power of 2!!!)
	parameter integer N_COLS   = 64,	// # of columns
	parameter integer BITDEPTH = 24,

	// Auto-set
	parameter integer LOG_N_ROWS  = $clog2(N_ROWS),
	parameter integer LOG_N_COLS  = $clog2(N_COLS)
)(
	// Frame Buffer write interface
	output wire [LOG_N_ROWS-1:0] fbw_row_addr,
	output wire fbw_row_store,
	input  wire fbw_row_rdy,
	output wire fbw_row_swap,

	output wire [BITDEPTH-1:0] fbw_data,
	output wire [LOG_N_COLS-1:0] fbw_col_addr,
	output wire fbw_wren,

	output wire frame_swap,
	input  wire frame_rdy,

    input wire [15:0] sample0,
    input wire [15:0] sample1,
    input wire [15:0] sample2,
    input wire [15:0] sample3,
    input wire sample_clk,

	// Clock / Reset
	input  wire clk,
	input  wire rst
);

	// Signals
	// -------

	// FSM
	localparam
		ST_WAIT_FRAME	= 0,
		ST_GEN_ROW		= 1,
		ST_WRITE_ROW	= 2,
		ST_WAIT_ROW		= 3;

	reg  [2:0] fsm_state;
	reg  [2:0] fsm_state_next;

	// Counters
	reg [11:0] frame;
	reg [LOG_N_ROWS-1:0] cnt_row;
	reg [LOG_N_COLS-1:0] cnt_col;
	reg cnt_row_last;
	reg cnt_col_last;

	// Output
	wire [7:0] color [0:2];


	// FSM
	// ---

	// State register
	always @(posedge clk or posedge rst)
		if (rst)
			fsm_state <= ST_WAIT_FRAME;
		else
			fsm_state <= fsm_state_next;

	// Next-State logic
	always @(*)
	begin
		// Default is not to move
		fsm_state_next = fsm_state;

		// Transitions ?
		case (fsm_state)
			ST_WAIT_FRAME:
				if (frame_rdy)
					fsm_state_next = ST_GEN_ROW;

			ST_GEN_ROW:
				if (cnt_col_last)
					fsm_state_next = ST_WRITE_ROW;

			ST_WRITE_ROW:
				if (fbw_row_rdy)
					fsm_state_next = cnt_row_last ? ST_WAIT_ROW : ST_GEN_ROW;

			ST_WAIT_ROW:
				if (fbw_row_rdy)
					fsm_state_next = ST_WAIT_FRAME;
		endcase
	end


	// Counters
	// --------

	// Frame counter
	always @(posedge clk or posedge rst)
		if (rst)
			frame <= 0;
		else if ((fsm_state == ST_WAIT_ROW) && fbw_row_rdy)
			frame <= frame + 1;

	// Row counter
	always @(posedge clk)
		if (fsm_state == ST_WAIT_FRAME) begin
			cnt_row <= 0;
			cnt_row_last <= 1'b0;
		end else if ((fsm_state == ST_WRITE_ROW) && fbw_row_rdy) begin
			cnt_row <= cnt_row + 1;
			cnt_row_last <= cnt_row == ((1 << LOG_N_ROWS) - 2);
		end

	// Column counter
	always @(posedge clk)
		if (fsm_state != ST_GEN_ROW) begin
			cnt_col <= 0;
			cnt_col_last <= 0;
		end else begin
			cnt_col <= cnt_col + 1;
			cnt_col_last <= cnt_col == (N_COLS - 2);
		end


	// Front-Buffer write
	// ------------------


    reg [15:0] sample0_latched;
    reg [15:0] sample1_latched;
    reg [15:0] sample2_latched;
    reg signed [15:0] sample3_latched;
    reg [12:0] slowdec = 13'h0;

    always @(posedge sample_clk) begin
        sample0_latched <= sample0 + 16'h8000;
        sample1_latched <= sample1 + 16'h8000;
        sample2_latched <= sample2 + 16'h8000;
        sample3_latched <= sample3;
        slowdec <= slowdec + 1;
    end

    wire [11:0] cur_px_r = cnt_row*64 + cnt_col;
    wire persist = sample3_latched > 4*2000;
    wire [11:0] cur_px_w =  persist ?
                             (64*sample0_latched[15:10] + sample1_latched[15:10]) :
                             slowdec[11:0];

    wire one = 1'b1;

    wire [7:0] color_bits = sample2_latched[15:8];
    wire [7:0] fbuf_wdata = persist ? color_bits : 8'h0;
    wire [7:0] fbuf_rdata;

    bram fbuf_bram(
        .clk_r(clk),
        .clk_w(sample_clk),
        .wen(one),
        .ren(one),
        .waddr(cur_px_w),
        .raddr(cur_px_r),
        .wdata(fbuf_wdata),
        .rdata(fbuf_rdata)
    );

	assign color[0] = {fbuf_rdata[7], fbuf_rdata[4], 6'h0};
	assign color[1] = {fbuf_rdata[6], fbuf_rdata[3], 6'h0};
	assign color[2] = {fbuf_rdata[5], fbuf_rdata[2], 6'h0};

	// Write enable and address
	assign fbw_wren = fsm_state == ST_GEN_ROW;
	assign fbw_col_addr = cnt_col;

	// Map to color
	generate
		if (BITDEPTH == 8)
			assign fbw_data = { color[2][7:5], color[1][7:5], color[0][7:6] };
		else if (BITDEPTH == 16)
			assign fbw_data = { color[2][7:3], color[1][7:2], color[0][7:3] };
		else if (BITDEPTH == 24)
			assign fbw_data = { color[2], color[1], color[0] };
	endgenerate


	// Back-Buffer store
	// -----------------

	assign fbw_row_addr  = cnt_row;
	assign fbw_row_store = (fsm_state == ST_WRITE_ROW) && fbw_row_rdy;
	assign fbw_row_swap  = (fsm_state == ST_WRITE_ROW) && fbw_row_rdy;


	// Next frame
	// ----------

	assign frame_swap = (fsm_state == ST_WAIT_ROW) && fbw_row_rdy;

endmodule // pgen_euro
