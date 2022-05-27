// ========== Copyright Header Begin ============================================
// Copyright (c) 2020 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ========== Copyright Header End ============================================

//==================================================================================================
//  Filename      : chunk_req.v
//  Created On    : 2020-12-12
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Chunk requester
//
//==================================================================================================
`include "dcp.h"

module chunk_req 
#(
    // Configuration Parameters
    parameter ADDR = 32,
    parameter FIFO_IDX = 1,
    parameter CHUNK_IDX = 4,
    parameter ARRAY_IDX = 32,
    parameter CR_CHUNK_SIZE = 64*8
)
(
    // Clock + Reset
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          invalidate,

    input  wire                          cr_conf_val,
    input  wire [ADDR      -1:0]         cr_conf_addr,
    input  wire [FIFO_IDX    :0]         cr_conf_fifos,

    input  wire                          cr_add_val,
    input  wire [FIFO_IDX     -1:0]      cr_add_fifo, 
    input  wire [ARRAY_IDX    -1:0]      cr_add_begin,
    input  wire [ARRAY_IDX    -1:0]      cr_add_end,

    output wire                          cr_pipe_val,
    input  wire                          cr_pipe_rdy,
    output wire [63:0]                   cr_pipe_data,
    
    output wire                          cr_active,
    // Data request interface
    output  wire                          cr_req_val,
    input   wire                          cr_req_rdy,
    output  wire  [CHUNK_IDX   -1:0]      cr_req_idx,
    output  wire  [ADDR        -1:0]      cr_req_addr,

    // Data fill interface
    input  wire                          cr_fill_val,
    input  wire  [CHUNK_IDX       -1:0]  cr_fill_idx,
    input  wire  [CR_CHUNK_SIZE  -1:0]  cr_fill_data
);

//==============================================================================
// Local Parameters
//==============================================================================

genvar j;
localparam INFLIGHT_IDX = 2;
localparam INFLIGHT = 2**INFLIGHT_IDX;

localparam CHUNK_ST_SIZE = 2;
localparam CHUNK_ST_INV = 2'b00;
localparam CHUNK_ST_PEND = 2'b01;
localparam CHUNK_ST_VAL = 2'b11;

localparam CHUNKS = 2**CHUNK_IDX;
localparam CHUNK_ALIGN = $clog2(CR_CHUNK_SIZE/8);
localparam WORD_ALIGN = 3; //32bit is 2^2 bytes and 64 bit is 2^3 bytes
localparam WORD_IDX = CHUNK_ALIGN-WORD_ALIGN;
localparam WORDS = 2**WORD_IDX;

localparam FIFO_SIZE = 2**FIFO_IDX;


/////////////////////////////
// Input control structure //
/////////////////////////////
// Register declarations per fifo
reg conf_A_val_r;
reg [ADDR-1:0] conf_A_r;
reg [FIFO_IDX:0] conf_fifos_r;

reg [FIFO_SIZE-1:0] fifo_entry_val_r;
reg [CHUNK_IDX-1:0] fifo_entry_head_r  [FIFO_SIZE-1:0];
reg [CHUNK_IDX-1:0] fifo_entry_tail_r [FIFO_SIZE-1:0];
reg [ARRAY_IDX-1:0] fifo_entry_cur_r  [FIFO_SIZE-1:0];
reg [ARRAY_IDX-1:0] fifo_entry_end_r  [FIFO_SIZE-1:0];

reg [CHUNK_ST_SIZE -1:0] chunk_st_r [CHUNKS-1:0];
reg [WORDS-1:0] chunk_map_r [CHUNKS-1:0];

// Wire declarations
wire [CHUNKS-1:0] chunk_st_inv;
wire [CHUNKS-1:0] chunk_st_pend;
wire [CHUNKS-1:0] chunk_st_val;
wire [ARRAY_IDX-1:0] buffer_begin [FIFO_SIZE-1:0];
wire [ARRAY_IDX-1:0] buffer_end [FIFO_SIZE-1:0];

wire [ARRAY_IDX-1:0] fifo_remaining_words [FIFO_SIZE-1:0];
wire [FIFO_SIZE-1:0] fifo_over;

// Hanshake Valid and Response is favorable (1)
wire cr_req_hsk = cr_req_val && cr_req_rdy;

// CONF REGS
always @(posedge clk)
if (!rst_n) begin
    conf_A_val_r <= 1'b0;
    // FIXME: These would not need reset
    conf_A_r     <= {ADDR{1'b0}};
    conf_fifos_r <= {FIFO_IDX+1{1'b0}};
end else begin
    conf_A_val_r <= cr_conf_val || conf_A_val_r && !invalidate;
    if (cr_conf_val) begin
        conf_A_r <= cr_conf_addr;
        conf_fifos_r <= cr_conf_fifos;
    end
end

assign cr_active = conf_A_val_r;

wire [FIFO_SIZE-1:0] fifo_remaining;
wire [FIFO_SIZE-1:0] fifo_chunk_req;
wire [FIFO_SIZE-1:0] fifo_chunk_inv;
wire [FIFO_SIZE-1:0] alloc_fifo_oh;
wire [FIFO_IDX -1:0] alloc_fifo;
wire [CHUNK_IDX-1:0] chunk_alloc_idx;
wire [CHUNK_IDX-1:0] chunk_head_nxt;
wire [CHUNK_IDX-1:0] chunk_clr_idx;
wire [CHUNK_IDX-1:0] chunk_tail_nxt;
wire [ARRAY_IDX-1:0] current_index;
wire [ARRAY_IDX-1:0] fifo_entry_cur_nxt;

rr_arbiter #(
  .SOURCES(FIFO_SIZE),
  .MODE(1)
  ) u_alloc_chunk_rr(
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (!cr_req_rdy),
  .valid_source   (fifo_chunk_req),
  .arb_src_oh (alloc_fifo_oh),
  .arb_src    (alloc_fifo),
  .arb_val    (cr_req_val)
  );

assign current_index = fifo_entry_cur_r[alloc_fifo];
assign cr_req_idx = chunk_alloc_idx;
wire [ARRAY_IDX+WORD_ALIGN-1:0] current_index_aligned = {current_index, {WORD_ALIGN{1'b0}}};
generate if (ARRAY_IDX+2 > ADDR) begin
    assign cr_req_addr = conf_A_r + current_index_aligned[ADDR-1:0];
end else begin
    assign cr_req_addr = conf_A_r + current_index_aligned;
end endgenerate

wire [CHUNK_IDX-1:0] req_offset;
wire [WORDS:0] words_requested; 
wire [WORDS-1:0] valid_words;
wire [WORDS-1:0] word_map;
wire [WORDS-1:0] word_map_masked;
wire chunk_empty;
wire not_last_word;
wire last_word = !not_last_word;

// Arbitrer to determine which fifo will promote a buffer slot to the main entry
wire [FIFO_SIZE-1:0] fifo_buffer_val;
wire [FIFO_SIZE-1:0] fifo_buffer_req;
wire [FIFO_SIZE-1:0] fifo_buffer_get;
wire [FIFO_SIZE-1:0] fifo_buffer_sel_oh;
wire [FIFO_IDX -1:0] fifo_buffer_sel;
rr_arbiter #(
  .SOURCES(FIFO_SIZE)
  ) u_entry_rr(
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (1'b0),
  .valid_source   (fifo_buffer_req),
  .arb_src_oh (fifo_buffer_sel_oh),
  .arb_src    (fifo_buffer_sel),
  .arb_val    ()
  );

// Arbitrer to determine which entry will make a request to the pipeline
wire [FIFO_SIZE-1:0] fifo_chunk_val;
wire [FIFO_SIZE-1:0] fifo_pipe_req;

wire pipe_val;
wire pipe_rdy;
wire pipe_read = !cr_fill_val;
wire pipe_hsk = pipe_val && pipe_rdy && pipe_read;
wire [63:0] pipe_data;
wire [WORD_IDX-1:0] pipe_word_idx;
wire [FIFO_SIZE-1:0] pipe_fifo_oh;
wire [FIFO_IDX-1:0] pipe_fifo;

// STAGE before CR_PIPE
reg pipe_val_r;
reg data_read_r;
reg [63:0] pipe_data_r;
reg [WORD_IDX-1:0] pipe_word_idx_r;

assign cr_pipe_val = pipe_val_r;
assign cr_pipe_data = data_read_r ? pipe_data : pipe_data_r;
assign pipe_rdy = !pipe_val_r || cr_pipe_rdy;

always @(posedge clk) begin
    if (!rst_n) begin
        pipe_val_r <= 1'b0;
        data_read_r <= 1'b0;
        // FIXME: These would not need reset
        pipe_word_idx_r <= {WORD_IDX{1'b0}};
        pipe_data_r     <= 64'd0;
    end else begin
        pipe_val_r <= pipe_hsk || pipe_val_r && !cr_pipe_rdy;
        data_read_r <= pipe_hsk;
        if (pipe_hsk) begin
            pipe_word_idx_r  <= pipe_word_idx;
        end
        if (data_read_r) begin
            pipe_data_r  <= pipe_data;
        end
    end
end

rr_arbiter #(
  .SOURCES(FIFO_SIZE)
  ) u_pipe_rr(
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (1'b0),
  .valid_source   (fifo_pipe_req),
  .arb_src_oh (pipe_fifo_oh),
  .arb_src    (pipe_fifo),
  .arb_val    (pipe_val)
  );

wire [511:0] read_chunk;
assign pipe_data = read_chunk[pipe_word_idx_r*64+:64];

// Chunk state changers
wire [CHUNKS-1:0] chunk_alloc;
wire [CHUNKS-1:0] chunk_fill;
wire [CHUNKS-1:0] chunk_req;
wire [CHUNKS-1:0] chunk_clr;
wire [FIFO_SIZE-1:0] add_buffer;
wire [FIFO_SIZE-1:0] fifo_chunk_alloc;
wire [FIFO_SIZE-1:0] fifo_chunk_clr;
wire [FIFO_SIZE-1:0] fifo_clr;
wire chunk_clr_val = pipe_hsk && last_word;

assign chunk_fill = ({{CHUNKS-1{1'b0}}, 1'b1} << cr_fill_idx) & {CHUNKS{cr_fill_val}};

// next chunk to allocate by fifo
assign chunk_alloc_idx = fifo_entry_head_r[alloc_fifo];
assign chunk_head_nxt  = chunk_alloc_idx + conf_fifos_r;
assign chunk_alloc = ({{CHUNKS-1{1'b0}}, 1'b1} << chunk_alloc_idx) & {CHUNKS{cr_req_hsk}};

// next chunk to read for pipe issue
assign chunk_clr_idx   = fifo_entry_tail_r[pipe_fifo];
assign chunk_tail_nxt  = chunk_clr_idx + conf_fifos_r;
assign chunk_req = ({{CHUNKS-1{1'b0}}, 1'b1} << chunk_clr_idx) & {CHUNKS{pipe_hsk}};
assign chunk_clr = chunk_req & {CHUNKS{last_word}};

// calculate the next base word index of chunk to allocate
assign req_offset      = cr_req_addr[CHUNK_ALIGN-1:WORD_ALIGN];
assign words_requested = WORDS[WORD_IDX:0] - req_offset; 
assign fifo_entry_cur_nxt = current_index + words_requested;

// add request to buffer
assign add_buffer = ({{FIFO_SIZE-1{1'b0}}, 1'b1} << cr_add_fifo) & {FIFO_SIZE{cr_add_val && conf_A_val_r}};

// buffer wants to allocate fifo entry
assign fifo_buffer_req = fifo_buffer_val & ~fifo_entry_val_r;
// buffer is granted to allocate entry
assign fifo_buffer_get = fifo_buffer_sel_oh & fifo_buffer_req;

//check that we are using the last word of the chunk
assign word_map = chunk_map_r[chunk_clr_idx];
assign word_map_masked = word_map & ~({{WORDS-1{1'b0}}, 1'b1} << pipe_word_idx);
assign not_last_word = |(word_map & (word_map - 1'b1));
  lzc #(
    .WIDTH ( WORDS ),
    .MODE  ( 0 ) // LSB
  ) i_lzc (
    .in_i    ( word_map),
    .cnt_o   ( pipe_word_idx),
    .empty_o ( chunk_empty)
  );

// see valid remaining words, to be set in the newly allocated chunk
wire [ARRAY_IDX-1:0] alloc_remaining_words = fifo_remaining_words[alloc_fifo];
// if more than 16 words remain, use the whole chunk
wire alloc_more_chunks = |alloc_remaining_words[ARRAY_IDX-1:WORD_IDX];
wire [WORDS*2-1:0] chunk_words = { {WORDS{1'b0}}, {WORDS{1'b1}} } << alloc_remaining_words[WORD_IDX-1:0];
// need to consider alignment
assign valid_words = (chunk_words[WORDS*2-1:WORDS] | {WORDS{alloc_more_chunks}}) << req_offset;

generate
for (j = 0; j < CHUNKS; j = j + 1) begin: chunks_gen
    // chunk states
    assign chunk_st_inv [j] = chunk_st_r[j] == CHUNK_ST_INV;
    assign chunk_st_pend[j] = chunk_st_r[j] == CHUNK_ST_PEND;
    assign chunk_st_val [j] = chunk_st_r[j] == CHUNK_ST_VAL;

    always @(posedge clk) begin
        if (!rst_n) begin
            chunk_st_r[j] <= {CHUNK_ST_SIZE{1'b0}};
        end else begin
            chunk_st_r[j] <= {chunk_fill[j]  || chunk_st_r[j][1] && !chunk_clr[j], 
                              chunk_alloc[j] || chunk_st_r[j][0] && !chunk_clr[j]};
        end
    end
    always @(posedge clk) begin
        if (chunk_alloc[j] || chunk_req[j]) begin
            chunk_map_r[j] <= chunk_alloc[j] ? valid_words : word_map_masked;
        end
    end
end

sram_chunk_data chunk_data_array(
    .MEMCLK     (clk),
    .RESET_N    (rst_n),
    .CE         (cr_fill_val || pipe_val && pipe_rdy),
    .A          (cr_fill_val ? cr_fill_idx : chunk_clr_idx),
    .DIN        (cr_fill_data),
    .RDWEN      (!cr_fill_val),
    .BW         ({16{32'hFFFFFFFF}}),
    .DOUT       (read_chunk),
    .BIST_COMMAND(4'h0),
    .BIST_DIN(4'h0),
    .BIST_DOUT(),
    .SRAMID(8'h00)
);

// fifo is not done allocating chunks
wire fifo_remaining_chunks = fifo_entry_end_r[alloc_fifo] > fifo_entry_cur_nxt;
assign fifo_chunk_req   = fifo_entry_val_r & fifo_chunk_inv;
assign fifo_chunk_alloc = {FIFO_SIZE{cr_req_hsk}} & alloc_fifo_oh;
assign fifo_chunk_clr   = {FIFO_SIZE{chunk_clr_val}} & pipe_fifo_oh;
assign fifo_clr = {FIFO_SIZE{!fifo_remaining_chunks}} & fifo_chunk_alloc; 
assign fifo_pipe_req = fifo_chunk_val;

for ( j = 0; j < FIFO_SIZE; j = j + 1) begin: fifos_gen
    assign {fifo_over[j], fifo_remaining_words[j]} = fifo_entry_end_r[j] - fifo_entry_cur_r[j];
    // fifo can request to pipe
    assign fifo_chunk_val[j] = chunk_st_val[fifo_entry_tail_r[j]] && (j < conf_fifos_r);
    // the head is invalid, so we can allocate
    assign fifo_chunk_inv[j] = chunk_st_inv[fifo_entry_head_r[j]];

    // FIFO ENTRY MANAGEMENT
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_entry_val_r [j] <= 1'b0;
            fifo_entry_head_r[j] <= {{CHUNK_IDX-FIFO_IDX{1'b0}}, j[FIFO_IDX-1:0]};
            fifo_entry_tail_r[j] <= {{CHUNK_IDX-FIFO_IDX{1'b0}}, j[FIFO_IDX-1:0]};
            // FIXME: These would not need reset
            fifo_entry_cur_r[j] <= {ARRAY_IDX{1'b0}};
            fifo_entry_end_r[j] <= {ARRAY_IDX{1'b0}};
        end else begin
            fifo_entry_val_r[j] <= fifo_buffer_get[j] || fifo_entry_val_r[j] && !fifo_clr[j];
            // Update next alloc for a fifo
            if (fifo_chunk_alloc[j]) begin
                fifo_entry_head_r[j] <= chunk_head_nxt;
            end
            if (fifo_chunk_clr[j]) begin
                fifo_entry_tail_r[j] <= chunk_tail_nxt;
            end
            if (fifo_buffer_get[j] || fifo_chunk_alloc[j]) begin
                fifo_entry_cur_r[j] <= fifo_buffer_get[j] ? buffer_begin[j] : fifo_entry_cur_nxt;
            end
            if (fifo_buffer_get[j]) begin
                fifo_entry_end_r[j] <= buffer_end[j];
            end
        end
    end

    // FIFO BUFFER MANAGEMENT
    fifo_buffer #(
      .SIZE ( ARRAY_IDX*2 ), //size of begin and end
      .INFLIGHT_IDX  ( 2 )
    ) i_buffer (
      .clk      (clk),
      .rst_n    (rst_n && !invalidate),
      .val_i    ( add_buffer[j]),
      .rdy_o    (  ),
      .data_i   ({cr_add_begin,cr_add_end}),

      .val_o    ( fifo_buffer_val[j]),
      .rdy_i    ( fifo_buffer_get[j] ),
      .data_o    ( {buffer_begin[j],buffer_end[j]} )
    );
end endgenerate

endmodule
