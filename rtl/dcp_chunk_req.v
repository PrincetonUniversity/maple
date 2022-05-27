
// ========== Copyright Header Begin ============================================
// Copyright (c) 2019 Princeton University
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
//  Filename      : dcp_chunk_req.v
//  Created On    : 2020-07-01
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Chunk requester
// 
// === ASSERTIONS ===
//
//==================================================================================================
`include "dcp.h"

module dcp_chunk_req 
(
    // Clock + Reset
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          c0_invalidate,
    input  wire                          invalidate,
    input  wire                          tlb_flush,

    /*AUTOSVA
    cradd_tlbreq: cr_add -NOT> tlb_req
    tlb_req_val = tlb_cr_req

    tlbreq_crreq:  tlb_cr -NOT> cr_req
    tlb_cr_val = tlb_cr_req

    crreq_crfill: cr_reqt -IN> cr_fill
    cr_reqt_val = cr_req_val
    cr_reqt_rdy = cr_req_rdy
    [`CR_CHUNK_IDX-1:0] cr_reqt_transid = cr_req_idx
    [`CR_CHUNK_IDX-1:0] cr_fill_transid = cr_fill_idx
    */
    output wire                           tlb_cr_req,
    input  wire                           tlb_cr_ack,
    output wire [`DCP_VADDR   -12-1:0]    tlb_cr_vpage,
    input  wire [`DCP_PADDR   -12-1:0]    tlb_ppn,

    input  wire                          cr_conf_val,
    input  wire [`DCP_VADDR-1:0]         cr_conf_addr,
    input  wire [`DCP_OPCODE_WIDTH-1:0]  cr_conf_op,
    input  wire                          cr_conf_coherent,
    input  wire [`FC_FIFO_IDX:0]         cr_conf_fifos,

    input  wire [`DCP_VADDR-1:0] cr_B,

    input  wire                          cr_add_val,
    input  wire [`FC_FIFO_SIZE-1:0]      cr_add_fifo_oh, 
    input  wire [`CR_ARRAY_IDX-1:0]      cr_add_begin,
    input  wire [`CR_ARRAY_IDX-1:0]      cr_add_end,

    output wire                          cr_pipe_val,
    input  wire                          cr_pipe_rdy,
    output wire [`FC_FIFO_SIZE-1:0]      cr_pipe_fifo_oh,
    input  wire [`FC_FIFO_SIZE-1:0]      cr_pipe_fifo_rdy, 
    output reg  [`DCP_OPCODE_WIDTH-1:0]  cr_pipe_op,
    output reg                           cr_pipe_len,
    output wire [`DCP_VADDR-1:0]         cr_pipe_addr,
    output wire [31:0]                   cr_pipe_data,
    
    output wire                          cr_active,
    // Data request interface
    output  wire                          cr_req_val,
    output  wire                          cr_req_coherent,
    input   wire                          cr_req_rdy,
    output  wire  [`CR_CHUNK_IDX   -1:0]  cr_req_idx,
    output  wire  [`DCP_PADDR  -1:0]      cr_req_paddr,
    output  wire                          cr_req_part,

    // Data fill interface
    input  wire                          cr_fill_val,
    input  wire                          cr_fill_part,
    input  wire  [`CR_CHUNK_IDX   -1:0]  cr_fill_idx,
    input  wire  [`CR_CHUNK_SIZE  -1:0]  cr_fill_data
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

localparam CHUNKS = 2**`CR_CHUNK_IDX;
localparam CHUNK_ALIGN = 6;
localparam WORD_ALIGN = 2; //32bit is 2^2 bytes and 64 bit is 2^3 bytes
localparam WORD_SIZE = (2**WORD_ALIGN)*8;
localparam WORD_IDX = CHUNK_ALIGN-WORD_ALIGN;
localparam WORDS = 2**WORD_IDX;

/////////////////////////////
// Input control structure //
/////////////////////////////
// Register declarations per fifo
reg conf_A_val_r;
reg [`DCP_VADDR-1:0] conf_A_r;
reg [`DCP_OPCODE_WIDTH-1:0] conf_op_r;
reg                         conf_coherent_r;
reg [`FC_FIFO_IDX:0] conf_fifos_r;

reg [`FC_FIFO_SIZE-1:0] fifo_entry_val_r;
reg [`CR_CHUNK_IDX-1:0] fifo_entry_head_r [`FC_FIFO_SIZE-1:0];
reg [`CR_CHUNK_IDX-1:0] fifo_entry_tail_r [`FC_FIFO_SIZE-1:0];
reg [`CR_ARRAY_IDX-1:0] fifo_entry_cur_r  [`FC_FIFO_SIZE-1:0];
reg [`CR_ARRAY_IDX-1:0] fifo_entry_end_r  [`FC_FIFO_SIZE-1:0];

reg [CHUNK_ST_SIZE -1:0] chunk_st_r [CHUNKS-1:0];
reg [WORDS-1:0] chunk_map_r [CHUNKS-1:0];

// Wire declarations
wire [CHUNKS-1:0] chunk_st_inv;
wire [CHUNKS-1:0] chunk_st_pend;
wire [CHUNKS-1:0] chunk_st_val;
wire [`CR_ARRAY_IDX-1:0] buffer_begin [`FC_FIFO_SIZE-1:0];
wire [`CR_ARRAY_IDX-1:0] buffer_end [`FC_FIFO_SIZE-1:0];

wire [`CR_ARRAY_IDX-1:0] fifo_remaining_words [`FC_FIFO_SIZE-1:0];
wire [`FC_FIFO_SIZE-1:0] fifo_over;

// Hanshake Valid and Response is favorable (1)
wire cr_req_val_pre;
wire cr_req_hsk = cr_req_val && cr_req_rdy;
assign cr_req_coherent = conf_coherent_r;

// TLB interface!
reg tlb_requested_r,tlb_request_r;
reg ppn_val_r;
reg [`DCP_PADDR-12-1:0] ppn_r;
reg [`DCP_VADDR-12-1:0] vpage_r;
wire [`DCP_VADDR-1:0] req_vaddr;
assign tlb_cr_vpage = req_vaddr[`DCP_VADDR-1:12];
wire tlb_hsk = tlb_cr_req && tlb_cr_ack;
wire ppn_hit = ppn_val_r && (vpage_r == tlb_cr_vpage);
assign tlb_cr_req = tlb_request_r;
wire tlb_requested = cr_req_val_pre && !tlb_request_r && ppn_hit || tlb_hsk || tlb_requested_r;
assign cr_req_val = cr_req_val_pre && tlb_requested;

always @(posedge clk) begin
    if (!rst_n) begin
        tlb_request_r <= 1'b0;
        tlb_requested_r <= 1'b0;
        ppn_val_r <= 1'b0;
        ppn_r     <= {`DCP_PADDR-12{1'b0}};
        vpage_r   <= {`DCP_VADDR-12{1'b0}};
    end else begin
        tlb_request_r <= cr_req_val_pre && !tlb_requested;
        tlb_requested_r <= tlb_requested && !cr_req_hsk;
        ppn_val_r <= tlb_hsk || ppn_val_r && !tlb_flush;
        if (tlb_hsk) begin
            ppn_r <= tlb_ppn;
            vpage_r <= tlb_cr_vpage;
        end
    end
end

// PERFORMANCE COUNTERS - Tells how many entries are in use
generate if (`DCP_PERF_COUNTERS) begin : fifo_perf_counter
    assign cr_active = conf_A_val_r;
end else begin : no_perf
    assign cr_active = conf_A_val_r;
end endgenerate

// CONF REGS
always @(posedge clk)
if (!rst_n) begin
    conf_A_val_r <= 1'b0;
    // FIXME: These would not need reset
    conf_A_r     <= {`DCP_VADDR{1'b0}};
    conf_op_r    <= {`DCP_OPCODE_WIDTH{1'b0}};
    conf_coherent_r <= 1'b0;
    conf_fifos_r <= {`FC_FIFO_IDX+1{1'b0}};
end else begin
    conf_A_val_r <= cr_conf_val || conf_A_val_r && !c0_invalidate;
    if (cr_conf_val) begin
        conf_A_r  <= cr_conf_addr;
        conf_op_r <= cr_conf_op;
        conf_coherent_r <= cr_conf_coherent;
        conf_fifos_r <= cr_conf_fifos;
    end
end

wire [`FC_FIFO_SIZE-1:0] fifo_remaining;
wire [`FC_FIFO_SIZE-1:0] fifo_chunk_req;
wire [`FC_FIFO_SIZE-1:0] fifo_chunk_inv;
wire [`FC_FIFO_SIZE-1:0] alloc_fifo_oh;
wire [`FC_FIFO_IDX -1:0] alloc_fifo;
wire [`CR_CHUNK_IDX-1:0] chunk_alloc_idx;
wire [`CR_CHUNK_IDX-1:0] chunk_head_nxt;
wire [`CR_CHUNK_IDX-1:0] chunk_clr_idx;
wire [`CR_CHUNK_IDX-1:0] chunk_tail_nxt;
wire [`CR_ARRAY_IDX-1:0] current_index;
wire [`CR_ARRAY_IDX-1:0] fifo_entry_cur_nxt;

rr_arbiter #(
  .SOURCES(`FC_FIFO_SIZE),
  .MODE(1)
  ) u_alloc_chunk_rr(
  .clk     (clk),
  .reset_n (rst_n && !invalidate),
  .stall   (!cr_req_hsk),
  .valid_source   (fifo_chunk_req),
  .arb_src_oh (alloc_fifo_oh),
  .arb_src    (alloc_fifo),
  .arb_val    (cr_req_val_pre)
  );

assign current_index = fifo_entry_cur_r[alloc_fifo] & {`CR_ARRAY_IDX{cr_req_val_pre}};
assign cr_req_idx = chunk_alloc_idx;
wire [`CR_ARRAY_IDX+WORD_ALIGN-1:0] current_index_p4 = {current_index, {WORD_ALIGN{1'b0}}};
generate if (`CR_ARRAY_IDX+2 > `DCP_VADDR) begin
    assign req_vaddr = cr_B + current_index_p4[`DCP_VADDR-1:0];
end else begin
    assign req_vaddr = cr_B + current_index_p4;
end endgenerate
assign cr_req_paddr = {tlb_hsk ? tlb_ppn : ppn_r, req_vaddr[11:0]};

wire [WORD_IDX-1:0] req_offset;
wire [WORDS:0] words_requested; 
wire [WORDS-1:0] valid_words;
wire [WORDS-1:0] word_map;
wire [WORDS-1:0] word_map_masked;
wire chunk_empty;
wire not_last_word;
wire last_word = !not_last_word;

// Arbitrer to determine which fifo will promote a buffer slot to the main entry
wire [`FC_FIFO_SIZE-1:0] fifo_buffer_val;
wire [`FC_FIFO_SIZE-1:0] fifo_buffer_req;
wire [`FC_FIFO_SIZE-1:0] fifo_buffer_get;
wire [`FC_FIFO_SIZE-1:0] fifo_buffer_sel_oh;
wire [`FC_FIFO_IDX -1:0] fifo_buffer_sel;
rr_arbiter #(
  .SOURCES(`FC_FIFO_SIZE)
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
wire [`FC_FIFO_SIZE-1:0] fifo_chunk_val;
wire [`FC_FIFO_SIZE-1:0] fifo_pipe_req;

wire pipe_val;
wire pipe_rdy;
wire pipe_read = !cr_fill_val;
wire pipe_hsk = pipe_val && pipe_rdy && pipe_read;
wire [`CR_ARRAY_IDX-1:0] pipe_data;
wire [WORD_IDX-1:0] pipe_word_idx;
wire [`FC_FIFO_SIZE-1:0] pipe_fifo_oh;
wire [`FC_FIFO_IDX-1:0] pipe_fifo;

// STAGE before CR_PIPE
reg pipe_val_r;
reg data_read_r;
reg [`CR_ARRAY_IDX-1:0] pipe_data_r;
reg [WORD_IDX-1:0] pipe_word_idx_r;
reg [`FC_FIFO_SIZE-1:0] pipe_fifo_oh_r;

assign cr_pipe_val = pipe_val_r;
assign cr_pipe_fifo_oh = pipe_fifo_oh_r;
assign cr_pipe_data = data_read_r ? pipe_data : pipe_data_r;
assign pipe_rdy = !pipe_val_r || cr_pipe_rdy;

always @(posedge clk) begin
    if (!rst_n) begin
        pipe_val_r <= 1'b0;
        data_read_r <= 1'b0;
        // FIXME: These would not need reset
        pipe_fifo_oh_r  <= {`FC_FIFO_SIZE{1'b0}};
        pipe_word_idx_r <= {WORD_IDX{1'b0}};
        pipe_data_r     <= {`CR_ARRAY_IDX{1'b0}};
    end else begin
        pipe_val_r <= pipe_hsk || pipe_val_r && !cr_pipe_rdy;
        data_read_r <= pipe_hsk;
        if (pipe_hsk) begin
            pipe_fifo_oh_r  <= pipe_fifo_oh;
            pipe_word_idx_r  <= pipe_word_idx;
        end
        if (data_read_r) begin
            pipe_data_r  <= pipe_data;
        end
    end
end

rr_arbiter #(
  .SOURCES(`FC_FIFO_SIZE)
  ) u_pipe_rr(
  .clk     (clk),
  .reset_n (rst_n && !invalidate),
  .stall   (1'b0),
  .valid_source   (fifo_pipe_req),
  .arb_src_oh (pipe_fifo_oh),
  .arb_src    (pipe_fifo),
  .arb_val    (pipe_val)
  );

always @ * begin : opcode
    case (conf_op_r)
        // Assume that the last bit set of the opcode reflects the 64 version of the load,
        // and that the opcodes are contiguous for 32 and 64 loads
        `DCP_LP_TLOAD32,`DCP_LP_TLOAD64: begin
          cr_pipe_op = `DCP_TLOAD32_OP + conf_op_r[0];
          cr_pipe_len = conf_op_r[0];
        end
        `DCP_LP_LLC_32,`DCP_LP_LLC_64: begin
          cr_pipe_op = `DCP_LLC_32 + conf_op_r[0];
          cr_pipe_len = conf_op_r[0];
        end
        `DCP_LP_PREFETCH: begin
          cr_pipe_op = `DCP_PREFETCH;
          cr_pipe_len = 1'b1;
        end
        default: begin
          cr_pipe_op = `DCP_PROD_OP;
          cr_pipe_len = 1'b0;
        end
    endcase
end //always_comb

wire [511:0] read_chunk;
wire [511:0] bw;
wire [63:0] b_block;
wire [`CR_ARRAY_IDX-1:0] b_value;
// The data is in "big endian" between flits, but LitEnd within the Flit!
assign b_block = read_chunk[pipe_word_idx_r[WORD_IDX-1:1]*64+:64];
generate if (`LITTLE_ENDIAN) begin : little_endian_gen
    assign pipe_data = b_block[!pipe_word_idx_r[0]*32+:32];
    assign b_value = {cr_pipe_data[7:0], cr_pipe_data[15:8], cr_pipe_data[23:16], cr_pipe_data[31:24]};
end else begin : big_endian_gen
    assign pipe_data = b_block[pipe_word_idx_r[0]*32+:32];
    assign b_value = cr_pipe_data;
end endgenerate

assign cr_pipe_addr = conf_A_r + ({b_value,2'd0} << cr_pipe_len);

// Chunk state changers
wire [CHUNKS-1:0] chunk_alloc;
wire [CHUNKS-1:0] chunk_fill;
wire [CHUNKS-1:0] chunk_req;
wire [CHUNKS-1:0] chunk_clr;
wire [`FC_FIFO_SIZE-1:0] add_buffer;
wire [`FC_FIFO_SIZE-1:0] fifo_chunk_alloc;
wire [`FC_FIFO_SIZE-1:0] fifo_chunk_clr;
wire [`FC_FIFO_SIZE-1:0] fifo_clr;
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

// add request to buffer
assign add_buffer = {`FC_FIFO_SIZE{cr_add_val}} & cr_add_fifo_oh;

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
wire [`CR_ARRAY_IDX-1:0] alloc_remaining_words = fifo_remaining_words[alloc_fifo];
// if more than 16 words remain, use the whole chunk
wire alloc_more_chunks = |alloc_remaining_words[`CR_ARRAY_IDX-1:4];
wire [WORDS*2-1:0] chunk_words = { {WORDS{1'b0}}, {WORDS{1'b1}} } << alloc_remaining_words[3:0];
// need to consider alignment
assign valid_words = (chunk_words[WORDS*2-1:WORDS] | {WORDS{alloc_more_chunks}}) << req_offset;

wire [511:0] fill_chunk = { cr_fill_part ? cr_fill_data[127:0]:cr_fill_data[511:384],
                            cr_fill_part ? cr_fill_data[127:0]:cr_fill_data[383:256],
                            cr_fill_part ? cr_fill_data[127:0]:cr_fill_data[255:128],
                            cr_fill_data[127:0] };
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

for (j = 0; j < WORDS; j = j + 1) begin: words_gen
    assign bw[j*WORD_SIZE+:WORD_SIZE] = {WORD_SIZE{1'b1}};
end

sram_chunk_data chunk_data_array(
    .MEMCLK     (clk),
    .RESET_N    (rst_n),
    .CE         (cr_fill_val || pipe_val && pipe_rdy),
    .A          (cr_fill_val ? cr_fill_idx : chunk_clr_idx),
    .DIN        (fill_chunk),
    .RDWEN      (!cr_fill_val),
    .BW         (bw),
    .DOUT       (read_chunk),
    .BIST_COMMAND(4'h0),
    .BIST_DIN(4'h0),
    .BIST_DOUT(),
    .SRAMID(8'h00)
);

// calculate the next base word index of chunk to allocate
assign req_offset      = cr_req_paddr[CHUNK_ALIGN-1:WORD_ALIGN];
assign words_requested = WORDS[WORD_IDX:0] - req_offset; 
assign fifo_entry_cur_nxt = current_index + words_requested;

// Calculate if we need less than 16 words
wire lt8w = !(|alloc_remaining_words[`CR_ARRAY_IDX-1:3]);
wire [2:0] wordc = alloc_remaining_words[2:0]; // word count, less than 8
wire lt4w = lt8w && wordc <=4;
wire lt3w = lt8w && !wordc[2];
wire lt2w = lt3w && wordc[1:0] != 2'b11;
wire lt1w = lt3w && !wordc[1];
wire req16 = req_offset[3] && req_offset[2] || // in the 4th block
             !req_offset[1] && !req_offset[0] && lt4w || // aligned word to 00 and less than 4 words
             !req_offset[1] &&  req_offset[0] && lt3w ||
              req_offset[1] && !req_offset[0] && lt2w || lt1w;
assign cr_req_part = req16;

// fifo is not done allocating chunks
wire fifo_remaining_chunks = fifo_entry_end_r[alloc_fifo] > fifo_entry_cur_nxt;
// Wants to request chunks when entry is val and there are chunks free
assign fifo_chunk_req   = fifo_entry_val_r & fifo_chunk_inv;
// Alloc chunk when we request it to memory (request is ack'ed)
assign fifo_chunk_alloc = {`FC_FIFO_SIZE{cr_req_hsk}} & alloc_fifo_oh;
// Request to the Pipe when the chunk is valid and the Pipe can accept them
assign fifo_pipe_req = fifo_chunk_val & cr_pipe_fifo_rdy;
// Clear chunk when we request to the Pipe the last Word
assign fifo_chunk_clr   = {`FC_FIFO_SIZE{chunk_clr_val}} & pipe_fifo_oh;
assign fifo_clr = {`FC_FIFO_SIZE{!fifo_remaining_chunks}} & fifo_chunk_alloc | {`FC_FIFO_SIZE{invalidate}}; 

for ( j = 0; j < `FC_FIFO_SIZE; j = j + 1) begin: fifos_gen
    assign {fifo_over[j], fifo_remaining_words[j]} = fifo_entry_end_r[j] - fifo_entry_cur_r[j];
    // fifo can request to pipe
    assign fifo_chunk_val[j] = chunk_st_val[fifo_entry_tail_r[j]] && (j[`FC_FIFO_IDX:0] < conf_fifos_r);
    // the head is invalid, so we can allocate
    assign fifo_chunk_inv[j] = chunk_st_inv[fifo_entry_head_r[j]];

    // FIFO ENTRY MANAGEMENT
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_entry_val_r [j] <= 1'b0;
            fifo_entry_head_r[j] <= {{`CR_CHUNK_IDX-`FC_FIFO_IDX{1'b0}}, j[`FC_FIFO_IDX-1:0]};
            fifo_entry_tail_r[j] <= {{`CR_CHUNK_IDX-`FC_FIFO_IDX{1'b0}}, j[`FC_FIFO_IDX-1:0]};
            // FIXME: These would not need reset
            fifo_entry_cur_r[j] <= {`CR_ARRAY_IDX{1'b0}};
            fifo_entry_end_r[j] <= {`CR_ARRAY_IDX{1'b0}};
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
      .SIZE ( `CR_ARRAY_IDX*2 ), //size of begin and end
      .INFLIGHT_IDX  ( 2 ) // Index 2, so 4 entries!
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
`ifdef DEC_ASSERT_ON
store_ready: assert property (
   @(posedge clk) disable iff (!rst_n) fifo_over == {`FC_FIFO_SIZE{1'b0}} )
     else $fatal(1,"FIFO OVER SHOULD ALWAYS be 0");
`endif
endmodule
