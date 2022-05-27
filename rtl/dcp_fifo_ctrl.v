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
//  Filename      : dcp_fifo_ctrl.v
//  Created On    : 2019-12-28
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Manage FIFO's
// 
//  MSHR interacts with this module by:
//  -(FC_INCR) Request a new FIFO entry for a certain FIFO idx, this entry will be stored in the MSHR entry and requested to the mem system.
//  -(FC_FILL) Memory response (data filled in the FIFO storage). The MSHR can clear its entry after writing here.
//  -(FC_DECR) Read port from a certain FIFO idx. Decrease the HEAD of the FIFO.
//
// === ASSUMPTIONS ===
// -FC_ENTRY_IDX must be always smaller thanm FC_GLOBL_IDX
// -We will not be able to allocate more FIFOs than FC_FIFOS_SIZE (in that case fc_add_res will always be 0, until FIFOs destruction)
// -All FIFOs can share FC_GLOBAL_SIZE entries. If tried to allocate more, fc_add_res will always be 0, until FIFOs destruction)
//
// === ASSERTIONS ===
// -If fc_incr_res is low, and fc_decr_hsk occurs for the same idx, fc_incr_res should be eventually high
// -If fc_fill_entry, that idx must be valid
// -If fc_invalidate an entry, that entry must be valid
//
//==================================================================================================


module dcp_fifo_ctrl 
  #(
    // Configuration Parameters
    parameter FC_GLOBL_IDX = 4, // Total number of entries shared among the FIFOs
    parameter FC_GLOBL_SIZE = 2 ** FC_GLOBL_IDX,
    parameter FC_ENTRY_IDX = 3,
    parameter FC_DATA_SIZE = 32,
    parameter FC_DATA_IDX  = 1,
    parameter FC_FIFOS_IDX = 1,
    parameter FC_DATA_LEN  = 1,
    parameter FC_FIFOS_SIZE = 2
  )
(
    // Clock + Reset
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          c0_invalidate,
    input  wire                          invalidate,

    input  wire  [FC_FIFOS_IDX-1:0]      fc_fifo_stats,
    output wire  [FC_ENTRY_IDX:0]        count_use,
    output wire                          fc_st_empty,
    output wire  [FC_FIFOS_SIZE-1:0]     fc_fifo_val,

    input  wire  [FC_FIFOS_SIZE-1:0]     fc_fifo_decr_len,
    output wire  [FC_FIFOS_SIZE-1:0]     fc_fifo_not_empty,
    input  wire  [FC_FIFOS_SIZE-1:0]     fc_fifo_incr_len,
    output wire  [FC_FIFOS_SIZE-1:0]     fc_fifo_full,
    output wire  [FC_FIFOS_IDX:0]        fc_num_fifos,

    // Fifo increase entry interface    // Fifo add (conf) interface
    input  wire                          fc_add_val, 
    output wire                          fc_add_hsk, 
    input  wire  [FC_GLOBL_IDX-1:0]      fc_add_size, 
    output wire  [FC_FIFOS_IDX-1:0]      fc_add_idx,

    // Fifo clear (delete) interface
    input  wire                          fc_clr_val, 
    input  wire  [FC_FIFOS_SIZE-1:0]     fc_clr_idx_oh,

    // Get (reserve) entry for a fifo 
    input  wire                          fc_incr_val,
    output wire                          fc_incr_res,
    output wire  [FC_GLOBL_IDX-1:0]      fc_incr_entry, // Store this FIFO entry index in the MSHR entry
    output wire  [FC_GLOBL_IDX-1:0]      fc_incr_entry_p1,
    input  wire  [FC_DATA_LEN -1:0]      fc_incr_len, // To know how to interpret the request
    input  wire  [FC_FIFOS_IDX-1:0]      fc_incr_idx,
    input  wire  [FC_FIFOS_SIZE-1:0]     fc_incr_idx_oh,
    input  wire  [FC_DATA_IDX -1:0]      fc_incr_addr_lsb, // Last bits of the addr

    // Fifo decrease entry interface
    // When Load interface tries to read from the FIFO, if will return (res == 1) if the HEAD contained valid data
    // It will return the data and the entry that was read.
    input  wire                          fc_decr_val,
    output wire                          fc_decr_res,
    output wire  [FC_GLOBL_IDX-1:0]      fc_decr_entry, // Entry read, unused
    input  wire  [FC_DATA_LEN -1:0]      fc_decr_len, // To know how to interpret the request
    input  wire  [FC_FIFOS_IDX-1:0]      fc_decr_idx,
    input  wire  [FC_FIFOS_SIZE-1:0]     fc_decr_idx_oh,
    output wire  [FC_DATA_SIZE-1:0]      fc_decr_data0, // Data read
    output wire  [FC_DATA_SIZE-1:0]      fc_decr_data1, // Data read

    // Data fill interface
    input  wire                          fc_fill_val,
    input  wire  [FC_GLOBL_IDX-1:0]      fc_fill_entry,
    output wire  [FC_DATA_IDX -1:0]      fc_fill_addr_lsb,
    input  wire  [FC_DATA_SIZE*2-1:0]    fc_fill_data,

    output wire  [FC_GLOBL_SIZE-1:0] data_val,
    output wire  [FC_GLOBL_SIZE-1:0] data_use
);

//==============================================================================
// Local Parameters
//==============================================================================

genvar j;
localparam FC_STATE_SIZE = 2;
localparam FC_STATE_INV = 2'b00;
localparam FC_STATE_PRI = 2'b01;
localparam FC_STATE_SEC = 2'b10;
localparam FC_STATE_VAL = 2'b11;

/////////////////////////////
// FIFOS control structure //
/////////////////////////////
// Register declarations per fifo
reg [FC_FIFOS_SIZE-1:0] fifo_val_r;
reg [FC_ENTRY_IDX -1:0] fifo_head_r [FC_FIFOS_SIZE-1:0];
reg [FC_ENTRY_IDX -1:0] fifo_tail_r [FC_FIFOS_SIZE-1:0];
reg [FC_GLOBL_IDX -1:0] fifo_top_r  [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_base   [FC_FIFOS_SIZE-1:0];
reg [FC_FIFOS_SIZE-1:0] last_decr_r;
// Fifo allocation index
reg  [FC_FIFOS_IDX:0] fifo_alloc_idx_r;
wire [FC_FIFOS_IDX:0] fifo_alloc_nxt; 

// Hanshake Valid and Response is favorable (1)
wire fc_add_res;
wire fc_incr_hsk;
wire fc_decr_hsk;

// Data struct registers
reg  [FC_STATE_SIZE-1:0] data_st_r [FC_GLOBL_SIZE-1:0];
reg  [FC_DATA_IDX  -1:0] data_idx_r  [FC_GLOBL_SIZE-1:0];
reg  [FC_DATA_SIZE -1:0] data_r     [FC_GLOBL_SIZE-1:0];

// Get current tail entry of Fifo to incr+1
wire [FC_GLOBL_IDX-1:0] fc_decr_entry_p1;

/////////// FIFO/SP state to report to the pipe ///////////////////
// Tells if the whole scratchpad is invalid!
assign fc_st_empty = !(|data_use);
//Tells the FIFOs that are valid
assign fc_fifo_val = fifo_val_r;
// Tell if all fifos are invalid
wire fifo_all_inval = !(|fifo_val_r);
// Last Addr bits of the request that is being filled
assign fc_fill_addr_lsb = data_idx_r[fc_fill_entry];

// PERFORMANCE COUTNERS - Tells how many entries are in use for the stats fifo
generate if (`DCP_PERF_COUNTERS) begin : fifo_perf_counter
    wire [FC_ENTRY_IDX-1:0] fifo_tail_sel = fifo_tail_r[fc_fifo_stats];
    wire [FC_ENTRY_IDX-1:0] fifo_head_sel = fifo_head_r[fc_fifo_stats];
    wire [FC_ENTRY_IDX:0] count_size = fifo_top_r[fc_fifo_stats] - fifo_base[fc_fifo_stats];
    wire [FC_ENTRY_IDX:0] fifo_tail_over = (fifo_tail_sel <= fifo_head_sel) ? {1'b0,fifo_tail_sel} + count_size : {1'b0,fifo_tail_sel};
    assign count_use = fifo_tail_over - {1'b0,fifo_head_sel};
end else begin : no_perf
    assign count_use = {FC_ENTRY_IDX+1{1'b0}};
end endgenerate

// Fifo does exist and the entry is not full (pending to read)
assign fc_incr_res = !(|(fc_fifo_full & fc_incr_idx_oh));
assign fc_incr_hsk = fc_incr_val && fc_incr_res;
// Fifo does exist and the entry is full (written)
assign fc_decr_res = |(fc_fifo_not_empty & fc_decr_idx_oh);
assign fc_decr_hsk = fc_decr_val && fc_decr_res;

// Output values for increase/decrease iface
assign fc_decr_data0 = data_r[fc_decr_entry];
assign fc_decr_data1 = data_r[fc_decr_entry_p1] & {FC_DATA_SIZE{fc_decr_len}};

//////////////////////////////////////////////////////////////////////////
// Wire declarations
wire [FC_FIFOS_SIZE-1:0] fc_clr;
wire [FC_FIFOS_SIZE-1:0] fc_add;
wire [FC_FIFOS_SIZE-1:0] fc_incr;
wire [FC_FIFOS_SIZE-1:0] fc_decr;

// Get the base of the last fifo allocated
wire [FC_GLOBL_IDX-1:0] base_alloc;
wire [FC_GLOBL_IDX-1:0] fifo_top_nxt;
wire alloc_overflow;
wire range_overflow;
wire range_max = !(|base_alloc) && (|fifo_alloc_idx_r); //base is 0 and alloc_idx is not 0
wire num_fifos_max = fifo_alloc_idx_r[FC_FIFOS_IDX]; //fifo counter has overflown
assign base_alloc = fifo_base[fc_add_idx];
assign {alloc_overflow,fifo_top_nxt} = {1'b0, base_alloc} + {1'b0, fc_add_size};
assign range_overflow = num_fifos_max || range_max || alloc_overflow && (|fifo_top_nxt);

// Add new FIFO, make sure there is space (no fifo limit and no overflow). Add error CODES!
assign fc_add_res = !range_overflow;
//Make sure we didnt allocated all FIFOs already or the entry index is not over GLOBAL_SIZE
assign fc_add_hsk = fc_add_val && fc_add_res;
assign fc_add_idx = fifo_alloc_idx_r[FC_FIFOS_IDX-1:0];
assign fc_num_fifos = fifo_alloc_idx_r;

// Get current tail entry of Fifo to incr
wire [FC_ENTRY_IDX-1:0] fc_incr_local_entry;
wire [FC_ENTRY_IDX-1:0] fc_incr_local_entry_p;
wire [FC_GLOBL_IDX-1:0] fc_incr_global_entry_p; 
wire [FC_ENTRY_IDX-1:0] fc_decr_local_entry;
wire [FC_ENTRY_IDX-1:0] fc_decr_local_entry_p;
wire [FC_GLOBL_IDX-1:0] fc_decr_global_entry_p; 
wire [FC_GLOBL_IDX-1:0] fc_incr_base; 
wire [FC_GLOBL_IDX-1:0] fc_incr_top; 
wire [FC_GLOBL_IDX-1:0] fc_decr_base; 
wire [FC_GLOBL_IDX-1:0] fc_decr_top; 
assign fc_incr_local_entry = fifo_tail_r[fc_incr_idx];
assign fc_decr_local_entry = fifo_head_r[fc_decr_idx];
assign fc_incr_local_entry_p = fc_incr_local_entry + {{FC_ENTRY_IDX-2{1'b0}}, fc_incr_len, !fc_incr_len};
assign fc_decr_local_entry_p = fc_decr_local_entry + {{FC_ENTRY_IDX-2{1'b0}}, fc_decr_len, !fc_decr_len};
assign fc_incr_base = fifo_base[fc_incr_idx];
assign fc_decr_base = fifo_base[fc_decr_idx];
assign fc_incr_top  = fifo_top_r[fc_incr_idx];
assign fc_decr_top  = fifo_top_r[fc_decr_idx];

wire incr_overflow;
wire decr_overflow;
assign {incr_overflow, fc_incr_global_entry_p} = {1'b0, fc_incr_base} + fc_incr_local_entry_p;
assign {decr_overflow, fc_decr_global_entry_p} = {1'b0, fc_decr_base} + fc_decr_local_entry_p;

// calculate next value for head and tail
wire [FC_ENTRY_IDX-1:0] fifo_tail_nxt;
wire [FC_ENTRY_IDX-1:0] fifo_head_nxt;
// calculate next value for head and tail
assign fifo_tail_nxt = (fc_incr_global_entry_p[FC_GLOBL_IDX-1:1] == fc_incr_top[FC_GLOBL_IDX-1:1]) ? 
                        {{FC_ENTRY_IDX-1{1'b0}},fc_incr_global_entry_p[0]} : fc_incr_local_entry_p;
assign fifo_head_nxt = (fc_decr_global_entry_p[FC_GLOBL_IDX-1:1] == fc_decr_top[FC_GLOBL_IDX-1:1]) ? 
                        {{FC_ENTRY_IDX-1{1'b0}},fc_decr_global_entry_p[0]} : fc_decr_local_entry_p;


/////////////////////////////////////
wire [FC_FIFOS_SIZE-1:0]                   fifo_global_equal;
wire [FC_GLOBL_IDX-1:0] fifo_global_head         [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_global_head_p1      [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_global_head_p1_wrap [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_global_tail         [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_global_tail_p1      [FC_FIFOS_SIZE-1:0];
wire [FC_GLOBL_IDX-1:0] fifo_global_tail_p1_wrap [FC_FIFOS_SIZE-1:0];

// Output values for increase/decrease iface
assign fc_incr_entry = fifo_global_tail[fc_incr_idx];
assign fc_decr_entry = fifo_global_head[fc_decr_idx];
assign fc_incr_entry_p1 = fifo_global_tail_p1_wrap[fc_incr_idx];
assign fc_decr_entry_p1 = fifo_global_head_p1_wrap[fc_decr_idx];

wire decr_more_than_incr = fc_decr_len && !fc_incr_len;
wire incr_more_than_decr = fc_incr_len && !fc_decr_len;

generate
//The base pointer of the FIFO
assign fifo_base[0] = {FC_GLOBL_IDX{1'b0}};
for ( j = 1; j < FC_FIFOS_SIZE; j = j + 1) begin: fc_fifos_full_gen
    assign fifo_base[j] = fifo_top_r[j-1];
end

for ( j = 0; j < FC_FIFOS_SIZE; j = j + 1) begin: fc_fifos_gen
    // NOT EMPTY CALCULATIONS
    wire head_overflow;
    assign {head_overflow, fifo_global_head_p1[j]} = {1'b0, fifo_global_head[j]} + {{FC_GLOBL_IDX{1'b0}}, 1'b1};
    assign fifo_global_head_p1_wrap[j] = (fifo_global_head_p1[j] == fifo_top_r[j]) ? fifo_base[j] : fifo_global_head_p1[j];

    wire head_valid = data_val[fifo_global_head[j]];
    wire head_p1_valid = data_val[fifo_global_head_p1_wrap[j]];
    assign fc_fifo_not_empty[j] = head_valid && ( head_p1_valid || !fc_fifo_decr_len[j]);

    //NOT FULL CALCULATIONS
    wire tail_overflow;
    assign {tail_overflow, fifo_global_tail_p1[j]} = {1'b0, fifo_global_tail[j]} + {{FC_GLOBL_IDX{1'b0}}, 1'b1};
    assign fifo_global_tail_p1_wrap[j] = (fifo_global_tail_p1[j] == fifo_top_r[j]) ? fifo_base[j] : fifo_global_tail_p1[j];

    assign fc_fifo_full[j] = fifo_global_equal[j] && !last_decr_r[j] || 
                            fc_fifo_incr_len[j] && (fifo_global_tail_p1_wrap[j] == fifo_global_head[j]);

    // FIFO MANAGEMENT
    assign fc_clr[j]  = c0_invalidate || fc_clr_val && fc_clr_idx_oh[j];
    assign fc_add[j]  = fc_add_hsk  && (j[FC_FIFOS_IDX-1:0] == fc_add_idx);
    assign fc_incr[j] = fc_incr_hsk && fc_incr_idx_oh[j];
    assign fc_decr[j] = fc_decr_hsk && fc_decr_idx_oh[j];
  
    wire incr = fc_incr[j] && (!fc_decr[j] || incr_more_than_decr);
    wire decr = fc_decr[j] && (!fc_incr[j] || decr_more_than_incr);
    always @(posedge clk)
    if (!rst_n) begin
        fifo_val_r[j]  <= 1'b0;
        last_decr_r[j] <= 1'b1; 
        fifo_top_r[j]  <= {FC_GLOBL_IDX{1'b0}};
    end else begin
        fifo_val_r[j] <= fc_add[j] || fifo_val_r[j] && !fc_clr[j];
        last_decr_r[j] <= decr || c0_invalidate || last_decr_r[j] && !incr;
        if (fc_add[j]) begin
            fifo_top_r[j] <= fifo_top_nxt;
        end
    end
  
    always @(posedge clk)
    if (!rst_n) begin
        fifo_head_r[j] <= {FC_ENTRY_IDX{1'b0}};
        fifo_tail_r[j] <= {FC_ENTRY_IDX{1'b0}};
    end else begin
        if (fc_incr[j] || c0_invalidate) begin // increment on allocation
            fifo_tail_r[j] <= fifo_tail_nxt & {FC_ENTRY_IDX{!c0_invalidate}};
        end
        if (fc_decr[j] || c0_invalidate) begin //increment on commit
            fifo_head_r[j] <= fifo_head_nxt & {FC_ENTRY_IDX{!c0_invalidate}};
        end
    end //end else

    assign fifo_global_head[j] = fifo_base[j] + fifo_head_r[j];
    assign fifo_global_tail[j] = fifo_base[j] + fifo_tail_r[j];
    assign fifo_global_equal[j] = fifo_global_tail[j] == fifo_global_head[j];
end //end for
endgenerate

////////////////////////////// FIFO FILL ///////////////////////////////

wire fill_overflow_p1;
wire [FC_GLOBL_IDX-1:0] fc_fill_entry_p1;
assign {fill_overflow_p1, fc_fill_entry_p1} = {1'b0, fc_fill_entry} + {{FC_GLOBL_IDX{1'b0}}, 1'b1};

wire [FC_GLOBL_SIZE-1:0] fc_fill0;
wire [FC_GLOBL_SIZE-1:0] fc_fill1;
wire [FC_GLOBL_SIZE-1:0] fc_fill;
wire [FC_GLOBL_SIZE-1:0] fc_inval;
wire [FC_GLOBL_SIZE-1:0] fc_alloc_pri;
wire [FC_GLOBL_SIZE-1:0] fc_alloc_sec;

wire alloc_sec = fc_incr_len[0] && !fc_incr_addr_lsb[0];
wire two_alloc = fc_incr_len[0] && fc_incr_addr_lsb[0];
generate
for ( j = 0; j < FC_GLOBL_SIZE; j = j + 1) begin: fc_gen
    assign data_val[j] = data_st_r[j] == FC_STATE_VAL;
    assign data_use[j] = |data_st_r[j]; //data_st_r[i] != FC_STATE_INV;

    assign fc_fill0[j] = fc_fill_val && (j[FC_GLOBL_IDX-1:0] == fc_fill_entry);
    assign fc_fill1[j] = fc_fill_val && (j[FC_GLOBL_IDX-1:0] == fc_fill_entry_p1) && (data_st_r[j] == FC_STATE_SEC);
    assign fc_fill[j]  = fc_fill0[j] || fc_fill1[j];

    assign fc_alloc_pri[j] = fc_incr_hsk && (j[FC_GLOBL_IDX-1:0] == fc_incr_entry) || fc_alloc_sec[j] && two_alloc; 
    assign fc_alloc_sec[j] = fc_incr_hsk && (j[FC_GLOBL_IDX-1:0] == fc_incr_entry_p1);

    assign fc_inval[j] = fc_decr_hsk && ( (j[FC_GLOBL_IDX-1:0] == fc_decr_entry) || 
                           fc_decr_len && (j[FC_GLOBL_IDX-1:0] == fc_decr_entry_p1)) || invalidate;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            data_st_r[j] <= FC_STATE_INV;
        end else begin  //invalidate only if state is valid
            data_st_r[j] <= {fc_alloc_sec[j] && alloc_sec || fc_fill[j] || data_st_r[j][1] && !(fc_inval[j] && data_st_r[j][0]),
                             fc_alloc_pri[j] || fc_fill[j] || data_st_r[j][0] && !(fc_inval[j] && data_st_r[j][1])}; 
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            data_idx_r[j] <= {FC_DATA_IDX{1'b0}};
        end else if (fc_alloc_pri[j]) begin
            data_idx_r[j] <= fc_incr_addr_lsb;
        end
    end
    always @(posedge clk) begin
        if (fc_fill0[j]) begin
            data_r[j] <= fc_fill_data[FC_DATA_SIZE-1:0];
        end else if (fc_fill1[j]) begin
            data_r[j] <= fc_fill_data[FC_DATA_SIZE*2-1:FC_DATA_SIZE];
        end
    end
end //end for
endgenerate

assign fifo_alloc_nxt = fifo_alloc_idx_r + {{FC_FIFOS_IDX{1'b0}}, 1'b1};
always @(posedge clk) begin
    if (!rst_n) begin
        fifo_alloc_idx_r <= {FC_FIFOS_IDX+1{1'b0}};
    end else if (fc_add_hsk || fifo_all_inval) begin
        fifo_alloc_idx_r <= fc_add_hsk ? fifo_alloc_nxt : {FC_FIFOS_IDX+1{1'b0}};
    end
end

endmodule
