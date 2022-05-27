/*
Copyright (c) 2020 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//==================================================================================================
//  Filename      : maple.v
//  Created On    : 2020-09-19
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : MAPLE pipelines and Chunk Request
//
//==================================================================================================

//`timescale 1 ns / 10 ps
`include "dcp.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif

module maple (
    input wire clk,
    input wire rst_n,
    input  wire invalidate,
    input  wire c0_invalidate,

    output wire                            tlb_maple_req,
    input  wire                            tlb_maple_ack,
    output wire [`DCP_VADDR   -12-1:0]     tlb_maple_vpage,
    input  wire [`DCP_PADDR   -12-1:0]     tlb_ppn,

    // Add Fifos
    input  wire                        fc_add_val,
    output  wire                       fc_add_hsk,
    input  wire  [`FC_GLOBL_IDX-1:0]   fc_add_size,
    output wire  [`FC_FIFO_IDX-1:0]    fc_add_idx,
    input  wire                        fc_clr_val,

    // Data fill interface
    input  wire                        fc_fill_val,
    input  wire  [`FC_GLOBL_IDX  -1:0] fc_fill_entry,
    input  wire  [`FC_DATA_SIZE*2-1:0] fc_fill_data,
    output wire  [`FC_DATA_IDX   -1:0] fc_fill_addr_lsb,
    output wire                        fc_st_empty,
    output wire [`FC_FIFO_SIZE   -1:0] fc_fifo_val,
    output wire [`FC_FIFO_IDX      :0] cr_conf_fifos,

    input  wire                        conf_op_stats,
    output wire [`FC_ENTRY_IDX     :0] count_use,
    output wire [`DCP_COUNT_STALL-1:0] count_st_fifo_stall,
    output wire [`DCP_COUNT_STALL-1:0] count_ld_fifo_stall,
    output wire [`DCP_COUNT_LAT  -1:0] count_st_fifo,
    output wire [`DCP_COUNT_LAT - 1:0] count_ld_fifo,
    output wire [`DCP_COUNT_STALL-1:0] count_dram_cycles,
    output wire [`DCP_COUNT_STALL-1:0] count_dram_trans,
    output wire [`DCP_COUNT_TRANS-1:0] count_fifo_transactions,
    output wire [`DCP_COUNT_TRANS-1:0] count_transactions,
    input  wire count_transactions_clr,
    input  wire count_noc2_stall_clr,
    input  wire count_noc2_stall_clk,

    input wire [`FC_FIFO_SIZE-1:0] c0_aconfigured_r,
    input wire [`FC_FIFO_SIZE-1:0] c0_econfigured_r,

    output  reg [`FC_FIFO_SIZE  -1:0]  l0_val_r,
    output  reg [`FC_FIFO_SIZE  -1:0]  s0_val_r,

    input  wire                        l0_val,
    output wire                        l0_rdy,
    input  wire                        l0_len,

    input  wire                        s0_val,
    output wire                        s0_rdy,
    input  wire [`FC_FIFO_SIZE  -1:0]  s0_fifo_arb,
    input  wire [`FC_FIFO_IDX   -1:0]  s0_fifo,
    input  wire [`FC_FIFO_SIZE  -1:0]  s0_fifo_oh,
    input  wire                         pipe_len, 
    input  wire [`DCP_OPCODE_WIDTH-1:0] pipe_op,
    input  wire                         pipe_loop,
    input  wire [`PACKET_HOME_ID_WIDTH-1:0] pipe_src,
    input  wire [`DCP_MSHRID_WIDTH -1:0] pipe_mshr,

    input  wire s0_addr_val,
    input  wire s0_data1_val,
    input  wire s0_data2_val,
    input  wire s0_double_load,
    input  wire [31:0] s0_data1,
    input  wire [`DCP_VADDR-1:0] s0_data2,
    input  wire [31:0] noc1_cmp_data1,
    input  wire [31:0] noc1_cmp_data2,
    input  wire [`DCP_VADDR-1:0] noc1_addr,
 
    output wire s2_req,
    input  wire s2_req_rdy,
    output wire [63:0] s2_data,
    output wire [`DCP_MSHRID_WIDTH-1:0]  s2_mshr,
    output wire [`FC_GLOBL_IDX    -1:0]  s2_entry,
    output wire [`FC_GLOBL_IDX    -1:0]  s2_entry_p1,
    output wire                          s2_4byte_align,
    output wire [`DCP_OPCODE_WIDTH-1:0]  s2_op,
    output wire [`DCP_REQ_LEN     -1:0]  s2_len,
    output wire [`PACKET_HOME_ID_WIDTH-1:0] s2_src,
    output wire [`DCP_PADDR_MASK          ] s2_addr,
    output wire s2_op_rmw, s2_op_prod, s2_op_prefetch,
    output wire s2_op_tload, s2_op_tload_32, s2_op_tload_64,

    output wire s3_val,
    input  wire s3_rdy,
    output wire [`DCP_MSHRID_WIDTH-1:0] s3_mshr,
    output wire [`PACKET_HOME_ID_WIDTH-1:0] s3_src, 

    output wire l2_val,
    input  wire l2_rdy,
    output wire [`DCP_MSHRID_WIDTH-1:0] l2_mshr,
    output wire [`PACKET_HOME_ID_WIDTH-1:0] l2_src, 
    output wire [`DCP_DATA_WIDTH-1:0] l2_data,

    output wire [50:0] pipe_debug,
    output wire [63:0] fc_debug_use,
    output wire [63:0] fc_debug_val
    );

genvar j;
///////////////////////////////////////
// Fifo management interface
////////////////////////////////////////
wire  [`FC_FIFO_IDX-1:0]      fc_fifo_stats;
// Fifo increase entry interface
wire                          fc_incr_hsk;
wire                          fc_incr_val;
wire                          fc_incr_res;
wire  [`FC_GLOBL_IDX-1:0]     fc_incr_entry; // Store this FIFO entry index in the MSHR entry
wire  [`FC_GLOBL_IDX-1:0]     fc_incr_entry_p1;
wire  [`DCP_REQ_LEN-1:0]      fc_incr_len; // To know how to interpret the request
wire  [`FC_FIFO_IDX-1:0]      fc_incr_idx;
wire  [`FC_FIFO_SIZE-1:0]     fc_incr_idx_oh;
wire  [`FC_DATA_IDX-1:0]      fc_incr_addr_lsb; // last bits of addr 
// Fifo decrease entry interface
// When Load interface tries to read from the FIFO, if will return (res == 1) if the HEAD contained valid data
// It will return the data and the entry that was read.
wire                          fc_decr_hsk;
wire                          fc_decr_val;
wire                          fc_decr_res;
wire  [`DCP_REQ_LEN-1:0]      fc_decr_len; // To know how to interpret the request
wire  [`FC_FIFO_IDX-1:0]      fc_decr_idx; 
wire  [`FC_FIFO_SIZE-1:0]     fc_decr_idx_oh; 
wire  [`FC_GLOBL_IDX-1:0]     fc_decr_entry; //unused
wire  [`FC_DATA_SIZE-1:0]     fc_decr_data0; // Data read
wire  [`FC_DATA_SIZE-1:0]     fc_decr_data1; // Data read
//Per fifo info
wire  [`FC_FIFO_SIZE-1:0]     fc_fifo_decr_len; 
wire  [`FC_FIFO_SIZE-1:0]     fc_fifo_not_empty; 
wire  [`FC_FIFO_SIZE-1:0]     fc_fifo_incr_len; 
wire  [`FC_FIFO_SIZE-1:0]     fc_fifo_full; 

//DEBUG iface
localparam FC_GLOBL_SIZE = 2 ** `FC_GLOBL_IDX;
wire [FC_GLOBL_SIZE-1:0] data_use; // assigned at fifo_ctrl
wire [FC_GLOBL_SIZE-1:0] data_val;
generate if (FC_GLOBL_SIZE >= 64) begin : range_check
    assign fc_debug_use = data_use[63:0];
    assign fc_debug_val = data_val[63:0];
end else begin : small_range
    assign fc_debug_use = {{64-FC_GLOBL_SIZE{1'b0}}, data_use};
    assign fc_debug_val = {{64-FC_GLOBL_SIZE{1'b0}}, data_val};
end endgenerate


//FIFO_management - Miss buffer
dcp_fifo_ctrl 
  #(.FC_GLOBL_IDX   (`FC_GLOBL_IDX), //index width of the fifo
    .FC_GLOBL_SIZE  (FC_GLOBL_SIZE),
    .FC_ENTRY_IDX   (`FC_ENTRY_IDX),
    .FC_DATA_LEN    (`DCP_REQ_LEN),
    .FC_DATA_IDX    (`FC_DATA_IDX),
    .FC_FIFOS_IDX   (`FC_FIFO_IDX),
    .FC_DATA_SIZE   (`FC_DATA_SIZE),
    .FC_FIFOS_SIZE  (`FC_FIFO_SIZE)
    ) u_fc (
    .fc_num_fifos   (cr_conf_fifos),
    .fc_clr_idx_oh  (s0_fifo_oh),
    .*
);
assign fc_fifo_stats = conf_op_stats ? s0_fifo : fc_decr_idx;

///////////////////
// LOAD PIPELINE //
///////////////////
// STAGE 0 /////// -> Register Load Request from NOC1
///////////////////

reg [`DCP_MSHRID_WIDTH-1:0] l0_mshr_r   [`FC_FIFO_SIZE-1:0];
reg [`FC_FIFO_SIZE-1:0]     l0_len_r;
reg [`PACKET_HOME_ID_WIDTH-1:0] l0_src_r[`FC_FIFO_SIZE-1:0];
wire [`FC_FIFO_SIZE-1:0] l0_alloc;
wire [`FC_FIFO_IDX-1:0]  l1_fifo;
wire [`FC_FIFO_SIZE-1:0] l1_fifo_oh;
wire [`FC_FIFO_SIZE-1:0] l1_read;
wire                     l1_val_sel;
wire                     l1_val;
wire                     l1_hsk;

assign fc_fifo_decr_len = l0_len_r;
wire l1_stall;

rr_arbiter #(
  .SOURCES(`FC_FIFO_SIZE),
  .MODE(1)
  ) u_ld_rr(
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (l1_stall),
  .valid_source (l0_val_r & (fc_fifo_not_empty|{`FC_FIFO_SIZE{invalidate}})),
  .arb_src_oh (l1_fifo_oh),
  .arb_src (l1_fifo),
  .arb_val (l1_val_sel)
  );

assign l0_rdy = !(|(l0_val_r & s0_fifo_oh));

generate
for ( j = 0; j < `FC_FIFO_SIZE; j = j + 1) begin: pipe_load_gen
    assign l0_alloc[j] = l0_val && s0_fifo_oh[j];
    assign l1_read [j] = l1_hsk && l1_fifo_oh[j];

    always @(posedge clk) begin
        if (!rst_n) begin
            l0_val_r[j] <= 1'b0;
            l0_len_r[j] <= 1'b0;
        end else begin
            l0_val_r[j] <= l0_alloc[j] || l0_val_r[j] && !l1_read[j];
            if (l0_alloc[j]) begin
                l0_len_r[j]  <= l0_len;
            end
        end
    end
    always @(posedge clk) begin
        if (l0_alloc[j]) begin
            l0_mshr_r[j] <= pipe_mshr;
            l0_src_r[j]  <= pipe_src;
        end
    end
end
endgenerate

/////////////////
//// STAGE 1 //// -> Read from the FIFO
/////////////////
reg                          l0_requested_r;
reg                          l1_val_r;
reg [`DCP_DATA_WIDTH-1:0]    l0_data_r;
reg [`DCP_DATA_WIDTH-1:0]    l1_data_r;
reg [`DCP_MSHRID_WIDTH-1:0]  l1_mshr_r;
reg [`PACKET_HOME_ID_WIDTH-1:0] l1_src_r;

// Wires
wire [`DCP_DATA_WIDTH-1:0] l0_data;
wire [`DCP_DATA_WIDTH-1:0] l1_data;
wire [`DCP_MSHRID_WIDTH-1:0] l1_mshr;
wire [`PACKET_HOME_ID_WIDTH-1:0] l1_src;
wire l1_rdy;
wire l1_requested;
wire l1_req_val,l1_req_rdy,l1_req_hsk;

assign l1_stall = !(l1_requested && l1_rdy); 
assign l1_req_val = l1_val_sel && !l0_requested_r;
assign l1_req_rdy = fc_decr_res || invalidate;

assign fc_incr_hsk = fc_incr_val && fc_incr_res;
assign fc_decr_hsk = fc_decr_val && fc_decr_res;
assign fc_decr_val = l1_req_val && !invalidate;
assign fc_decr_len = |(l0_len_r & l1_fifo_oh);
assign fc_decr_idx = l1_fifo;
assign fc_decr_idx_oh = l1_fifo_oh;

assign l1_requested = l1_req_hsk || l0_requested_r;
assign l1_val = l1_val_sel && l1_requested;
assign l1_req_hsk = l1_req_val && l1_req_rdy;

assign l1_rdy = !l1_val_r || l2_rdy;
assign l1_hsk = l1_val && l1_rdy;

assign l0_data = {fc_decr_data0, fc_decr_data1};
assign l1_data = l1_req_hsk ? l0_data : l0_data_r;
assign l1_mshr = l0_mshr_r[l1_fifo];
assign l1_src  = l0_src_r[l1_fifo];

always @(posedge clk) begin
    if (!rst_n) begin
        l0_requested_r <= 1'b0;
    end else begin
        l0_requested_r <= l1_requested && !l1_hsk;
    end
end
always @(posedge clk) begin
    if (l1_req_hsk) begin
        l0_data_r <= l0_data;
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        l1_val_r <= 1'b0;
    end else begin
        l1_val_r <= l1_hsk || l1_val_r && !l2_rdy;
    end
end

always @(posedge clk) begin
    if (l1_hsk) begin
        l1_data_r  <= l1_data;
        l1_mshr_r  <= l1_mshr;
        l1_src_r   <= l1_src;
    end
end

/////////////////
//// STAGE 2 //// -> Response to the NOC2
/////////////////
assign l2_val = l1_val_r;
assign l2_mshr = l1_mshr_r;
assign l2_data = l1_data_r;
assign l2_src = l1_src_r;

/////////////////////
// STORE PIPELINE////
/////////////////////
// STAGE 0 /// -> Register Store requests
/////////////////////

reg [`FC_FIFO_SIZE-1:0] s0_loop_r;
reg [`FC_FIFO_SIZE-1:0] s0_double_r;
reg [`DCP_MSHRID_WIDTH-1:0] s0_mshr_r   [`FC_FIFO_SIZE-1:0];
reg [`FC_FIFO_SIZE    -1:0] s0_len_r;
reg [`DCP_OPCODE_WIDTH-1:0] s0_op_r     [`FC_FIFO_SIZE-1:0];
reg [`DCP_VADDR       -1:0] s0_addr_r   [`FC_FIFO_SIZE-1:0];
reg [31:0] s0_data1_r   [`FC_FIFO_SIZE-1:0];
reg [`DCP_VADDR       -1:0] s0_data2_r  [`FC_FIFO_SIZE-1:0];
reg [`PACKET_HOME_ID_WIDTH-1:0] s0_src_r[`FC_FIFO_SIZE-1:0];

// We do not need the core id where the transaction came from because we have it from the core config msg.
wire                     s0_hsk, s1_hsk;
wire [`FC_FIFO_IDX -1:0] s1_fifo;
wire [`FC_FIFO_SIZE-1:0] s1_fifo_oh;
wire [`FC_FIFO_SIZE-1:0] s1_read;
wire [`FC_FIFO_SIZE-1:0] s0_alloc;
wire                     s1_val_sel;

// New load request coming from NOC1
assign s0_rdy  = !(|(s0_val_r & s0_fifo_arb));
assign s0_hsk = s0_val && s0_rdy;

wire s1_stall;
assign fc_fifo_incr_len = s0_len_r;
rr_arbiter #(
  .SOURCES(`FC_FIFO_SIZE),
  .MODE(1)
  ) u_st_rr(
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (s1_stall),
  .valid_source      (s0_val_r & (~fc_fifo_full|{`FC_FIFO_SIZE{invalidate}})),
  .arb_src_oh (s1_fifo_oh),
  .arb_src (s1_fifo),
  .arb_val (s1_val_sel)
  );

generate
for ( j = 0; j < `FC_FIFO_SIZE; j = j + 1) begin : pipe_store_gen
    assign s0_alloc[j] = s0_hsk && s0_fifo_arb[j];
    assign s1_read [j] = s1_hsk && s1_fifo_oh[j];

    always @(posedge clk) begin
        if (!rst_n) begin
            s0_val_r[j]    <= 1'b0;
            s0_double_r[j] <= 1'b0;
            s0_len_r[j]    <= 1'b0;
            s0_op_r [j]    <= {`DCP_OPCODE_WIDTH{1'b0}};
        end else begin
            s0_val_r[j]    <= s0_alloc[j] || s0_val_r[j] && !(s1_read[j] && !s0_double_r[j]);
            s0_double_r[j] <= s0_alloc[j] && s0_double_load || s0_double_r[j] && !s1_read[j];
            if (s0_alloc[j]) begin 
                s0_len_r[j]  <= pipe_len;
                s0_op_r [j]  <= pipe_op;
            end
        end
    end
    //CAS2 and the previous one wasn't CAS1, so the CAS2 contains all data compressed
    wire noc1_cmp = (pipe_op == `DCP_AMO_OP_CAS2) && (s0_op_r[j]!=`DCP_AMO_OP_CAS1);
    always @(posedge clk) begin
        if (!rst_n) begin
            s0_addr_r[j]   <= {`DCP_VADDR{1'b0}};
            s0_data1_r[j]  <= 32'd0;
            s0_data2_r[j]  <= {`DCP_VADDR{1'b0}};
        end else begin
            if (s0_alloc[j] && s0_addr_val) begin
                s0_addr_r[j] <= noc1_addr;
            end
            if (s0_alloc[j] && (s0_data1_val || noc1_cmp) ) begin
                s0_data1_r[j] <= s0_data1_val ? s0_data1 : noc1_cmp_data1;
            end
            if (s0_alloc[j] && (s0_data2_val || noc1_cmp) ) begin
                s0_data2_r[j] <= s0_data2_val ? s0_data2 : noc1_cmp_data2;
            end
        end
    end
    always @(posedge clk) begin
        if (s0_alloc[j]) begin
            s0_loop_r[j] <= pipe_loop;
            s0_mshr_r[j] <= pipe_mshr;
            s0_src_r [j] <= pipe_src;
        end
    end
end endgenerate

/////////////////
//// STAGE 1 ////  -> Reserve FIFO slots
/////////////////
reg                  s0_requested_r;
reg                  s0_tlb_requested_r;
reg                  s1_val_r;
reg [`DCP_PADDR-1:0] s1_addr_r;

reg s1_loop_r;
reg [`DCP_MSHRID_WIDTH -1:0] s1_mshr_r;
reg [`FC_FIFO_IDX      -1:0] s1_fifo_r;
reg [`FC_GLOBL_IDX -1:0] s0_entry_r;
reg [`FC_GLOBL_IDX -1:0] s0_entry_p1_r;
reg [`FC_GLOBL_IDX -1:0] s1_entry_r;
reg [`FC_GLOBL_IDX -1:0] s1_entry_p1_r;
reg [`DCP_REQ_LEN      -1:0] s1_len_r; // To know how to interpret the request
reg [`DCP_OPCODE_WIDTH-1:0]  s1_op_r;
reg [`PACKET_HOME_ID_WIDTH-1:0] s1_src_r;
reg                          s1_requested_r;
reg  [31:0] s1_data1_r;
reg  [`DCP_VADDR-1:0] s1_data2_r;

reg [`DCP_PADDR-12-1:0]       s0_ppn_r;

wire                          s1_val;
wire                          s1_addr_val,s1_req_real,s1_req_val;
wire                          s1_req_rdy,s1_req_hsk;
wire                          s1_tlb_real,s1_tlb_val,s1_tlb_rdy, s1_tlb_hsk;
wire                          s1_rdy;
wire [`DCP_MSHRID_WIDTH-1:0]  s1_mshr;
wire [`DCP_OPCODE_WIDTH-1:0]  s1_op;
wire                          s1_rmw, s1_tload, s1_prefetch, s1_cas1;
wire [`PACKET_HOME_ID_WIDTH-1:0] s1_src; 
wire [`DCP_VADDR           -1:0] s1_vaddr;
wire [`DCP_VADDR           -1:0] s1_second_vaddr;
wire                             s1_double;
wire [31:0]                      s1_data1;
wire [`DCP_VADDR-1:0]            s1_data2;

wire s2_hsk,s2_requested;
//Check if stage0 is completed
wire s1_requested, s1_tlb_requested, s1_completed;

//Operations which are requesting an (prefetch included in rmw range)
assign s1_addr_val = s1_rmw && !s1_cas1 || s1_tload;
//Reserve fifo slot unless cas1 or invalidate, or prefetch
assign s1_req_real = !s1_cas1 && !s1_prefetch && !invalidate;
assign s1_tlb_real = s1_addr_val && !invalidate;

assign tlb_maple_req = s1_tlb_val && s1_tlb_real;
assign tlb_maple_vpage = s1_vaddr[`DCP_VADDR-1:12];

//Only allocate an entry in the fifo if real transaction
assign fc_incr_val = s1_req_val && s1_req_real;
assign fc_incr_len = |(s0_len_r & s1_fifo_oh);
assign fc_incr_addr_lsb = s1_vaddr[2+:`FC_DATA_IDX] & {`FC_DATA_IDX{s1_addr_val}}; //Last bits of the addr
assign fc_incr_idx = s1_fifo;
assign fc_incr_idx_oh = s1_fifo_oh;

assign s1_requested = s1_req_hsk || s0_requested_r;
assign s1_tlb_requested = s1_tlb_hsk || s0_tlb_requested_r;
assign s1_completed = s1_requested && s1_tlb_requested;
assign s1_val = s1_val_sel && s1_completed;

//Reserve FIFO slot, req_val until its requested_r, rdy high
//when handshake or if there was not a request (!req_real)
assign s1_req_val = s1_val_sel && !s0_requested_r;
assign s1_req_rdy = fc_incr_res || !s1_req_real;
assign s1_req_hsk = s1_req_val && s1_req_rdy;

//Request TLB translation
assign s1_tlb_val = s1_val_sel && !s0_tlb_requested_r;
assign s1_tlb_rdy = tlb_maple_ack || !s1_tlb_real;
assign s1_tlb_hsk = s1_tlb_val && s1_tlb_rdy;

assign s1_rdy  = !s1_val_r || s2_hsk;
assign s1_hsk  = s1_val && s1_rdy;
assign s1_stall = !(s1_completed && s1_rdy);

assign s1_data1 = s0_data1_r[s1_fifo];
assign s1_data2 = s0_data2_r[s1_fifo];
assign s1_double = s0_double_r[s1_fifo];
assign s1_vaddr = s1_double ? s1_data2 : s0_addr_r[s1_fifo];
assign s1_src   = s0_src_r[s1_fifo];
assign s1_mshr  = s0_mshr_r[s1_fifo];
assign s1_op    = s0_op_r[s1_fifo];
assign s1_rmw   = s1_op[`DCP_OPCODE_WIDTH-1]; //MSB -> RMW
assign s1_cas1  = s1_op == `DCP_AMO_OP_CAS1;
assign s1_tload = (s1_op == `DCP_TLOAD32_OP) || (s1_op == `DCP_TLOAD64_OP);
assign s1_prefetch = (s1_op == `DCP_PREFETCH);

always @(posedge clk) begin
    if (!rst_n) begin
        s0_requested_r     <= 1'b0;
        s0_tlb_requested_r <= 1'b0;
    end else begin
        s0_requested_r     <= s1_requested && !s1_hsk;
        s0_tlb_requested_r <= s1_tlb_requested && !s1_hsk;
    end
end
always @(posedge clk) begin
    if (s1_req_hsk) begin
        s0_entry_r    <= fc_incr_entry;
        s0_entry_p1_r <= fc_incr_entry_p1;
    end
    if (s1_tlb_hsk) begin
        s0_ppn_r <= tlb_ppn;
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        s1_val_r       <= 1'b0;
        s1_requested_r <= 1'b0;
        s1_addr_r      <= {`DCP_PADDR{1'b0}};
    end else begin
        s1_val_r       <= s1_hsk || s1_val_r && !s2_hsk;
        s1_requested_r <= s2_requested && !s2_hsk;
        if (s1_hsk) begin
            s1_addr_r  <= {s1_tlb_hsk ? tlb_ppn : s0_ppn_r, s1_vaddr[11:0]};
        end
    end
end

always @(posedge clk) begin
    if (s1_hsk) begin
        s1_data1_r  <= s1_data1;
        s1_data2_r  <= s1_data2;
        s1_entry_r    <= s1_req_hsk ? fc_incr_entry : s0_entry_r;
        s1_entry_p1_r <= s1_req_hsk ? fc_incr_entry_p1 : s0_entry_p1_r;
        s1_loop_r  <= s0_loop_r[s1_fifo] || s1_double;
        s1_mshr_r  <= s1_mshr;
        s1_fifo_r  <= s1_fifo;
        s1_len_r   <= fc_incr_len;
        s1_op_r    <= s1_op & {`DCP_OPCODE_WIDTH{!invalidate}};
        s1_src_r   <= s1_src;
    end
end

//////////////////////////
//// STAGE 2 //// -> Make request if TLOAD OR DCP
///////////////////////////
reg                          s2_val_r;
reg                          s2_loop_r;
reg [`DCP_MSHRID_WIDTH-1:0]  s2_mshr_r;
reg [`DCP_OPCODE_WIDTH-1:0]  s2_op_r;
reg [`PACKET_HOME_ID_WIDTH-1:0] s2_src_r;

wire no_ack = s2_loop_r;
wire s2_val;
wire s2_rdy;
wire s3_finish;

wire   s2_op_cas1     = (s2_op == `DCP_AMO_OP_CAS1);
assign s2_op_prefetch = (s2_op == `DCP_PREFETCH);
assign s2_op_tload_32 = (s2_op == `DCP_TLOAD32_OP);
assign s2_op_tload_64 = (s2_op == `DCP_TLOAD64_OP);
assign s2_op_tload = (s2_op_tload_32 || s2_op_tload_64);
assign s2_op_prod  = (s2_op == `DCP_PROD_OP);
assign s2_op_rmw   = s2_op[`DCP_OPCODE_WIDTH-1] && !s2_op_cas1;

assign s2_data = {s1_data2_r[31:0],s1_data1_r};
assign s2_requested = s2_req && s2_req_rdy || s1_requested_r;
assign s2_val = s1_val_r && s2_requested; //we can progress if already requested
assign s2_req = s1_val_r && !s1_requested_r;
assign s2_hsk = s2_val && s2_rdy;
assign s2_src = s1_src_r;
assign s2_addr = s1_addr_r;
assign s2_4byte_align = s1_addr_r[2]; //3rd bit of the addr (Little endian)

always @(posedge clk) begin
    if (!rst_n) begin
        s2_val_r   <= 1'b0;
        s2_loop_r  <= 1'b0;
        s2_mshr_r  <= {`DCP_MSHRID_WIDTH{1'b0}};
        s2_op_r    <= {`DCP_OPCODE_WIDTH{1'b0}};
        s2_src_r   <= {`PACKET_HOME_ID_WIDTH{1'b0}};
    end else begin
        s2_val_r <= s2_hsk || s2_val_r && !s3_finish;
        if (s2_hsk) begin
            s2_loop_r  <= s1_loop_r;
            s2_mshr_r  <= s2_mshr;
            s2_op_r    <= s2_op;
            s2_src_r   <= s2_src;
        end
    end
end

assign s2_op    = s1_op_r;
assign s2_mshr  = s1_mshr_r;
assign s2_entry = s1_entry_r;
assign s2_entry_p1 = s1_entry_p1_r;
assign s2_len   = s1_len_r;
assign s2_rdy = !s2_val_r || s3_finish;
assign s3_finish = s3_rdy || no_ack;

//////////////////////////
//// STAGE 3 //// -> ACK the initial Store request
/////////////////////////// 
assign s3_val = s2_val_r && !no_ack;
assign s3_mshr = s2_mshr_r;
assign s3_src = s2_src_r;


/////////////////////
///// COUNTERS //////
/////////////////////
generate if (`DCP_PERF_COUNTERS) begin : perf_counters
    reg  [`DCP_COUNT_TRANS-1:0] count_transactions_r;
    reg  [`DCP_COUNT_TRANS-1:0] count_fifo_transactions_r [`FC_FIFO_SIZE-1:0];
    reg  [`DCP_COUNT_STALL-1:0] count_st_fifo_stall_r     [`FC_FIFO_SIZE-1:0];
    reg  [`DCP_COUNT_STALL-1:0] count_ld_fifo_stall_r     [`FC_FIFO_SIZE-1:0];
    reg  [`DCP_COUNT_LAT-1:0] count_st_fifo_r             [`FC_FIFO_SIZE-1:0];
    reg  [`DCP_COUNT_LAT-1:0] count_ld_fifo_r             [`FC_FIFO_SIZE-1:0];
    reg  [`DCP_COUNT_STALL-1:0] count_dram_r;
    reg  [`DCP_COUNT_STALL-1:0] count_dram_trans_r;

    wire sat_transactions;
    wire [`DCP_COUNT_TRANS-1:0] count_transactions_nxt;
    assign {sat_transactions, count_transactions_nxt} = {1'b0, count_transactions_r} + {{`DCP_COUNT_TRANS{1'b0}}, 1'b1};
    
    // COUNT TOTAL TRANSACTIONS///
    wire count_transactions_set = fc_decr_hsk && !sat_transactions;
    always @(posedge clk) begin
        if (!rst_n) begin
            count_transactions_r  <= {`DCP_COUNT_TRANS{1'b0}};
        end else if (count_transactions_set || count_transactions_clr) begin
            count_transactions_r  <= count_transactions_nxt & {`DCP_COUNT_TRANS{!count_transactions_clr}};
        end
    end

    ///COUNT NOC2 STALL///
    reg  [`DCP_COUNT_STALL-1:0] count_noc2_stall_r;
    wire [`DCP_COUNT_STALL-1:0] count_noc2_stall_nxt = count_noc2_stall_r + {{`DCP_COUNT_STALL-1{1'b0}},1'b1};

        
    always @(posedge clk) begin
        if (!rst_n) begin
            count_noc2_stall_r <= {`DCP_COUNT_STALL{1'b0}};
        end else if (count_noc2_stall_clk) begin
            count_noc2_stall_r <= count_noc2_stall_nxt & {`DCP_COUNT_STALL{!count_noc2_stall_clr}};
        end
    end
    /*
    ///COUNT DRAM LATENCY for MSHRID #0///
    reg dram_ongoing_r;
    wire req_id = !(|dcp_noc2buffer_mshrid);
    wire res_id = !(|noc3decoder_dcp_mshrid);
    wire new_dram = dcp_noc2buffer_hsk && req_id;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            count_dram_r <= {`DCP_COUNT_STALL{1'b0}};
            count_dram_trans_r <= {`DCP_COUNT_STALL{1'b0}};
            dram_ongoing_r <= 1'b0;
        end else begin
            dram_ongoing_r <= new_dram || dram_ongoing_r && !(noc3decoder_dcp_val && res_id);
            if (dram_ongoing_r) count_dram_r <= count_dram_r + {{`DCP_COUNT_STALL-1{1'b0}},1'b1};
            if (new_dram) count_dram_trans_r <= count_dram_trans_r + {{`DCP_COUNT_STALL-1{1'b0}},1'b1};
        end
    end
    */
    //////////////////////////////////////////////////////////
    // Count cycles that ld/st pipe is stalled due to the FIFO
    for ( j = 0; j < `FC_FIFO_SIZE; j = j + 1) begin: count_stall_gen
        wire sat_ld_fifo_stall;
        wire sat_st_fifo_stall;
        wire ld_hsk = fc_decr_hsk && fc_decr_idx_oh[j];
        wire st_hsk = fc_incr_hsk && fc_incr_idx_oh[j];
        wire [`DCP_COUNT_STALL-1:0] count_ld_fifo_stall_nxt;
        wire [`DCP_COUNT_STALL-1:0] count_st_fifo_stall_nxt;
        assign {sat_ld_fifo_stall, count_ld_fifo_stall_nxt} = {1'b0, count_ld_fifo_stall_r[j]} + {{`DCP_COUNT_STALL{1'b0}},1'b1};
        assign {sat_st_fifo_stall, count_st_fifo_stall_nxt} = {1'b0, count_st_fifo_stall_r[j]} + {{`DCP_COUNT_STALL{1'b0}},1'b1};
        wire count_ld_stall_clr = c0_invalidate;// || st_hsk;
        wire count_st_stall_clr = c0_invalidate;// || st_hsk
        wire count_ld_fifo_stall_clk = l0_val_r[j] && !sat_ld_fifo_stall || count_ld_stall_clr;
        wire count_st_fifo_stall_clk = s0_val_r[j] && !sat_st_fifo_stall || count_st_stall_clr;
        
        wire count_ld_clr = c0_invalidate;
        wire count_st_clr = c0_invalidate;
        wire count_ld_fifo_clk = c0_econfigured_r[j] && !l0_val_r[j] || count_ld_clr;
        wire count_st_fifo_clk = c0_aconfigured_r[j] && !s0_val_r[j] || count_st_clr;
        wire [`DCP_COUNT_LAT-1:0] count_ld_fifo_nxt = count_ld_fifo_r[j] + {{`DCP_COUNT_LAT-1{1'b0}},1'b1};
        wire [`DCP_COUNT_LAT-1:0] count_st_fifo_nxt = count_st_fifo_r[j] + {{`DCP_COUNT_LAT-1{1'b0}},1'b1};
        
        always @(posedge clk) begin
            if (!rst_n) begin
                count_ld_fifo_stall_r[j] <= {`DCP_COUNT_STALL{1'b0}};
                count_st_fifo_stall_r[j] <= {`DCP_COUNT_STALL{1'b0}};
                count_st_fifo_r[j] <= {`DCP_COUNT_LAT{1'b0}};
                count_ld_fifo_r[j] <= {`DCP_COUNT_LAT{1'b0}};
            end else begin
                if (count_ld_fifo_stall_clk)
                    count_ld_fifo_stall_r[j] <= count_ld_fifo_stall_nxt & {`DCP_COUNT_STALL{!count_ld_stall_clr}};
                if (count_st_fifo_stall_clk)
                    count_st_fifo_stall_r[j] <= count_st_fifo_stall_nxt & {`DCP_COUNT_STALL{!count_st_stall_clr}};
    
                if (count_ld_fifo_clk)
                    count_ld_fifo_r[j] <= count_ld_fifo_nxt & {`DCP_COUNT_LAT{!count_ld_clr}};
                if (count_st_fifo_clk)
                    count_st_fifo_r[j] <= count_st_fifo_nxt & {`DCP_COUNT_LAT{!count_st_clr}};
            end
        end
    
        wire [`DCP_COUNT_TRANS-1:0] count_transactions_nxt = count_fifo_transactions_r[j] +
                                            {{`DCP_COUNT_TRANS-2{1'b0}}, fc_decr_len, !fc_decr_len};
        always @(posedge clk) begin
            if (!rst_n) begin
                count_fifo_transactions_r[j]  <= {`DCP_COUNT_TRANS{1'b0}};
            end else if (ld_hsk || count_ld_clr) begin
                count_fifo_transactions_r[j]  <= count_transactions_nxt & {`DCP_COUNT_TRANS{!count_ld_clr}};
            end
        end //always
    end //for

    assign count_st_fifo_stall = count_st_fifo_stall_r[fc_fifo_stats];
    assign count_ld_fifo_stall = count_ld_fifo_stall_r[fc_fifo_stats];
    assign count_st_fifo = count_st_fifo_r[fc_fifo_stats];
    assign count_ld_fifo = count_ld_fifo_r[fc_fifo_stats];
    assign count_fifo_transactions = count_fifo_transactions_r[fc_fifo_stats];
    assign count_transactions = count_transactions_r;
end else begin : no_perf
    assign count_st_fifo_stall = `DCP_COUNT_STALL'd0;
    assign count_ld_fifo_stall = `DCP_COUNT_STALL'd0;
    assign count_st_fifo = `DCP_COUNT_LAT'd0;
    assign count_ld_fifo = `DCP_COUNT_LAT'd0;
    assign count_fifo_transactions = `DCP_COUNT_TRANS'd0;
    assign count_transactions = `DCP_COUNT_TRANS'd0;
end endgenerate
assign count_dram_cycles = `DCP_COUNT_STALL'd0;
assign count_dram_trans  = `DCP_COUNT_STALL'd0;

assign pipe_debug = {no_ack, s2_val, s1_val_r,
                     s1_val, s1_val_sel, s1_fifo, s0_len_r, fc_fifo_full, s0_val_r,
                     l1_val, l1_val_sel, l1_fifo, l0_len_r, fc_fifo_not_empty, l0_val_r,
                     fc_incr_val, fc_incr_entry,fc_decr_val,fc_decr_entry};

////// ASSERTIONS ///////////////
`ifdef DEC_ASSERT_ON

`endif
endmodule
