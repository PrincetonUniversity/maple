/*
Copyright (c) 2019 Princeton University
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
//  Filename      : dcp_pipe.v
//  Created On    : 2019-11-30
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Pipeline for DCP unit, buffers messages from the network, reserve MSHR entry, issue request to NOC1,
//                  receive response from NOC2, store data into MSHR corresponding entry, issue data in order to FIFO
//
//==================================================================================================

//`timescale 1 ns / 10 ps
`include "dcp.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif

module dcp_pipe (
    input wire clk,
    input wire rst_n,
    input wire [`HOME_ALLOC_METHOD_WIDTH-1:0] home_alloc_method,
    input wire [`HOME_ID_WIDTH-1:0] system_tile_count,
    
    // MAPLE uses AutoSVA for automatic formal verification
    // Check out the repo at https://github.com/PrincetonUniversity/AutoSVA

    /*AUTOSVA
    ls_iface: store --NOT> load
    store_val = dcp_noc2buffer_val && dcp_noc2buffer_type == `DCP_NOC2_STORE_ACK
    store_rdy = dcp_noc2buffer_rdy
    load_val = dcp_noc2buffer_val && dcp_noc2buffer_type == `DCP_NOC2_LOAD_ACK
    load_rdy = dcp_noc2buffer_rdy

    exc_val: tlb_exc --OUT> tlb_get
    tlb_exc_transid = tlb_ptw_src[1]
    tlb_get_val = dcp_pipe.tlb_get_pfault
    tlb_get_transid = dcp_pipe.tlb_exc_src

    exc_req: tlb_get --OUT> tlb_set
    [1:0] tlb_get_data = dcp_pipe.tlb_exc_src_oh
    tlb_set_val = dcp_pipe.tlb_conf_mmpage
    tlb_set_transid = dcp_pipe.tlb_mmpage_src_oh[1]
    [1:0] tlb_set_data = dcp_pipe.tlb_mmpage_src_oh
    
    noc1in: noc1decoder_dcp --IN> dcp_noc2bufferack
    dcp_noc2bufferack_val = dcp_noc2buffer_val && (dcp_noc2buffer_type==`DCP_NOC2_STORE_ACK || dcp_noc2buffer_type == `DCP_NOC2_LOAD_ACK)
    dcp_noc2bufferack_rdy = dcp_noc2buffer_rdy
    [`DCP_MSHRID_WIDTH-1:0] dcp_noc2bufferack_transid = dcp_noc2buffer_mshrid
    
    noc1out: dcp_noc1bufferout --OUT> noc2decoder_dcp
    dcp_noc1bufferout_val = dcp_noc1buffer_val
    dcp_noc1bufferout_rdy = dcp_noc1buffer_rdy
    [`DCP_MSHRID_WIDTH-1:0] dcp_noc1bufferout_transid = dcp_noc1buffer_mshrid[`DCP_MSHRID_WIDTH-1:0]
    [`DCP_MSHRID_WIDTH-1:0] noc2decoder_dcp_transid = noc2decoder_dcp_mshrid[`DCP_MSHRID_WIDTH-1:0]

    noc2out: dcp_noc2bufferout --OUT> noc3decoder_dcp
    dcp_noc2bufferout_val = dcp_noc2buffer_val && dcp_noc2buffer_type == `DCP_NOC2_LOAD_REQ64
    dcp_noc2bufferout_rdy = dcp_noc2buffer_rdy
    [`DCP_MSHRID_WIDTH-1:0] dcp_noc2bufferout_transid = dcp_noc2buffer_mshrid
    */

    // Update TLB from DCP
    output wire                            tlb_update,
    output wire                            tlb_conf_ptbase,
    output wire                            tlb_disable,
    output wire                            tlb_flush,
    output wire [63:0]                     conf_data,
    
    // TLB req/res iface
    output wire                            tlb_req,
    input  wire                            tlb_ack,
    input  wire                            tlb_exc_val,
    input  wire [`TLB_SRC_NUM  -1:0]       tlb_ptw_src,
    output wire [`DCP_VADDR    -1:0]       tlb_vaddr,
    input  wire [`DCP_PADDR    -1:0]       tlb_paddr,

    // Snoop TLB entries from DCP
    output wire                            tlb_snoop_val,
    input  wire [63:0]                     tlb_snoop_entry, 

    // NOC1 - Outgoing Atomic op request to L2 
    input  wire                          dcp_noc1buffer_rdy,
    output wire                          dcp_noc1buffer_val,
    output wire [`MSG_TYPE_WIDTH-1:0]    dcp_noc1buffer_type,
    output wire [`DCP_MSHRID_WIDTH -1:0] dcp_noc1buffer_mshrid,
    output wire [`DCP_PADDR_MASK       ] dcp_noc1buffer_address,
    output wire [`DCP_UNPARAM_2_0      ] dcp_noc1buffer_size,
    output wire [`PACKET_HOME_ID_WIDTH-1:0] dcp_noc1buffer_homeid,
    output wire [`MSG_AMO_MASK_WIDTH-1:0] dcp_noc1buffer_write_mask,
    output wire [`DCP_UNPARAM_63_0     ] dcp_noc1buffer_data_0,
    output wire [`DCP_UNPARAM_63_0     ] dcp_noc1buffer_data_1,
    
    // NOC2 - Outgoing TLoad op request to DRAM or Store ACK 
    input  wire                          dcp_noc2buffer_rdy,
    output wire                          dcp_noc2buffer_val,
    output wire [`MSG_TYPE_WIDTH   -1:0] dcp_noc2buffer_type,
    output wire [`DCP_MSHRID_WIDTH -1:0] dcp_noc2buffer_mshrid,
    output wire [`DCP_PADDR_MASK       ] dcp_noc2buffer_address,
    output wire [`DCP_UNPARAM_63_0     ] dcp_noc2buffer_data,
    output wire [`PACKET_HOME_ID_WIDTH-1:0] dcp_noc2buffer_homeid,
    output wire [`MSG_SRC_FBITS_WIDTH -1:0] dcp_noc2buffer_fbits,

    // NOC1 - Incoming Load/Store requests
    input  wire                          noc1decoder_dcp_val,
    output wire                          noc1decoder_dcp_ack,
    ///AUTOSVA trans_id
    input  wire [`DCP_MSHRID_WIDTH -1:0] noc1decoder_dcp_mshrid,
    input  wire [`MSG_TYPE_WIDTH   -1:0] noc1decoder_dcp_reqtype,
    input  wire [`DCP_UNPARAM_63_0     ] noc1decoder_dcp_data,
    input  wire [`DCP_PADDR_MASK       ] noc1decoder_dcp_address,
    input  wire [`MSG_DATA_SIZE_WIDTH-1:0]  noc1decoder_dcp_size,  
    input  wire [`MSG_SRC_X_WIDTH  -1:0]    noc1decoder_dcp_src_x,  
    input  wire [`MSG_SRC_Y_WIDTH  -1:0]    noc1decoder_dcp_src_y,  
    input  wire [`MSG_SRC_CHIPID_WIDTH-1:0] noc1decoder_dcp_chipid,
    input  wire [`MSG_SRC_FBITS_WIDTH- 1:0] noc1decoder_dcp_fbits,

    // NOC2 - Atomic op response from L2
    input  wire                          noc2decoder_dcp_val,
    output wire                          noc2decoder_dcp_ack,
    input  wire [`DCP_MSHRID_WIDTH -1:0] noc2decoder_dcp_mshrid,
    input  wire [`MSG_LENGTH_WIDTH -1:0] noc2decoder_dcp_length,
    input  wire [`MSG_TYPE_WIDTH   -1:0] noc2decoder_dcp_reqtype,
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] noc2decoder_dcp_data,

    // NOC3 - TLoad response from DRAM
    input  wire                          noc3decoder_dcp_val,
    output wire                          noc3decoder_dcp_ack,
    ///AUTOSVA trans_id
    input  wire [`DCP_MSHRID_WIDTH -1:0] noc3decoder_dcp_mshrid,
    input  wire [`MSG_TYPE_WIDTH   -1:0] noc3decoder_dcp_reqtype,
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] noc3decoder_dcp_data
);

genvar j;
localparam TILEDIM_BITS = 4;
localparam TILEID_BITS = TILEDIM_BITS*2;
/////////////////////////////////////////
// Fifo management interface
////////////////////////////////////////
wire dcp_noc1buffer_hsk;
wire dcp_noc2buffer_hsk;
assign  dcp_noc1buffer_hsk = dcp_noc1buffer_val && dcp_noc1buffer_rdy;
assign  dcp_noc2buffer_hsk = dcp_noc2buffer_val && dcp_noc2buffer_rdy;

wire [`PACKET_HOME_ID_WIDTH-1:0] noc1decoder_dcp_src;
wire [TILEID_BITS-1:0] noc1decoder_dcp_tile = {noc1decoder_dcp_src_y[TILEDIM_BITS-1:0],noc1decoder_dcp_src_x[TILEDIM_BITS-1:0]};
assign noc1decoder_dcp_src = {noc1decoder_dcp_chipid, noc1decoder_dcp_src_y,noc1decoder_dcp_src_x};

// Add Fifos
wire                          fc_add_val;
wire                          fc_add_hsk;
wire  [`FC_GLOBL_IDX-1:0]     fc_add_size;
wire  [`FC_FIFO_IDX-1:0]      fc_add_idx;
// Remove Fifos
wire                          fc_clr_val;

// Data fill interface
wire                        fc_fill_val;
wire  [`FC_GLOBL_IDX  -1:0] fc_fill_entry;
wire  [`FC_DATA_SIZE*2-1:0] fc_fill_data;
wire  [`FC_DATA_IDX   -1:0] fc_fill_addr_lsb;
wire                        fc_st_empty;
wire [`FC_FIFO_SIZE   -1:0] fc_fifo_val;

wire invalidate;
wire c0_invalidate,c0_hsk;
wire [63:0] c0_data;

/////////////////////
// TLB config/flush
/////////////////////
wire tlb_conf,tlb_poll;
wire tlb_conf_mmpage,tlb_conf_entry;
wire tlb_get_pfault;

// TLB ARBITRATION //
wire [`DCP_PADDR-12-1:0] tlb_ppn = tlb_paddr[`DCP_PADDR-1:12];
wire [`DCP_VADDR-12-1:0] tlb_exc_vpage;

reg  [`TLB_SRC_NUM-1:0] tlb_exc_vec_r,tlb_req_vec_r;
wire [`TLB_SRC_IDX-1:0] tlb_src,tlb_exc_src;
wire [`TLB_SRC_NUM-1:0] tlb_src_oh,tlb_exc_src_oh,tlb_mmpage_src_oh;
wire [`TLB_SRC_NUM-1:0] tlb_exc_src_req,tlb_req_src_ack;
wire [`TLB_SRC_NUM-1:0] tlb_exc_masked = tlb_exc_vec_r | tlb_req_vec_r;
wire tlb_exc_req,tlb_exc_ack;

wire tlb_maple_req;
wire tlb_maple_ack = tlb_ack && tlb_src_oh[0];
wire tlb_cr_req;
wire tlb_cr_ack = tlb_ack && tlb_src_oh[1];
wire [`DCP_VADDR-12-1:0] tlb_maple_vpage;
wire [`DCP_VADDR-12-1:0] tlb_cr_vpage;
assign tlb_vaddr = {tlb_src ? tlb_cr_vpage : tlb_maple_vpage, {12-`TLB_SRC_NUM{1'b0}}, tlb_src_oh };
assign tlb_exc_vpage = {`DCP_VADDR-12{tlb_exc_src_oh[1]}} & tlb_cr_vpage | 
                       {`DCP_VADDR-12{tlb_exc_src_oh[0]}} & tlb_maple_vpage;
wire [63:0] tlb_exc_vaddr = { {64-`DCP_VADDR{1'b0}}, tlb_exc_vpage & {`DCP_VADDR-12{tlb_get_pfault}}, 
                              {12-`TLB_SRC_NUM{1'b0}}, tlb_exc_src_req };

// NOTE: This RR only accepts pow2, pad with sources zeros if not pow2!
rr_arbiter #(
  .SOURCES(`TLB_SRC_NUM)
  ) u_tlb_rr (
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (!tlb_ack),
  .valid_source ({tlb_cr_req,tlb_maple_req} & ~tlb_exc_masked ),
  .arb_src_oh (tlb_src_oh),
  .arb_src (tlb_src),
  .arb_val (tlb_req)
  );
rr_arbiter #(
  .SOURCES(`TLB_SRC_NUM)
) u_tlb_exc_rr (
  .clk     (clk),
  .reset_n (rst_n),
  .stall   (!tlb_get_pfault),
  .valid_source (tlb_exc_vec_r),
  .arb_src_oh (tlb_exc_src_oh),
  .arb_src (tlb_exc_src),
  .arb_val (tlb_exc_req)
  );

assign tlb_mmpage_src_oh = c0_data[`TLB_SRC_NUM-1:0]; // last 4 bits of response is the source
assign tlb_exc_src_req = {`TLB_SRC_NUM{tlb_get_pfault}} & tlb_exc_src_oh;
assign tlb_req_src_ack = {`TLB_SRC_NUM{tlb_conf_mmpage}} & tlb_mmpage_src_oh; 

  always @(posedge clk) begin
    if (!rst_n) begin
        tlb_exc_vec_r  <= {`TLB_SRC_NUM{1'b0}};
        tlb_req_vec_r  <= {`TLB_SRC_NUM{1'b0}};
    end else begin
        tlb_exc_vec_r <= {`TLB_SRC_NUM{tlb_exc_val}} & tlb_ptw_src | tlb_exc_vec_r & ~tlb_exc_src_req;
        tlb_req_vec_r <= tlb_exc_src_req | tlb_req_vec_r & ~tlb_req_src_ack;
    end
end

///////////////
// Config CR //
///////////////
wire                          cr_conf_val;
wire [`DCP_VADDR-1:0]         cr_conf_addr;
wire [`DCP_OPCODE_WIDTH-1:0]  cr_conf_op;
wire                          cr_conf_coherent;
wire [`DCP_VADDR-1:0] cr_B;
// Push to CR
wire                          cr_add_val;
wire [`FC_FIFO_SIZE-1:0]      cr_add_fifo_oh; 
wire [`CR_ARRAY_IDX-1:0]      cr_add_begin;
wire [`CR_ARRAY_IDX-1:0]      cr_add_end;
// CR to Store pipe
wire                          cr_pipe_val;
wire                          cr_pipe_rdy;
wire [`FC_FIFO_SIZE-1:0]      cr_pipe_fifo_oh; 
wire [`FC_FIFO_SIZE-1:0]      cr_pipe_fifo_rdy; 
wire [`DCP_OPCODE_WIDTH-1:0]  cr_pipe_op;
wire                          cr_pipe_len;
wire [`DCP_VADDR-1:0]         cr_pipe_addr;
wire [31:0]                   cr_pipe_data;
// CR data request interface
wire                          cr_req_val;
wire                          cr_req_rdy;
wire                          cr_req_part;
wire                          cr_req_coherent;
wire  [`CR_CHUNK_IDX   -1:0]  cr_req_idx;
wire  [`DCP_PADDR_MASK ]      cr_req_paddr;
// CR data fill interface
wire                          cr_fill_val;
wire  [`CR_CHUNK_IDX   -1:0]  cr_fill_idx;
wire  [`CR_CHUNK_SIZE  -1:0]  cr_fill_data;
wire                          cr_fill_part;
// CR state
wire                          cr_active;
wire [`FC_FIFO_IDX:0]         cr_conf_fifos;

generate if (`DCP_CR) begin : cr
dcp_chunk_req u_cr (.*);
end else begin : no_cr
    assign tlb_cr_req = 1'b0;
    assign cr_pipe_val = 1'b0;
    assign cr_req_val = 1'b0;
    assign cr_active = 1'b0;
end endgenerate
///////////////////////////////////
// NOC1 incoming transactions ack//
///////////////////////////////////
reg [`FC_FIFO_SIZE-1:0] c0_aconfigured_r;
reg [`FC_FIFO_SIZE-1:0] c0_econfigured_r;
reg [TILEID_BITS-1:0] conf_access_id_r [`FC_FIFO_SIZE-1:0];
reg [TILEID_BITS-1:0] conf_execute_id_r[`FC_FIFO_SIZE-1:0];

wire l0_rdy;
wire s0_rdy;
wire c0_rdy;
wire noc1decoder_dcp_hsk;
wire noc1decoder_dcp_conf;
wire noc1decoder_dcp_load;
wire noc1decoder_dcp_store;
wire msg_type_st = (noc1decoder_dcp_reqtype ==`DCP_REQ_STORE);
wire msg_type_ld = (noc1decoder_dcp_reqtype ==`DCP_REQ_LOAD);

// Device Mask and Intruction type
wire [2:0] device_id = noc1decoder_dcp_address[22:20];
wire maple_or_mmu_mask = !device_id[2] && !device_id[0];
wire mmu_mask = device_id[1];//when and'ed with the previous mask
wire dream_or_spd_mask = !device_id[2] && device_id[0];
wire spd_mask = dream_or_spd_mask && !device_id[1];

wire maple_st = msg_type_st && maple_or_mmu_mask;
wire maple_ld = msg_type_ld && maple_or_mmu_mask;
wire c0_st_hsk  = c0_hsk && maple_st;

// Fifo bits config (bits 11:9)
wire [2:0] fifo_bits = noc1decoder_dcp_address[`DCP_FIFO_LEN]; // 48 bits
wire [`FC_FIFO_IDX- 1:0] s0_fifo = fifo_bits[`FC_FIFO_IDX-1:0];
wire [`FC_FIFO_SIZE-1:0] s0_fifo_oh = {{`FC_FIFO_SIZE-1{1'b0}}, 1'b1} << s0_fifo;
wire [`DCP_OPCODE_WIDTH-1:0] s0_op = noc1decoder_dcp_address[`DCP_OPCODE];
wire [`DCP_OPCODE_WIDTH-1:0] pipe_op;

// Get operation codes
wire s0_op_cas1 = (s0_op == `DCP_AMO_OP_CAS1);
wire s0_op_cas2 = (s0_op == `DCP_AMO_OP_CAS2);
wire s0_op_regular_atomic = (s0_op >= `DCP_AMO_OP_ADD) && (s0_op <= `DCP_AMO_OP_SWAP);
wire s0_op_atomic = s0_op_regular_atomic || s0_op_cas1 || s0_op_cas2;

wire s0_prefetch= (s0_op == `DCP_PREFETCH);
wire s0_double_load;
wire s0_load32 = s0_op == `DCP_TLOAD32_OP || s0_op == `DCP_LLC_32;
wire s0_load64 = s0_op == `DCP_TLOAD64_OP || s0_op == `DCP_LLC_64;
wire s0_load   = s0_load32 || s0_load64;
wire s0_prod    = s0_op == `DCP_PROD_OP;
wire store_op_legal = !mmu_mask && (s0_prod || s0_load || s0_op_atomic || s0_prefetch);
wire cons_op        = !mmu_mask && (s0_op == `DCP_CONS_OP);

wire conf_op_fifo_add = maple_ld && (s0_op == `DCP_FIFO_OP);
wire conf_op_fifo_clr = maple_ld && (s0_op == `DCP_FIFO_CLR_OP);

wire conf_op_init  = maple_ld && (s0_op == `DCP_INIT_OP);
wire conf_op_aconf = maple_ld && (s0_op == `DCP_ACON_OP);
wire conf_op_econf = maple_ld && (s0_op == `DCP_ECON_OP);
wire conf_op_adesc = maple_ld && (s0_op == `DCP_ADES_OP);
wire conf_op_edesc = maple_ld && (s0_op == `DCP_EDES_OP) || conf_op_fifo_clr;
wire conf_op_reset = maple_ld && (s0_op == `DCP_REST_OP);
wire conf_op_stats = maple_ld && (s0_op == `DCP_STAT_OP);
wire conf_op_ld13  = maple_ld && (s0_op == `DCP_TLBF_OP);
wire conf_op_debug = conf_op_ld13 && !mmu_mask;
wire conf_B = (s0_op == `DCP_SET_B32_FIFO || s0_op == `DCP_SET_B64_FIFO);

// conf TLB
wire nil_conf = fifo_bits == 3'd0;
// Add the handshake to the tlb operations
assign tlb_poll = c0_hsk && conf_op_ld13 && mmu_mask;
assign tlb_conf = c0_st_hsk && mmu_mask && (s0_op == `DCP_TLBP_OP);
// Operations with tlb_poll load
assign tlb_get_pfault = tlb_poll && fifo_bits[0];
assign tlb_snoop_val  = tlb_poll && fifo_bits[1];
assign tlb_flush = tlb_poll && nil_conf || c0_invalidate || tlb_conf_ptbase;
// Operations with tlb_conf store
assign tlb_conf_ptbase = tlb_conf && fifo_bits[0]; // bit 0 set indicates page table conf
assign tlb_conf_mmpage = tlb_conf && fifo_bits[1]; // bit 1 set, resolve mem-mapped page fault
assign tlb_conf_entry  = tlb_conf && fifo_bits[2]; // bit 2 set, TLB entry conf
assign tlb_disable = tlb_conf && nil_conf;
assign tlb_update = tlb_conf_entry;
assign conf_data  = c0_data; 

wire conf_op_connection  = conf_op_aconf || conf_op_econf || conf_op_adesc || conf_op_edesc ||
                        conf_op_fifo_add || conf_op_init || conf_op_reset;

wire [`FC_FIFO_SIZE-1:0] ld_fifo_rdy;
wire [`FC_FIFO_SIZE-1:0] st_fifo_rdy;
wire [`FC_FIFO_SIZE-1:0] fifo_not_aconfig;
wire [`FC_FIFO_SIZE-1:0] fifo_not_econfig;
wire is_access;
wire is_execute;

assign is_access  = (|(c0_aconfigured_r & s0_fifo_oh)) && (noc1decoder_dcp_tile == conf_access_id_r[s0_fifo]);
assign is_execute = (|(c0_econfigured_r & s0_fifo_oh)) && (noc1decoder_dcp_tile == conf_execute_id_r[s0_fifo]);

assign noc1decoder_dcp_load  = maple_ld && cons_op && is_execute; 
assign noc1decoder_dcp_store = maple_st && store_op_legal && is_access;
assign noc1decoder_dcp_conf  = !noc1decoder_dcp_load && !noc1decoder_dcp_store;

assign noc1decoder_dcp_hsk   = noc1decoder_dcp_val && noc1decoder_dcp_ack;

// Should we always accept? supposedly enough resources! There is an assertion for this at the end
assign noc1decoder_dcp_ack = noc1decoder_dcp_load  ? l0_rdy :
                             noc1decoder_dcp_store ? s0_rdy : c0_rdy; 

wire noc2_store_sel;
wire noc2_load_sel;
wire noc2_conf_sel;
wire noc2_tload_sel;

wire config_inv_stall;
wire fifos_allocated;
wire no_fifos;

///////////////////
// CONF PIPELINE //
///////////////////
// STAGE 0 /////// -> Register Load Request from NOC1
///////////////////
wire [`FC_FIFO_SIZE-1:0]  l0_val_r;
wire [`FC_FIFO_SIZE-1:0]  s0_val_r;

reg [`FC_FIFO_SIZE-1:0] c0_B_val_r;
reg [`FC_FIFO_SIZE-1:0] c0_B_len_r;
reg [`FC_FIFO_SIZE-1:0][`DCP_VADDR-1:0] c0_B_r;
reg                             c0_val_r;
reg [`DCP_STAT_WIDTH      -1:0] c0_stat_idx_r;
reg [`DCP_MSHRID_WIDTH    -1:0] c0_mshr_r;
reg [`PACKET_HOME_ID_WIDTH-1:0] c0_src_r;
reg [`MSG_SRC_FBITS_WIDTH -1:0] c0_fbits_r;
reg  [63:0] c0_data_r;
reg c0_is_load_r;
reg [`DCP_FIFO_LEN] c0_fifos_alloc_r;
reg c0_invalidate_r;
reg c0_allocate_r;

wire [`DCP_FIFO_LEN] c0_fifo_len;
wire c0_aflag;
wire c0_eflag;
wire c0_val;
wire c0_configure;
wire c0_aconfigure_any;
wire c0_econfigure_any;
wire [`FC_FIFO_SIZE-1:0] c0_aconfigure;
wire [`FC_FIFO_SIZE-1:0] c0_econfigure;
wire [`FC_FIFO_SIZE-1:0] c0_ainvalidate;
wire [`FC_FIFO_SIZE-1:0] c0_einvalidate;
wire [`FC_FIFO_SIZE-1:0] c0_set_B;

wire [63:0] c0_stat_data [`DCP_STAT_SIZE-1:0];
wire [63:0] c0_debug_data;
//OUTPUT CONF PIPE DATA
wire [63:0] c0_open_data = { {32-`FC_FIFO_IDX{1'b0}}, fc_add_idx & {`FC_FIFO_IDX{conf_op_connection}}, 30'd0, 
                            c0_aflag && conf_op_connection, c0_eflag && conf_op_connection};

wire c1_hsk;
wire c1_rdy;
assign c0_val = noc1decoder_dcp_val && noc1decoder_dcp_conf;
assign c0_rdy = !c0_val_r;// || c1_rdy; bad timing
assign c0_hsk = c0_val && c0_rdy;

assign c0_aconfigure_any = |c0_aconfigure;
assign c0_econfigure_any = |c0_econfigure;
assign c0_configure = c0_aconfigure_any || c0_econfigure_any;
assign c0_invalidate = c0_hsk && conf_op_reset;
assign invalidate = c0_invalidate_r;

wire init_allocate = c0_hsk && conf_op_init && no_fifos && !c0_allocate_r;
always @(posedge clk) begin
    if (!rst_n) begin
        c0_val_r         <= 1'b0;
        c0_invalidate_r  <= 1'b0;
        c0_stat_idx_r    <= `DCP_STAT_WIDTH'd0;
        c0_allocate_r    <= 1'b0;
    end else begin
        c0_val_r         <= c0_hsk || c0_val_r && !c1_rdy;
        c0_invalidate_r  <= c0_invalidate || c0_invalidate_r && !c0_configure;
        c0_stat_idx_r    <= c0_hsk && conf_op_stats ? c0_stat_idx_r + `DCP_STAT_WIDTH'd1 : c0_stat_idx_r;
        c0_allocate_r    <= init_allocate || c0_allocate_r && fc_add_hsk; 
    end
end
// Config CR and LOOPs
wire cr_is_configured = cr_active && c0_B_val_r[0];
assign cr_conf_val  = c0_st_hsk && (s0_op >= `DCP_LP_TLOAD32) && (s0_op <= `DCP_LP_PREFETCH) && fifos_allocated && !c0_allocate_r;
assign cr_conf_addr = c0_data[`DCP_VADDR-1:0];
assign cr_conf_op   = s0_op;
assign cr_conf_coherent = fifo_bits[0];
assign cr_add_val = c0_st_hsk && (s0_op == `DCP_LOOP_OP) && is_access && cr_is_configured;
assign cr_add_fifo_oh = s0_fifo_oh;
assign cr_add_begin = c0_data[63:32];
assign cr_add_end   = c0_data[31:0];
assign cr_B = c0_B_r[0];

assign c0_aflag = c0_aconfigure_any || //if fifo newly configured
//if conf msg came from a configured fifo, we dont consider the case when we
//add the fifo, cause the fifo_idx is the size of the new fifo
                  !conf_op_init && !conf_op_fifo_add && is_access || init_allocate;

wire clr_not_bound = |(fifo_not_econfig & s0_fifo_oh);  
assign c0_eflag = c0_econfigure_any || is_execute || conf_op_fifo_clr && clr_not_bound;

always @(posedge clk) begin
    if (!rst_n) begin
        c0_data_r <= 64'd0;
    end else if (c0_hsk) begin
        c0_data_r <= conf_op_stats ? c0_stat_data[c0_stat_idx_r] : 
                     conf_op_debug ? c0_debug_data :
                     tlb_snoop_val ? tlb_snoop_entry : 
                     (tlb_exc_vaddr | c0_open_data);
    end
end
always @(posedge clk) begin
    if (c0_hsk) begin
        c0_mshr_r <= noc1decoder_dcp_mshrid;
        c0_src_r  <= noc1decoder_dcp_src;
        c0_fbits_r  <= noc1decoder_dcp_fbits;
        c0_is_load_r <= msg_type_ld;
        c0_fifos_alloc_r <= c0_fifo_len; 
    end
end

// Try to create new fifo if nobody is connected to the next index
// It shouldnt have an open connection (assertion about that)
// Since the add should fail if max_fifos is reached
wire add_single_fifo = c0_hsk && conf_op_fifo_add;
assign fc_add_val = (add_single_fifo || c0_allocate_r) && !config_inv_stall;
// Clear fifo if Im the Execute or IF I'm not and the fifo is not bound!
assign fc_clr_val = c0_hsk && conf_op_fifo_clr && !config_inv_stall && (clr_not_bound || is_execute);

assign c0_fifo_len = noc1decoder_dcp_address[`DCP_FIFO_LEN];
wire [`DCP_FIFO_LEN] fifo_len = c0_allocate_r ? c0_fifos_alloc_r : c0_fifo_len;
wire [7:0] len_pre = {  fifo_len == `DCP_LEN_128,
                        fifo_len == `DCP_LEN_96 || fifo_len == `DCP_LEN_80 || fifo_len == `DCP_LEN_64,
                        fifo_len == `DCP_LEN_96 || fifo_len == `DCP_LEN_48 || fifo_len == `DCP_LEN_32,
                        fifo_len == `DCP_LEN_80 || fifo_len == `DCP_LEN_48 || fifo_len == `DCP_LEN_16,
                        fifo_len == `DCP_LEN_8,
                        3'b0};
assign fc_add_size = len_pre[`FC_GLOBL_IDX-1:0];

generate
for (j = 0; j < `FC_FIFO_SIZE; j=j+1) begin : conf_fifo_gen
    wire fifo_hsk = c0_hsk && s0_fifo_oh[j];
    assign c0_aconfigure [j]= fifo_hsk && st_fifo_rdy[j] && !config_inv_stall && conf_op_aconf;
    assign c0_econfigure [j]= fifo_hsk && ld_fifo_rdy[j] && !config_inv_stall && conf_op_econf;
    assign c0_ainvalidate[j]= c0_invalidate || fifo_hsk && conf_op_adesc && is_access;
    assign c0_einvalidate[j]= c0_invalidate || fifo_hsk && conf_op_edesc && is_execute;
    assign c0_set_B[j] = c0_st_hsk && s0_fifo_oh[j] && conf_B;

    always @(posedge clk) begin
        if (!rst_n) begin
            c0_aconfigured_r[j] <= 1'b0;
            c0_econfigured_r[j] <= 1'b0;
            c0_B_val_r[j]       <= 1'b0;
            c0_B_len_r[j]       <= 1'b0;
            c0_B_r[j]           <= {`DCP_VADDR{1'b0}};
        end else begin
            c0_aconfigured_r[j] <= c0_aconfigure[j] || c0_aconfigured_r[j] && !c0_ainvalidate[j];
            c0_econfigured_r[j] <= c0_econfigure[j] || c0_econfigured_r[j] && !c0_einvalidate[j];
            c0_B_val_r[j]       <= c0_set_B[j] || c0_B_val_r[j] && !c0_invalidate;
            if (c0_set_B[j] || c0_invalidate) begin
                c0_B_len_r[j]   <= c0_invalidate ? '0 : (s0_op == `DCP_SET_B64_FIFO);
                c0_B_r[j]       <= c0_invalidate ? '0 : c0_data[`DCP_VADDR-1:0];
            end
        end
    end

    always @(posedge clk) begin
        if (c0_aconfigure[j]) begin
            conf_access_id_r[j]  <= noc1decoder_dcp_tile;
        end
        if (c0_econfigure[j]) begin
            conf_execute_id_r[j] <= noc1decoder_dcp_tile;
        end
    end
end endgenerate

///////////////////
// STAGE 1 /////// -> ACK transaction
///////////////////

wire c1_val;
wire c1_is_load;
wire [`DCP_MSHRID_WIDTH-1:0] c1_mshr;
wire [`PACKET_HOME_ID_WIDTH-1:0] c1_src;
wire [`MSG_SRC_FBITS_WIDTH -1:0] c1_fbits;
wire [63:0] c1_data;

assign c1_val  = c0_val_r;
assign c1_hsk  = c1_val && c1_rdy;
assign c1_mshr = c0_mshr_r;
assign c1_src  = c0_src_r;
assign c1_fbits   = c0_fbits_r;
assign c1_is_load = c0_is_load_r;

///////////////////
// LOAD PIPELINE //
///////////////////
wire [`FC_FIFO_SIZE-1:0] l0_alloc;
wire                    l0_len;
wire                    l0_val;
// New load request coming from NOC1
assign l0_val = noc1decoder_dcp_hsk && noc1decoder_dcp_load;
assign l0_len = (noc1decoder_dcp_size == `MSG_DATA_SIZE_8B); // 8B is 1, 4B is 0

wire l2_val,l2_rdy;
wire [`DCP_MSHRID_WIDTH-1:0] l2_mshr;
wire [`PACKET_HOME_ID_WIDTH-1:0] l2_src; 
wire [`DCP_DATA_WIDTH-1:0] l2_data;
assign l2_rdy = noc2_load_sel && dcp_noc2buffer_rdy;

/////////////////////
// STORE PIPELINE////
/////////////////////
wire [`FC_FIFO_SIZE-1:0] s0_alloc;
wire                     s0_val;
wire                     s0_len;

wire any_s0_val = |s0_val_r;
wire any_l0_val = |l0_val_r;
assign fifos_allocated = |fc_fifo_val;
assign no_fifos = !fifos_allocated;
// Keep invalidate signal high until all the remaining state is invalidated
assign config_inv_stall = invalidate && (!fc_st_empty || any_s0_val || any_l0_val);

assign fifo_not_aconfig = ~s0_val_r & ~c0_aconfigured_r;
assign fifo_not_econfig = ~l0_val_r & ~c0_econfigured_r;
assign ld_fifo_rdy = fifo_not_econfig & fc_fifo_val;
assign st_fifo_rdy = fifo_not_aconfig & fc_fifo_val;

// New load request coming from NOC1
wire s0_in = noc1decoder_dcp_val && noc1decoder_dcp_store;
wire [`FC_FIFO_SIZE-1:0] s0_fifo_arb = s0_in ? s0_fifo_oh : cr_pipe_fifo_oh;
assign s0_val = s0_in || cr_pipe_val;
assign cr_pipe_rdy = s0_rdy; //REVISIT add !s0_in if both allowed concurrently, careful with c0
assign cr_pipe_fifo_rdy = ~s0_val_r;

////////////////////////////////////////
/// DATA and ADDR FIELDS FOR StageO ///
////////////////////////////////////////
// For l0_len -> 8B is 1, 4B is 0. 
// For atomics, only 32 bits supported
//TLoad64 stores 8byte addr, s0_len should be asserted
//Tload32 also stores 8byte addr, but we don't want to put 8B in the queue, only 4B
assign s0_len = l0_len && !s0_load32 && !s0_op_atomic;

wire s0_addr_val  = s0_op_regular_atomic || s0_op_cas2 || s0_load || s0_prefetch || cr_pipe_val;
wire s0_data1_val = s0_op_regular_atomic || s0_op_cas1 || (pipe_op == `DCP_PROD_OP);
wire s0_data2_val = s0_in && (s0_op_cas1 || s0_prod && l0_len || s0_double_load);
wire B_set = c0_B_val_r[s0_fifo];
wire B_is_64 = c0_B_len_r[s0_fifo];
wire [1:0] shift = {B_set, B_set && B_is_64};
wire [31:0] noc1_data_hi;
wire [31:0] noc1_data_lo;
wire [31:0] noc1_cmp_data1;
wire [31:0] noc1_cmp_data2;
// noc1_idx contains a 32 bits offset of an addr (need to be BigEnd)
wire [31:0] noc1_idx = s0_op_atomic ? c0_data[63:32] : c0_data[31:0];
// data inserted in the fifo (or requested to L2 is in LitEnd!)
wire [31:0] s0_data1 = cr_pipe_val ? cr_pipe_data : noc1_data_lo;
// s0_data2 contains an addr offset in the case of double_load (BigEnd), 
// it contains LitEnd encoded data otherwise 
wire [`DCP_VADDR-1:0] s0_data2 = s0_double_load ? 
                       c0_B_r[s0_fifo] + (c0_data[63:32] << shift) : noc1_data_hi;
wire [`DCP_VADDR-1:0] noc1_addr = cr_pipe_val ? cr_pipe_addr : 
                       c0_B_r[s0_fifo] + (noc1_idx << shift);

// double_load when upper 32 bits of the 64bit-store are used (not zero)
assign s0_double_load = (|noc1_data_hi) && s0_load;

wire dream_sp_read_val;
wire dream_sp_read_to_ariane;
wire dream_read_to_ariane;

reg dream_read_to_ariane_r;
wire [63:0] dream_ariane_data;
reg [63:0] dream_ariane_data_r;

always @(posedge clk) begin
    if (!rst_n) begin
        dream_read_to_ariane_r <= 1'b0;
    end else if (c0_hsk) begin
        dream_read_to_ariane_r <= dream_read_to_ariane;
        dream_ariane_data_r <= dream_ariane_data;
    end
end

wire [63:0] pre = dream_read_to_ariane_r ? dream_ariane_data_r : c0_data_r;
generate if (`LITTLE_ENDIAN) begin : little_endian_gen 
    assign c0_data = {noc1_data_hi[7:0],noc1_data_hi[15:8],noc1_data_hi[23:16],noc1_data_hi[31:24],
                     noc1_data_lo[7:0],noc1_data_lo[15:8],noc1_data_lo[23:16],noc1_data_lo[31:24]};
    assign c1_data = {pre[7:0],pre[15:8],pre[23:16],pre[31:24],pre[39:32],pre[47:40],pre[55:48],pre[63:56]};
    assign noc1_data_hi = noc1decoder_dcp_data[31:0];
    assign noc1_data_lo = noc1decoder_dcp_data[63:32];
    assign noc1_cmp_data1 = {noc1_data_lo[31:16], {16{noc1_data_lo[23]}} };
    assign noc1_cmp_data2 = {noc1_data_lo[15:0], {16{noc1_data_lo[7]}} };
end else begin : big_endian_gen
    assign c0_data = {noc1_data_hi,noc1_data_lo};
    assign c1_data = pre;
    assign noc1_data_hi = noc1decoder_dcp_data[63:32];
    assign noc1_data_lo = noc1decoder_dcp_data[31:0];
    assign noc1_cmp_data1 = { {16{noc1_data_lo[15]}}, noc1_data_lo[15:0]};
    assign noc1_cmp_data2 = { {16{noc1_data_lo[31]}}, noc1_data_lo[31:16]};
end endgenerate

wire                          pipe_len, pipe_loop;
wire [`PACKET_HOME_ID_WIDTH-1:0] pipe_src;
wire [`DCP_MSHRID_WIDTH -1:0] pipe_mshr;
assign pipe_len  = s0_in ? s0_len : cr_pipe_len;
assign pipe_op   = s0_in ? s0_op  : cr_pipe_op;
assign pipe_loop = cr_pipe_val;
assign pipe_mshr = noc1decoder_dcp_mshrid;
assign pipe_src  = noc1decoder_dcp_src;

wire [`DCP_MSHRID_WIDTH-1:0]  s2_mshr;
wire [`FC_GLOBL_IDX    -1:0]  s2_entry, s2_entry_p1;
wire [`DCP_OPCODE_WIDTH-1:0]  s2_op;
wire [`DCP_REQ_LEN     -1:0]  s2_len;
wire [`PACKET_HOME_ID_WIDTH-1:0] s2_src;
wire [`HOME_ID_WIDTH-1:0] s2_amo_homeid; 
wire [`DCP_PADDR_MASK       ] s2_addr;

wire s2_req,s2_req_rdy;
wire s2_req_tload_done;
wire s2_op_rmw,s2_op_prod;
wire s2_op_tload,s2_op_tload_32,s2_op_tload_64;
wire s2_op_prefetch;
wire s2_4byte_align;
wire s3_rdy;
wire [`DCP_UNPARAM_63_0] s2_data;

wire dcp_noc1buffer_rdy_maple, dcp_noc1buffer_rdy_dream;

wire cr_noc1_req;
wire cr_noc2_req;
wire s2_amo_rdy   = dcp_noc1buffer_rdy_maple && !cr_noc1_req;
wire s2_tload_rdy = dcp_noc2buffer_rdy && !cr_noc2_req;
wire fill_prod_rdy;

//Leave to '0, that is the homeid for DRAM accesses
wire [`PACKET_HOME_ID_WIDTH-1:0] s2_tload_homeid = {1'b1,{(`PACKET_HOME_ID_WIDTH-1){1'b0}}};

assign s2_req_rdy  = s2_op_rmw   ? s2_amo_rdy :
                     s2_op_tload ? s2_req_tload_done : 
                     s2_op_prod  ? fill_prod_rdy :
                             1'b1; // IF no need to request then always granted
reg s2_first4b_requested_r;

wire s2_req_first4b = s2_req && s2_op_tload && s2_op_tload_64 && s2_4byte_align;
wire s2_req_first4b_hsk = s2_req_first4b && !s2_first4b_requested_r && dcp_noc2buffer_rdy;
always @(posedge clk) begin
    if (!rst_n) begin
        s2_first4b_requested_r <= 1'b0;
    end else begin
        s2_first4b_requested_r <= s2_req_first4b_hsk || s2_first4b_requested_r && !s2_tload_rdy;
    end
end
assign s2_req_tload_done = s2_tload_rdy && (!s2_req_first4b || s2_first4b_requested_r);
assign cr_req_rdy = cr_noc1_req && dcp_noc1buffer_rdy_maple || cr_noc2_req && dcp_noc2buffer_rdy;

/////////////////////////////
// Send NOC1 request message 
/////////////////////////////

wire                          dcp_noc1buffer_val_maple, dcp_noc1buffer_val_dream;
wire [`MSG_TYPE_WIDTH-1:0]    dcp_noc1buffer_type_maple, dcp_noc1buffer_type_dream;
wire [`DCP_MSHRID_WIDTH -1:0] dcp_noc1buffer_mshrid_maple, dcp_noc1buffer_mshrid_dream;
wire [`DCP_PADDR_MASK       ] dcp_noc1buffer_address_maple, dcp_noc1buffer_address_dream;
wire [`DCP_UNPARAM_2_0      ] dcp_noc1buffer_size_maple, dcp_noc1buffer_size_dream;
wire [`PACKET_HOME_ID_WIDTH-1:0] dcp_noc1buffer_homeid_maple;
wire [`DCP_UNPARAM_63_0     ] dcp_noc1buffer_data_0_maple, dcp_noc1buffer_data_0_dream;
wire [`DCP_UNPARAM_63_0     ] dcp_noc1buffer_data_1_maple, dcp_noc1buffer_data_1_dream;
wire [`MSG_AMO_MASK_WIDTH-1:0] dcp_noc1buffer_write_mask_maple, dcp_noc1buffer_write_mask_dream;

reg maple_last_r;
wire noc1_maple_sel = dcp_noc1buffer_val_maple && (!maple_last_r || !dcp_noc1buffer_val_dream);
assign dcp_noc1buffer_rdy_maple = dcp_noc1buffer_rdy && noc1_maple_sel;
assign dcp_noc1buffer_rdy_dream = dcp_noc1buffer_rdy && !noc1_maple_sel;

always @(posedge clk) begin
    if (!rst_n) begin
        maple_last_r <= 1'b0;
    end else if (dcp_noc1buffer_hsk) begin
        maple_last_r <= noc1_maple_sel;
    end
end

// Connect wires from MSHR and REQ to NOC1 buffer
// Only request AMO during second msg (data), not the addr msg
assign cr_noc1_req = cr_req_val && cr_req_coherent;
assign  dcp_noc1buffer_val_maple  = s2_req && s2_op_rmw || cr_noc1_req;
//Opcode to format that OpenPiton understands made in encoder! Also size!
assign  dcp_noc1buffer_type_maple = {`DCP_OPCODE_WIDTH{s2_req}} & s2_op;
// If prefetch, make MSHR all 1's, ignore response then
assign  dcp_noc1buffer_mshrid_maple = cr_noc1_req ? {1'b1, {`DCP_MSHRID_WIDTH-1-`CR_CHUNK_IDX{1'b0}}, cr_req_idx} :
                                    {s2_op_prefetch, s2_op_prefetch ? {{`DCP_MSHRID_WIDTH-2-`CR_CHUNK_IDX{1'b0}},1'b1,{`CR_CHUNK_IDX{1'b0}}} : s2_entry};             
assign  dcp_noc1buffer_address_maple = cr_noc1_req ? cr_req_paddr : s2_addr;
//This is the size of the operands, AMOs only 4B, CR is either 64B (full chunk) or 16B (part) 
assign  dcp_noc1buffer_size_maple   = cr_noc1_req ? (cr_req_part ? `MSG_DATA_SIZE_16B : `MSG_DATA_SIZE_64B) :
                                s2_len ? `MSG_DATA_SIZE_8B : `MSG_DATA_SIZE_4B;
assign  dcp_noc1buffer_data_0_maple  = {s2_data[31:0] ,s2_data[31:0]}; 
assign  dcp_noc1buffer_data_1_maple  = {s2_data[63:32],s2_data[63:32]};

// HOMEID encoding
assign dcp_noc1buffer_homeid[`PACKET_HOME_ID_CHIP_MASK] = 1'b0; // non-csm mode only has 1 chip alone
flat_id_to_xy homeid_to_xy (
    .flat_id(s2_amo_homeid),
    .x_coord(dcp_noc1buffer_homeid[`PACKET_HOME_ID_X_MASK]),
    .y_coord(dcp_noc1buffer_homeid[`PACKET_HOME_ID_Y_MASK])
    );

assign dcp_noc1buffer_write_mask_maple = 16'hFFFF;

assign dcp_noc1buffer_val = dcp_noc1buffer_val_maple || dcp_noc1buffer_val_dream;
assign dcp_noc1buffer_type = (noc1_maple_sel) ? dcp_noc1buffer_type_maple : dcp_noc1buffer_type_dream;
assign dcp_noc1buffer_mshrid = (noc1_maple_sel) ? dcp_noc1buffer_mshrid_maple : dcp_noc1buffer_mshrid_dream;
assign dcp_noc1buffer_address = (noc1_maple_sel) ? dcp_noc1buffer_address_maple : dcp_noc1buffer_address_dream;
assign dcp_noc1buffer_size = (noc1_maple_sel) ? dcp_noc1buffer_size_maple : dcp_noc1buffer_size_dream;
assign dcp_noc1buffer_data_0 = (noc1_maple_sel) ? dcp_noc1buffer_data_0_maple : dcp_noc1buffer_data_0_dream;
assign dcp_noc1buffer_data_1 = (noc1_maple_sel) ? dcp_noc1buffer_data_1_maple : dcp_noc1buffer_data_1_dream;
assign dcp_noc1buffer_write_mask = (noc1_maple_sel) ? dcp_noc1buffer_write_mask_maple : dcp_noc1buffer_write_mask_dream;

//////////////////////////
//// STAGE 3 //// -> ACK the initial Store request
/////////////////////////// 
// Every transaction has to ACK the Store!
wire s3_val;
wire [`DCP_MSHRID_WIDTH-1:0] s3_mshr;
wire [`PACKET_HOME_ID_WIDTH-1:0] s3_src; 
wire no_ack;
assign s3_rdy = noc2_store_sel && dcp_noc2buffer_rdy;


////////////////////
// NOC2 request/ack ///
////////////////////
reg conf_last_r;
always @(posedge clk) begin
    if (!rst_n) begin
        conf_last_r <= 1'b0;
    end else if (dcp_noc2buffer_hsk) begin
        conf_last_r <= noc2_conf_sel;
    end
end

wire noc2_val;
wire [`DCP_MSHRID_WIDTH-1:0] noc2_mshr;
wire [`PACKET_HOME_ID_WIDTH-1:0] noc2_homeid;
wire [`DCP_DATA_WIDTH-1:0] noc2_data;
wire [`MSG_TYPE_WIDTH-1:0] noc2_reqtype;
wire [`DCP_PADDR_MASK    ] noc2_address;
wire noc2_conf_load_sel;

wire conf_prio = !conf_last_r || !s3_val && !l2_val;
wire st_ld_prio = conf_last_r || !c1_val;
assign cr_noc2_req = cr_req_val && !cr_req_coherent;
assign noc2_tload_sel = s2_req && s2_op_tload || cr_noc2_req; //give priority to TLOAD request
assign noc2_store_sel = s3_val && !noc2_tload_sel && st_ld_prio;
assign noc2_load_sel  = l2_val && !noc2_tload_sel && st_ld_prio && !s3_val;
assign noc2_conf_sel  = c1_val && !noc2_tload_sel && conf_prio;
assign noc2_conf_load_sel = noc2_conf_sel && c1_is_load;

assign noc2_val = c1_val || l2_val || s3_val || noc2_tload_sel;
wire [`DCP_MSHRID_WIDTH-1:0] tload_mshr;
assign tload_mshr = cr_noc2_req ? {1'b1, {`DCP_MSHRID_WIDTH-1-`CR_CHUNK_IDX{1'b0}}, cr_req_idx} : 
                                 (s2_first4b_requested_r ? s2_entry_p1 : s2_entry);
//For ACK responses set MSHR id of the request, for Tload set the FIFO entry idx
assign noc2_mshr = {`DCP_MSHRID_WIDTH{noc2_tload_sel}} & tload_mshr |
                   {`DCP_MSHRID_WIDTH{noc2_store_sel}} & s3_mshr | 
                   {`DCP_MSHRID_WIDTH{noc2_load_sel}}  & l2_mshr |
                   {`DCP_MSHRID_WIDTH{noc2_conf_sel}}  & c1_mshr;

//All ACK responses set the SRC core id as Destiny. Tload sets Homeid (for DRAM is '0) 
assign noc2_homeid = {`PACKET_HOME_ID_WIDTH{noc2_tload_sel}} & s2_tload_homeid |
                     {`PACKET_HOME_ID_WIDTH{noc2_store_sel}} & s3_src | 
                     {`PACKET_HOME_ID_WIDTH{noc2_load_sel}}  & l2_src |
                     {`PACKET_HOME_ID_WIDTH{noc2_conf_sel}}  & c1_src;

//Neither ACK or TLOAD req write anything.
//Give back zeros on fault or invalidation
assign noc2_data = noc2_load_sel && !invalidate ? l2_data : c1_data; 

//Opcode to format that OpenPiton understands made in encoder! Also size!
assign noc2_reqtype = noc2_load_sel || noc2_conf_load_sel ? `DCP_NOC2_LOAD_ACK :
                     noc2_tload_sel ? `DCP_NOC2_LOAD_REQ64 :
                     /*Store pipe ack*/ `DCP_NOC2_STORE_ACK;

//TLOAD addr, other responses (ACK) do not need address 
wire s2_addr_carry = s2_first4b_requested_r;
assign noc2_address = {s2_addr[`DCP_PADDR-1:3] + s2_addr_carry, s2_4byte_align && !s2_first4b_requested_r, 2'b0}; 

//BUFFER2 REQUEST
assign  dcp_noc2buffer_val      = noc2_val;
assign  dcp_noc2buffer_type     = noc2_reqtype;
assign  dcp_noc2buffer_mshrid   = noc2_mshr;

assign  dcp_noc2buffer_address  = cr_noc2_req ? cr_req_paddr : noc2_address; 

assign  dcp_noc2buffer_data     = noc2_data;
assign  dcp_noc2buffer_homeid   = noc2_homeid; 
assign  dcp_noc2buffer_fbits    = c1_fbits;

assign c1_rdy = noc2_conf_sel && dcp_noc2buffer_rdy;

/////////////////////////////////////////
// Fill interface, data coming from NOC2
////////////////////////////////////////
// Fill interface from NOC1, normal AE, arbitrate according to NOCs priority 3-2-1.
wire noc_resp_val;
wire fill_prod_val;
// Upper bit means the request was from CR
wire noc2_mshr_msb = noc2decoder_dcp_mshrid[`DCP_MSHRID_WIDTH-1];
wire noc3_mshr_msb = noc3decoder_dcp_mshrid[`DCP_MSHRID_WIDTH-1];
wire noc2_cr_range, noc3_cr_range;
generate if ((`DCP_MSHRID_WIDTH-`CR_CHUNK_IDX) >= 2) begin : range_check
    assign noc2_cr_range = noc2_mshr_msb && !(|noc2decoder_dcp_mshrid[`DCP_MSHRID_WIDTH-2:`CR_CHUNK_IDX]);
    assign noc3_cr_range = noc3_mshr_msb && !(|noc3decoder_dcp_mshrid[`DCP_MSHRID_WIDTH-2:`CR_CHUNK_IDX]);
end else begin : no_range
    assign noc2_cr_range = noc2_mshr_msb;
    assign noc3_cr_range = noc3_mshr_msb;
end endgenerate

wire noc3_resp_val = noc3decoder_dcp_val && !noc3_mshr_msb;
// Ignore responses from the range where MSB is set
wire noc2_resp_val = noc2decoder_dcp_val && !noc2_mshr_msb;

// Assume that we cannot have NOC2 and NOC3 requests for CR at the same time
wire cr_noc2_res = noc2decoder_dcp_val && noc2_cr_range;
wire cr_noc3_res = noc3decoder_dcp_val && noc3_cr_range;
assign cr_fill_val = (cr_noc2_res || cr_noc3_res) && !invalidate;
assign cr_fill_idx = cr_noc2_res ? noc2decoder_dcp_mshrid[`CR_CHUNK_IDX-1:0] : noc3decoder_dcp_mshrid[`CR_CHUNK_IDX-1:0];
assign cr_fill_data = cr_noc2_res ? noc2decoder_dcp_data : noc3decoder_dcp_data;
assign cr_fill_part = cr_noc2_res && (noc2decoder_dcp_length <= 2);

// Responses for the Fifo Controller (FC) module
assign noc_resp_val = noc3_resp_val || noc2_resp_val;
assign fill_prod_val= s2_req && s2_op_prod;

assign fc_fill_val   = noc_resp_val || fill_prod_val;
assign fc_fill_entry = noc3_resp_val ? noc3decoder_dcp_mshrid[`FC_GLOBL_IDX-1:0] :
                       noc2_resp_val ? noc2decoder_dcp_mshrid[`FC_GLOBL_IDX-1:0] : s2_entry;

wire [`NOC_DATA_WIDTH-1:0] noc3_fill_data;
wire [`NOC_DATA_WIDTH-1:0] noc2_fill_data;
wire [`FC_DATA_SIZE  -1:0] noc3_fill_data0;
wire [`FC_DATA_SIZE  -1:0] noc3_fill_data1;
wire [`FC_DATA_SIZE  -1:0] noc2_fill_data0;
wire [`FC_DATA_SIZE  -1:0] noc2_fill_data1;

generate if (`FC_DATA_IDX == 2) begin
    assign noc3_fill_data = noc3decoder_dcp_data[`NOC_DATA_WIDTH-1:0];
end else begin
    assign noc3_fill_data = noc3decoder_dcp_data[`NOC_DATA_WIDTH*fc_fill_addr_lsb[`FC_DATA_IDX-1:1] +:`NOC_DATA_WIDTH];
end endgenerate
assign noc2_fill_data = noc2decoder_dcp_data[`NOC_DATA_WIDTH*fc_fill_addr_lsb[1] +:`NOC_DATA_WIDTH];
assign noc3_fill_data0 = noc3_fill_data[`FC_DATA_SIZE*(!fc_fill_addr_lsb[0])+:`FC_DATA_SIZE];
assign noc2_fill_data0 = noc2_fill_data[`FC_DATA_SIZE*(!fc_fill_addr_lsb[0])+:`FC_DATA_SIZE];
assign noc3_fill_data1 = noc3_fill_data[`FC_DATA_SIZE-1:0];
assign noc2_fill_data1 = noc2_fill_data[`FC_DATA_SIZE-1:0];
assign fc_fill_data  = noc3_resp_val ? {noc3_fill_data1, noc3_fill_data0} :
                       noc2_resp_val ? {noc2_fill_data1, noc2_fill_data0} : 
                                             s2_data; //Produce
assign fill_prod_rdy = !noc_resp_val; 
assign noc3decoder_dcp_ack = 1'b1; //Never block NOC3

wire noc2decoder_dcp_ack_dream = 1'b1; // Dream never blocks NOC2
wire noc2decoder_dcp_ack_maple = (!noc3_resp_val || noc2_mshr_msb);
wire dream_mshrid = (noc2decoder_dcp_mshrid > 8'd144);
assign noc2decoder_dcp_ack = dream_mshrid ? noc2decoder_dcp_ack_dream : noc2decoder_dcp_ack_maple;

/////////////////////////
/// CALCULATE L2 SLICE///
////////////////////////
reg [`HOME_ID_WIDTH-1:0] home_addr;
always @ *
begin
    case (home_alloc_method)
        `HOME_ALLOC_LOW_ORDER_BITS:
        begin
            home_addr = dcp_noc1buffer_address[`HOME_ID_ADDR_POS_LOW];
        end
        `HOME_ALLOC_MIDDLE_ORDER_BITS:
        begin
            home_addr = dcp_noc1buffer_address[`HOME_ID_ADDR_POS_MIDDLE];
        end
        `HOME_ALLOC_HIGH_ORDER_BITS:
        begin
            home_addr = dcp_noc1buffer_address[`HOME_ID_ADDR_POS_HIGH];
        end
        `HOME_ALLOC_MIXED_ORDER_BITS:
        begin
            home_addr = (dcp_noc1buffer_address[`HOME_ID_ADDR_POS_LOW] ^ dcp_noc1buffer_address[`HOME_ID_ADDR_POS_MIDDLE]);
        end
    endcase
end

l15_home_encoder u_l15_home_encoder(
   .home_in        (home_addr),
   .num_homes      (system_tile_count),
   .lhid_out       (s2_amo_homeid)
);

//PERFORMANCE COUNTERS DECLARATION
wire [`FC_ENTRY_IDX     :0] count_use;
wire [`DCP_COUNT_STALL-1:0] count_st_fifo_stall;
wire [`DCP_COUNT_STALL-1:0] count_ld_fifo_stall;
wire [`DCP_COUNT_LAT  -1:0] count_st_fifo;
wire [`DCP_COUNT_LAT  -1:0] count_ld_fifo;
wire [`DCP_COUNT_STALL-1:0] count_dram_cycles;
wire [`DCP_COUNT_STALL-1:0] count_dram_trans;
wire [`DCP_COUNT_TRANS-1:0] count_fifo_transactions;
wire [`DCP_COUNT_TRANS-1:0] count_transactions;

wire count_transactions_clr = c0_hsk && conf_op_stats;
wire count_noc2_stall_clr = dcp_noc2buffer_hsk || c0_invalidate;
wire count_noc2_stall_clk = dcp_noc2buffer_val || count_noc2_stall_clr;

assign c0_stat_data[3]= {count_transactions, count_fifo_transactions};
assign c0_stat_data[2]= {count_st_fifo_stall, count_ld_fifo_stall};
assign c0_stat_data[1]= {count_st_fifo, count_ld_fifo};
assign c0_stat_data[0]= {count_dram_trans, count_dram_cycles};

wire [50:0] pipe_debug;
wire [63:0] fc_debug_use;
wire [63:0] fc_debug_val;
assign c0_debug_data =  fifo_bits == 3'd0 ? fc_debug_use : 
                        fifo_bits == 3'd1 ? fc_debug_val : 
                        {fc_fifo_val[2:0], invalidate, tlb_req, tlb_src, tlb_maple_req, tlb_cr_req, 
                        dcp_noc2buffer_rdy, noc2_load_sel, l2_val, noc2_store_sel, s3_val, pipe_debug};

maple u_maple (.*);

custom_acc u_acc(
    .clk (clk),
    .rst_n (rst_n),
    .config_hsk (c0_hsk && (dream_or_spd_mask) ), // handshake, 1 pulse, warning: assume only dream signals reach here
    .config_addr  (noc1decoder_dcp_address[19:0]),
    .config_data_hi (noc1_data_hi), //32 higher bits
    .config_data_lo (noc1_data_lo), //32 lower bits
    .config_load (msg_type_ld),
    .config_size (noc1decoder_dcp_size),

    .noc1buffer_rdy (dcp_noc1buffer_rdy_dream),
    .noc1buffer_val (dcp_noc1buffer_val_dream),
    .noc1buffer_type (dcp_noc1buffer_type_dream),
    .noc1buffer_mshrid (dcp_noc1buffer_mshrid_dream),
    .noc1buffer_address (dcp_noc1buffer_address_dream),
    .noc1buffer_size (dcp_noc1buffer_size_dream),
    .noc1buffer_data_0 (dcp_noc1buffer_data_0_dream),
    .noc1buffer_data_1 (dcp_noc1buffer_data_1_dream),
    .noc1buffer_write_mask (dcp_noc1buffer_write_mask_dream),
    .noc2decoder_val  (noc2decoder_dcp_val && dream_mshrid),
    .noc2decoder_mshrid (noc2decoder_dcp_mshrid),
    .noc2decoder_data (noc2decoder_dcp_data),

    .read_to_ariane_data (dream_ariane_data),
    .read_to_ariane_val  (dream_read_to_ariane)
);

////// ASSERTIONS ///////////////
`ifdef DEC_ASSERT_ON
store_ready: assert property (
   @(posedge clk) disable iff (!rst_n) noc1decoder_dcp_val && noc1decoder_dcp_store |-> s0_rdy)
     else $fatal(1,"NOC1 input for Stores should always be ready");
load_ready: assert property (
   @(posedge clk) disable iff (!rst_n) noc1decoder_dcp_val && noc1decoder_dcp_load |-> l0_rdy)
     else $fatal(1,"NOC1 input for Loads should always be ready");
create_fifo_no_config: assert property (
   @(posedge clk) disable iff (!rst_n) fc_add_hsk |-> fifo_not_aconfig[fc_add_idx])
     else $fatal(1,"FIFO created on existing connection");
`endif
endmodule
