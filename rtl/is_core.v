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
//  Filename      : is_core.v
//  Created On    : 2020-11-30
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Wrapper for the dcp (decoupled MAPLE hardware) and the local MMU.
//                  'IS' stands for Intelligent Storage, is_core.v is the core module
//                   that is inserted into a tiled-manycore like Openpiton's
//
//==================================================================================================

`include "define.tmp.h"
`include "dmbr_define.v"
`include "l15.tmp.h"
`include "jtag.vh"
`include "dcp.h"
`include "is.h"

module is_core (

    input  wire                                 clk_gated,
    input  wire                                 rst_n_f,
    input  wire [31:0]                          config_system_tile_count,
    input  wire [`HOME_ALLOC_METHOD_WIDTH-1:0]  config_home_alloc_method,
    input  wire [`NOC_CHIPID_WIDTH-1:0]         config_chipid,
    input  wire [`NOC_X_WIDTH-1:0]              config_coreid_x,
    input  wire [`NOC_Y_WIDTH-1:0]              config_coreid_y,
    input  wire                                 splitter_dev1_vr_noc1_val,
    input  wire [`NOC_DATA_WIDTH-1:0]           splitter_dev1_vr_noc1_dat,
    output wire                                 splitter_dev1_vr_noc1_rdy,
    output wire                                 dev1_merger_vr_noc1_val,  
    output wire [`NOC_DATA_WIDTH-1:0]           dev1_merger_vr_noc1_dat,  
    input  wire                                 dev1_merger_vr_noc1_rdy,  
    input  wire                                 splitter_dev1_vr_noc2_val,
    input  wire [`NOC_DATA_WIDTH-1:0]           splitter_dev1_vr_noc2_dat,
    output wire                                 splitter_dev1_vr_noc2_rdy,
    output wire                                 dev1_merger_vr_noc2_val,  
    output wire [`NOC_DATA_WIDTH-1:0]           dev1_merger_vr_noc2_dat,  
    input  wire                                 dev1_merger_vr_noc2_rdy,  
    input  wire                                 splitter_dev1_vr_noc3_val,
    input  wire [`NOC_DATA_WIDTH-1:0]           splitter_dev1_vr_noc3_dat,
    output wire                                 splitter_dev1_vr_noc3_rdy,
    input  wire                                 l15_transducer_ack,
    input  wire                                 l15_transducer_header_ack,

    output wire                                 transducer_l15_val,
    output wire [`PCX_REQTYPE_WIDTH-1:0]        transducer_l15_rqtype,
    output wire [`L15_AMO_OP_WIDTH-1:0]         transducer_l15_amo_op,
    output wire [`PCX_SIZE_FIELD_WIDTH-1:0]     transducer_l15_size,
    output wire [`L15_PADDR_HI:0]               transducer_l15_address,
    output wire [63:0]                          transducer_l15_data,
    output wire                                 transducer_l15_nc,
    output wire [`L15_THREADID_MASK]            transducer_l15_threadid,
    output wire                                 transducer_l15_prefetch,
    output wire                                 transducer_l15_blockstore,
    output wire                                 transducer_l15_blockinitstore,
    output wire [1:0]                           transducer_l15_l1rplway,
    output wire                                 transducer_l15_invalidate_cacheline,
    output wire [`TLB_CSM_WIDTH-1:0]            transducer_l15_csm_data,
    output wire [63:0]                          transducer_l15_data_next_entry,
    output wire                                 transducer_l15_req_ack,


    input wire                                  l15_transducer_val,
    input wire [3:0]                            l15_transducer_returntype,
    input wire [`L15_THREADID_MASK]             l15_transducer_threadid,
    input wire [63:0]                           l15_transducer_data_0,
    input wire [63:0]                           l15_transducer_data_1

);

// MMU Config iface
wire tlb_disable,tlb_conf_ptbase,tlb_flush,tlb_update;
wire [63:0] conf_data;

// TLB req/res iface
wire tlb_req,tlb_ack,tlb_val,tlb_exc;
wire tlb_exc_val = tlb_val && tlb_exc;
wire [`DCP_VADDR-1:0]  tlb_vaddr,tlb_ptw_vaddr;
wire [`L15_PADDR_HI:0] tlb_paddr;
// TLB Snoop iface
wire tlb_snoop_val;
wire [63:0] tlb_snoop_entry;

///////////////
// Decouplng //
///////////////
dcp dcp (
    .clk                        ( clk_gated                 ),
    .rst_n                      ( rst_n_f                   ),
    .system_tile_count          ( config_system_tile_count[`HOME_ID_WIDTH-1:0]),
    .home_alloc_method          ( config_home_alloc_method  ),

    .chipid                     ( config_chipid             ),
    .coreid_x                   ( config_coreid_x           ),
    .coreid_y                   ( config_coreid_y           ),
    // input from noc1
    .src_dcp_vr_noc1_val        ( splitter_dev1_vr_noc1_val  ),
    .src_dcp_vr_noc1_dat        ( splitter_dev1_vr_noc1_dat  ),
    .src_dcp_vr_noc1_rdy        ( splitter_dev1_vr_noc1_rdy  ),

    // output to noc1
    .dcp_dst_vr_noc1_val        ( dev1_merger_vr_noc1_val    ),
    .dcp_dst_vr_noc1_dat        ( dev1_merger_vr_noc1_dat    ),
    .dcp_dst_vr_noc1_rdy        ( dev1_merger_vr_noc1_rdy    ),

    // input from noc2
    .src_dcp_vr_noc2_val        ( splitter_dev1_vr_noc2_val  ),
    .src_dcp_vr_noc2_dat        ( splitter_dev1_vr_noc2_dat  ),
    .src_dcp_vr_noc2_rdy        ( splitter_dev1_vr_noc2_rdy  ),

    // output to noc2
    .dcp_dst_vr_noc2_val        ( dev1_merger_vr_noc2_val    ),
    .dcp_dst_vr_noc2_dat        ( dev1_merger_vr_noc2_dat    ),
    .dcp_dst_vr_noc2_rdy        ( dev1_merger_vr_noc2_rdy    ),

    // input from noc3
    .src_dcp_vr_noc3_val        ( splitter_dev1_vr_noc3_val  ),
    .src_dcp_vr_noc3_dat        ( splitter_dev1_vr_noc3_dat  ),
    .src_dcp_vr_noc3_rdy        ( splitter_dev1_vr_noc3_rdy  ),

    // Config iface
    .tlb_flush                  (tlb_flush), 
    .tlb_disable                (tlb_disable), 
    .tlb_conf_ptbase            (tlb_conf_ptbase), 
    .tlb_update                 (tlb_update),
    .conf_data                  (conf_data),

    // TLB request iface
    .tlb_req                    (tlb_req), 
    .tlb_ack                    (tlb_ack), 
    .tlb_exc_val                (tlb_exc_val), 
    .tlb_ptw_src                (tlb_ptw_vaddr[`TLB_SRC_NUM-1:0]), 
    .tlb_vaddr                  (tlb_vaddr), 
    .tlb_paddr                  (tlb_paddr[`DCP_PADDR-1:0]),

    // TLB snoop interface to DCP
    .tlb_snoop_val              (tlb_snoop_val),
    .tlb_snoop_entry            (tlb_snoop_entry)
);

// MMU-L15 iface
wire l15_store,l15_interrupt;
wire                           mmutx_l15_val;
wire [`L15_PADDR_HI:0]         mmutx_l15_address;
wire [63:0]                    mmutx_l15_data;
wire [4:0]                     mmutx_l15_rqtype = l15_store ? `PCX_REQTYPE_STORE : `PCX_REQTYPE_LOAD;
wire                           mmutx_l15_nc     = l15_interrupt;
wire [2:0]                     mmutx_l15_size   = 3'b011; // 8 bytes
wire [`L15_AMO_OP_WIDTH-1:0]   mmutx_l15_amo_op = {`L15_AMO_OP_WIDTH{1'b0}};
wire                           l15_mmutx_val,l15_mmutx_ack;
wire [63:0] l15_rdata = mmutx_l15_address[3] ? l15_transducer_data_1 : l15_transducer_data_0;

io_mmu #(
      .VADDR (`DCP_VADDR),
      .DATA_TLB_ENTRIES (`DCP_TLB_SIZE)
  ) is_mmu (
        .clk_i                  ( clk_gated                 ),
        .rst_ni                 ( rst_n_f                   ),

        // Resquest iface 
        .req_i        (tlb_req),        // request address translation
        .vaddr_i      (tlb_vaddr),      // virtual address in
        .is_store_i   (1'b0),           // the translation is requested by a store
        // Response iface
        .hit_o        (tlb_ack),        // sent in the same cycle as the request if translation hits in the DTLB
        .paddr_o      (tlb_paddr),      // translated address
        // Exception interface
        .valid_o      (tlb_val),        // translation is valid
        .exc_val_o    (tlb_exc),        // address translation threw an exception
        .ptw_addr_o   (tlb_ptw_vaddr),  // address translation threw an exception

        // MMU Config iface
        .flush_i          (tlb_flush),
        .tlb_disable      (tlb_disable), 
        .tlb_conf_ptbase  (tlb_conf_ptbase), 
        .tlb_update       (tlb_update),
        .conf_data        (conf_data),

        // Control signals
        .asid_i           (1'b0),
        // Performance counter
        .tlb_miss_o      (),
    
        // TLB snoop interface to DCP
        .tlb_snoop_val              (tlb_snoop_val),
        .tlb_snoop_entry            (tlb_snoop_entry),
        // PTW memory interface
        .l15_store        (l15_store),
        .l15_interrupt    (l15_interrupt),
        .l15_val          (mmutx_l15_val),
        .l15_ack          (l15_mmutx_ack),
        .l15_address      (mmutx_l15_address),
        .l15_data         (mmutx_l15_data),
        .l15_rvalid       (l15_mmutx_val),
        .l15_rdata        (l15_rdata)
);

//////////////////////////////
// OTHER IFACE using L15 ? //
/////////////////////////////

wire                           l15_customtx_ack;
wire [4:0]                     customtx_l15_rqtype;
wire [`L15_AMO_OP_WIDTH-1:0]   customtx_l15_amo_op;
wire [2:0]                     customtx_l15_size;
wire                           customtx_l15_val;
wire [`PHY_ADDR_WIDTH-1:0]     customtx_l15_address;
wire [63:0]                    customtx_l15_data;
wire                           customtx_l15_nc;
wire                           l15_customtx_val;

// pc_cmp signals
wire [47:0] pc_w;
wire inst_done;
assign inst_done = 1'b1;
assign pc_w = 48'd0;

/////////////////////////
// CUSTOM vs MMU Arbiter //
/////////////////////////

// The Custom iface is not used, but it is left here in case another
// device inside of the wrapper that uses the L15
assign customtx_l15_val = 1'b0;

reg header_ack_q;
always @(posedge clk_gated) begin
    if (!rst_n_f) begin
        header_ack_q <= 1'b0;
    end else begin
        header_ack_q <= (l15_transducer_header_ack || header_ack_q ) && !l15_transducer_ack;
    end
end

wire l15_val_pre, l15_src;
rr_arbiter #(
  .SOURCES(2)
  ) u_l15_rr (
  .clk     (clk_gated),
  .reset_n (rst_n_f),
  .stall   (!l15_transducer_ack),
  .valid_source ({mmutx_l15_val,customtx_l15_val}),
  .arb_src_oh (),
  .arb_src (l15_src),
  .arb_val (l15_val_pre)
  );
  
assign transducer_l15_val = l15_val_pre && !header_ack_q;
assign transducer_l15_req_ack = l15_transducer_val; // Always accept response
assign transducer_l15_threadid = l15_src; // MMU=1; Custom=0
// Multiplex fields between MMU and Custom
assign transducer_l15_rqtype  = l15_src ?  mmutx_l15_rqtype : customtx_l15_rqtype;
assign transducer_l15_amo_op  = l15_src ?  mmutx_l15_amo_op : customtx_l15_amo_op;
assign transducer_l15_size    = l15_src ?  mmutx_l15_size : customtx_l15_size;
assign transducer_l15_address = l15_src ?  mmutx_l15_address : customtx_l15_address;
assign transducer_l15_data = l15_src ?  mmutx_l15_data : customtx_l15_data;
assign transducer_l15_nc   = l15_src ?  mmutx_l15_nc : customtx_l15_nc;

// Unused fields
assign transducer_l15_prefetch = 1'b0;
assign transducer_l15_blockstore = 1'b0;
assign transducer_l15_blockinitstore = 1'b0;
assign transducer_l15_l1rplway = 2'd0;
assign transducer_l15_invalidate_cacheline = 1'b0;
assign transducer_l15_csm_data = {`TLB_CSM_WIDTH{1'b0}};
assign transducer_l15_data_next_entry = 64'd0;

// Check response msgtype
wire l15_rtype_correct = (l15_transducer_returntype==`CPX_RESTYPE_LOAD) || (l15_transducer_returntype==`CPX_RESTYPE_STORE_ACK);
// Split L1.5 ack to appropriate 
assign l15_mmutx_ack = l15_src ? l15_transducer_ack : 1'b0;
assign l15_customtx_ack = l15_src ? 1'b0 : l15_transducer_ack;
// Send l1.5 message to appropriate destination
assign l15_mmutx_val = l15_transducer_val && l15_transducer_threadid && l15_rtype_correct;
assign l15_customtx_val = l15_transducer_val && !l15_transducer_threadid;


endmodule
