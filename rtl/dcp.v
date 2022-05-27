
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
//  Filename      : dcp.v
//  Created On    : 2019-11-23
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : DCP top module
//
//==================================================================================================

//`timescale 1 ns / 10 ps
`include "dcp.h"
`include "is.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif

module dcp (
    input                                   clk,
    input                                   rst_n,
    input wire [`HOME_ALLOC_METHOD_WIDTH-1:0] home_alloc_method,
    input wire [`HOME_ID_WIDTH-1:0]         system_tile_count,
    
    // IS Tile identifier
    input [`NOC_CHIPID_WIDTH-1:0]           chipid,
    input [`NOC_X_WIDTH-1:0]                coreid_x,
    input [`NOC_Y_WIDTH-1:0]                coreid_y,

    // Config iface
    output wire                             tlb_conf_ptbase,
    output wire                             tlb_disable,
    output                                  tlb_flush,
    output wire                             tlb_update,
    output wire [63:0]                      conf_data,

    // TLB req/res iface
    output                                  tlb_req,
    output [`DCP_VADDR    -1:0]             tlb_vaddr,
    input                                   tlb_ack,
    input                                   tlb_exc_val,
    input  [`TLB_SRC_NUM  -1:0]             tlb_ptw_src,
    input  [`DCP_PADDR    -1:0]             tlb_paddr,

    // Snoop TLB entries
    output wire                            tlb_snoop_val,
    input  wire [63:0]                     tlb_snoop_entry,  

    // Issue Atomic requests to L2
    output                                  dcp_dst_vr_noc1_val,
    input                                   dcp_dst_vr_noc1_rdy,
    output [`NOC_DATA_WIDTH-1:0]            dcp_dst_vr_noc1_dat,
    
    // Receive Store/Load requests from the core L1.5
    input                                   src_dcp_vr_noc1_val,
    output                                  src_dcp_vr_noc1_rdy,
    input  [`NOC_DATA_WIDTH-1:0]            src_dcp_vr_noc1_dat,

    // Issue requests to DRAM or ACK to L1.5
    output                                  dcp_dst_vr_noc2_val,
    input                                   dcp_dst_vr_noc2_rdy,
    output [`NOC_DATA_WIDTH-1:0]            dcp_dst_vr_noc2_dat,  
    
    // Receive Atomic response from L2
    input                                   src_dcp_vr_noc2_val,
    output                                  src_dcp_vr_noc2_rdy,
    input  [`NOC_DATA_WIDTH-1:0]            src_dcp_vr_noc2_dat,
    
    // Receive data from DRAM (for TLOADS)
    input                                   src_dcp_vr_noc3_val,
    output                                  src_dcp_vr_noc3_rdy,
    input  [`NOC_DATA_WIDTH-1:0]            src_dcp_vr_noc3_dat
);

/* IF WE WANT TO TEST WITH SOME EXTRA NOC CONGESTION
reg [3:0] cont_r;
wire cont0 = cont_r == 4'd0;
always @(posedge clk) begin
    if (!rst_n) begin
        cont_r <= 4'd0;
    end else if (noc2out_valid) begin
        cont_r <= cont_r + 1'b1;
    end
end
*/

wire noc2out_valid;
wire noc2out_ready = dcp_dst_vr_noc2_rdy; // && cont0;
assign dcp_dst_vr_noc2_val = noc2out_valid;// && cont0;


//////////////////////////////////////////////////////
// NOC1 decoder signals, response (incoming) interface
///////////////////////////////////////////////////// 
wire                               noc1decoder_dcp_ack;
wire                               noc1decoder_dcp_val;
wire [`MSG_TYPE_WIDTH   -1:0]      noc1decoder_dcp_reqtype;
wire [`DCP_MSHRID_WIDTH -1:0]      noc1decoder_dcp_mshrid;
wire [`DCP_UNPARAM_63_0     ]      noc1decoder_dcp_data;
wire [`DCP_PADDR_MASK       ]      noc1decoder_dcp_address;  
wire [`MSG_DATA_SIZE_WIDTH-1:0]    noc1decoder_dcp_size;  
wire [`MSG_SRC_X_WIDTH  -1:0]      noc1decoder_dcp_src_x;  
wire [`MSG_SRC_Y_WIDTH  -1:0]      noc1decoder_dcp_src_y;  
wire [`MSG_SRC_CHIPID_WIDTH-1:0]   noc1decoder_dcp_chipid;
wire [`MSG_SRC_FBITS_WIDTH-1:0]    noc1decoder_dcp_fbits;  
// NOC1 BUFFER signal declarations
wire                                   dcp_noc1buffer_rdy;
wire                                   dcp_noc1buffer_val;
wire [`MSG_TYPE_WIDTH            -1:0] dcp_noc1buffer_type;
wire [`DCP_MSHRID_WIDTH          -1:0] dcp_noc1buffer_mshrid;
wire [`DCP_PADDR_MASK                ] dcp_noc1buffer_address;
wire [`MSG_DATA_SIZE_WIDTH-1:0]        dcp_noc1buffer_size;
wire [`DCP_UNPARAM_63_0              ] dcp_noc1buffer_data_0;
wire [`DCP_UNPARAM_63_0              ] dcp_noc1buffer_data_1;
wire [`PACKET_HOME_ID_WIDTH-1:0]       dcp_noc1buffer_homeid;
wire [`MSG_AMO_MASK_WIDTH-1:0]         dcp_noc1buffer_write_mask;

//////////////////////////////////////////////////////
// NOC2 decoder signals, response (incoming) interface
/////////////////////////////////////////////////////
wire                               noc2decoder_dcp_ack;
wire                               noc2decoder_dcp_val;
wire [`MSG_TYPE_WIDTH  -1:0]       noc2decoder_dcp_reqtype;
wire [`DCP_MSHRID_WIDTH-1:0]       noc2decoder_dcp_mshrid;
wire [`MSG_LENGTH_WIDTH-1:0]       noc2decoder_dcp_length;
wire [`DCP_NOC_RES_DATA_SIZE-1:0]  noc2decoder_dcp_data;
// NOC2 to buffer and encoder, Request or ACK (outgoing) interface
wire                               dcp_noc2buffer_val;
wire                               dcp_noc2buffer_rdy;
wire [`DCP_NOC2_REQTYPE_WIDTH-1:0] dcp_noc2buffer_type;
wire [`NOC_DATA_WIDTH        -1:0] dcp_noc2buffer_data;
wire [`DCP_MSHRID_WIDTH      -1:0] dcp_noc2buffer_mshrid;
wire [`PHY_ADDR_WIDTH        -1:0] dcp_noc2buffer_address;
wire [`PACKET_HOME_ID_WIDTH  -1:0] dcp_noc2buffer_homeid;
wire [`MSG_SRC_FBITS_WIDTH   -1:0] dcp_noc2buffer_fbits;

//////////////////////////////////////////////////////
// NOC3 decoder signals, response (incoming) interface
/////////////////////////////////////////////////////
wire                               noc3decoder_dcp_ack;
wire                               noc3decoder_dcp_val;
wire [`MSG_TYPE_WIDTH   -1:0]      noc3decoder_dcp_reqtype;
wire [`DCP_MSHRID_WIDTH -1:0]      noc3decoder_dcp_mshrid;
wire [`DCP_NOC_RES_DATA_SIZE-1:0]  noc3decoder_dcp_data;

////////////////////
// pipeline instance
////////////////////
dcp_pipe u_pipe(
      .clk(clk),
      .rst_n(rst_n),
      .system_tile_count              (system_tile_count  ),
      .home_alloc_method              (home_alloc_method  ),
      // Config iface
      .tlb_disable                    (tlb_disable), 
      .tlb_conf_ptbase                (tlb_conf_ptbase), 
      .tlb_flush                      (tlb_flush), 
      .tlb_update                     (tlb_update),
      .conf_data                      (conf_data), 
      // TLB iface
      .tlb_req                        (tlb_req), 
      .tlb_ack                        (tlb_ack), 
      .tlb_exc_val                    (tlb_exc_val),
      .tlb_ptw_src                    (tlb_ptw_src),
      .tlb_vaddr                      (tlb_vaddr), 
      .tlb_paddr                      (tlb_paddr), 
      // Snoop interface
      .tlb_snoop_val              (tlb_snoop_val),
      .tlb_snoop_entry            (tlb_snoop_entry),

      // NOC1 signals
      .noc1decoder_dcp_val            (noc1decoder_dcp_val),
      .noc1decoder_dcp_ack            (noc1decoder_dcp_ack),
      .noc1decoder_dcp_mshrid         (noc1decoder_dcp_mshrid),
      .noc1decoder_dcp_reqtype        (noc1decoder_dcp_reqtype),
      .noc1decoder_dcp_data           (noc1decoder_dcp_data),
      .noc1decoder_dcp_address        (noc1decoder_dcp_address),
      .noc1decoder_dcp_size           (noc1decoder_dcp_size),
      .noc1decoder_dcp_src_x          (noc1decoder_dcp_src_x),
      .noc1decoder_dcp_src_y          (noc1decoder_dcp_src_y),
      .noc1decoder_dcp_chipid         (noc1decoder_dcp_chipid),
      .noc1decoder_dcp_fbits          (noc1decoder_dcp_fbits),
      // NOC2 signals
      .noc2decoder_dcp_val            (noc2decoder_dcp_val),
      .noc2decoder_dcp_ack            (noc2decoder_dcp_ack),
      .noc2decoder_dcp_mshrid         (noc2decoder_dcp_mshrid),
      .noc2decoder_dcp_length         (noc2decoder_dcp_length),
      .noc2decoder_dcp_reqtype        (noc2decoder_dcp_reqtype),
      .noc2decoder_dcp_data           (noc2decoder_dcp_data),
      // NOC3 signals
      .noc3decoder_dcp_val            (noc3decoder_dcp_val),
      .noc3decoder_dcp_ack            (noc3decoder_dcp_ack),
      .noc3decoder_dcp_mshrid         (noc3decoder_dcp_mshrid),
      .noc3decoder_dcp_reqtype        (noc3decoder_dcp_reqtype),
      .noc3decoder_dcp_data           (noc3decoder_dcp_data),
      
      // NOC1 signals
      .dcp_noc1buffer_val             (dcp_noc1buffer_val),
      .dcp_noc1buffer_rdy             (dcp_noc1buffer_rdy),
      .dcp_noc1buffer_type            (dcp_noc1buffer_type),
      .dcp_noc1buffer_mshrid          (dcp_noc1buffer_mshrid),
      .dcp_noc1buffer_address         (dcp_noc1buffer_address),
      .dcp_noc1buffer_size            (dcp_noc1buffer_size),
      .dcp_noc1buffer_homeid          (dcp_noc1buffer_homeid),
      .dcp_noc1buffer_write_mask      (dcp_noc1buffer_write_mask),
      .dcp_noc1buffer_data_0          (dcp_noc1buffer_data_0),
      .dcp_noc1buffer_data_1          (dcp_noc1buffer_data_1),      

      .dcp_noc2buffer_val         (dcp_noc2buffer_val),
      .dcp_noc2buffer_rdy         (dcp_noc2buffer_rdy),
      .dcp_noc2buffer_type        (dcp_noc2buffer_type),
      .dcp_noc2buffer_data        (dcp_noc2buffer_data),
      .dcp_noc2buffer_mshrid      (dcp_noc2buffer_mshrid),
      .dcp_noc2buffer_address     (dcp_noc2buffer_address),
      .dcp_noc2buffer_homeid      (dcp_noc2buffer_homeid),
      .dcp_noc2buffer_fbits       (dcp_noc2buffer_fbits)
  );

////////////////////////////////
// NOC1 OUTGOING TRANSACTIONS///
////////////////////////////////

// NOC1 buffer/encoder wires 
wire                               noc1buffer_noc1encoder_val;
wire                               noc1buffer_noc1encoder_ack;
wire [`MSG_TYPE_WIDTH-1:0]         noc1buffer_noc1encoder_type;
wire [`DCP_MSHRID_WIDTH-1:0]       noc1buffer_noc1encoder_mshrid;
wire [`DCP_PADDR_HI:0]             noc1buffer_noc1encoder_address;
wire                               noc1buffer_noc1encoder_non_cacheable;
wire [`MSG_DATA_SIZE_WIDTH-1:0]    noc1buffer_noc1encoder_size;
wire                               noc1buffer_noc1encoder_prefetch;
wire [`PACKET_HOME_ID_WIDTH  -1:0] noc1buffer_noc1encoder_homeid;
wire [`DCP_UNPARAM_63_0          ] noc1buffer_noc1encoder_data_0;
wire [`DCP_UNPARAM_63_0          ] noc1buffer_noc1encoder_data_1; 
wire [`MSG_AMO_MASK_WIDTH    -1:0] noc1buffer_noc1encoder_write_mask;
  
// NoC1 buffers data before send out to NoC1, unlike NoC3 which doesn't have to buffer
dcp_noc1buffer u_noc1buffer(
      .clk(clk),
      .rst_n(rst_n),

      .dcp_noc1buffer_val             (dcp_noc1buffer_val),
      .dcp_noc1buffer_rdy             (dcp_noc1buffer_rdy),

      .dcp_noc1buffer_data_0          (dcp_noc1buffer_data_0),
      .dcp_noc1buffer_data_1          (dcp_noc1buffer_data_1),
      .dcp_noc1buffer_type            (dcp_noc1buffer_type),
      .dcp_noc1buffer_mshrid          (dcp_noc1buffer_mshrid),
      .dcp_noc1buffer_address         (dcp_noc1buffer_address),
      .dcp_noc1buffer_size            (dcp_noc1buffer_size),
      .dcp_noc1buffer_homeid          (dcp_noc1buffer_homeid),
      .dcp_noc1buffer_write_mask      (dcp_noc1buffer_write_mask),
      
      .noc1buffer_noc1encoder_val     (noc1buffer_noc1encoder_val),
      .noc1buffer_noc1encoder_ack     (noc1buffer_noc1encoder_ack),
      
      .noc1buffer_noc1encoder_data_0        (noc1buffer_noc1encoder_data_0),
      .noc1buffer_noc1encoder_data_1        (noc1buffer_noc1encoder_data_1),
      .noc1buffer_noc1encoder_type          (noc1buffer_noc1encoder_type),
      .noc1buffer_noc1encoder_mshrid        (noc1buffer_noc1encoder_mshrid),
      .noc1buffer_noc1encoder_address       (noc1buffer_noc1encoder_address),
      .noc1buffer_noc1encoder_non_cacheable (noc1buffer_noc1encoder_non_cacheable),
      .noc1buffer_noc1encoder_size          (noc1buffer_noc1encoder_size),
      .noc1buffer_noc1encoder_prefetch      (noc1buffer_noc1encoder_prefetch),
      .noc1buffer_noc1encoder_homeid        (noc1buffer_noc1encoder_homeid),
      .noc1buffer_noc1encoder_write_mask    (noc1buffer_noc1encoder_write_mask)
  );
  
   dcp_noc1encoder u_noc1encoder(
      .clk(clk),
      .rst_n(rst_n),

      .noc1buffer_noc1encoder_val     (noc1buffer_noc1encoder_val),
      .noc1buffer_noc1encoder_ack     (noc1buffer_noc1encoder_ack),

      .noc1buffer_noc1encoder_data_0  (noc1buffer_noc1encoder_data_0),
      .noc1buffer_noc1encoder_data_1  (noc1buffer_noc1encoder_data_1),
      .noc1buffer_noc1encoder_type    (noc1buffer_noc1encoder_type),
      .noc1buffer_noc1encoder_mshrid  (noc1buffer_noc1encoder_mshrid),
      .noc1buffer_noc1encoder_address (noc1buffer_noc1encoder_address),
      .noc1buffer_noc1encoder_non_cacheable(noc1buffer_noc1encoder_non_cacheable),
      .noc1buffer_noc1encoder_size    (noc1buffer_noc1encoder_size),
      .noc1buffer_noc1encoder_prefetch(noc1buffer_noc1encoder_prefetch),
      .noc1buffer_noc1encoder_homeid  (noc1buffer_noc1encoder_homeid),
      .noc1buffer_noc1encoder_write_mask(noc1buffer_noc1encoder_write_mask),

      .chipid     (chipid),
      .coreid_x   (coreid_x),
      .coreid_y   (coreid_y),
      
      .noc1_out_val   (dcp_dst_vr_noc1_val), //Output 
      .noc1_out_ready (dcp_dst_vr_noc1_rdy),
      .noc1_out_data  (dcp_dst_vr_noc1_dat)
  );


//////////////////////////////////////////////////
// Get Requests from NOC1 - INCOMING TRANSACTIONS
//////////////////////////////////////////////////
wire [`DCP_NOC_REQ_BUF_SIZE-1:0] noc1_data;
wire         noc1_data_val;
wire         noc1_data_ack;

// NOC1 input buffer
// NOC1 buffer gets all the flits of a packet before transmitting it to the DCP
dcp_nocbuffer_dec #(
      .DATA_BUF_TOTAL_SIZE    (`DCP_NOC_REQ_BUF_SIZE)
  ) u_noc1_input_buffer (
      .clk            (clk),
      .rst_n          (rst_n),
      .noc_in_val     (src_dcp_vr_noc1_val),
      .noc_in_data    (src_dcp_vr_noc1_dat),
      .noc_in_rdy     (src_dcp_vr_noc1_rdy),
      .msg            (noc1_data),
      .msg_ack        (noc1_data_ack),
      .msg_val        (noc1_data_val)
  );
  
// noc1decoder takes the data from the buffer and decode it to meaningful signals to the DCP
     dcp_noc1decoder #(
    .DATA_BUF_TOTAL_SIZE    (`DCP_NOC_REQ_BUF_SIZE) //Buffer Size
      ) u_noc1decoder(
      .noc_data                  (noc1_data),
      .noc_data_val              (noc1_data_val),
      .noc_data_ack              (noc1_data_ack),

      .nocdecoder_dcp_val        (noc1decoder_dcp_val),
      .nocdecoder_dcp_ack        (noc1decoder_dcp_ack),
      .nocdecoder_dcp_mshrid     (noc1decoder_dcp_mshrid),
      .nocdecoder_dcp_reqtype    (noc1decoder_dcp_reqtype),
      .nocdecoder_dcp_data       (noc1decoder_dcp_data),
      .nocdecoder_dcp_address    (noc1decoder_dcp_address),
      .nocdecoder_dcp_size       (noc1decoder_dcp_size),
      .nocdecoder_dcp_src_x      (noc1decoder_dcp_src_x),
      .nocdecoder_dcp_src_y      (noc1decoder_dcp_src_y),
      .nocdecoder_dcp_chipid     (noc1decoder_dcp_chipid),
      .nocdecoder_dcp_fbits      (noc1decoder_dcp_fbits)
    );

///////////////////////////////////////////////////
// Get Responses from NOC2 - INCOMING TRANSACTIONS/
///////////////////////////////////////////////////

// NoC2 buffer gets all the flits of a packet before transmitting it to the NOC2 decoder
wire [`DCP_NOC_RES_BUF_SIZE-1:0] noc2_data;
wire noc2_data_val;
wire noc2_data_ack;
  
dcp_nocbuffer_dec #(
      .DATA_BUF_TOTAL_SIZE    (`DCP_NOC_RES_BUF_SIZE)
  ) u_noc2_input_buffer (
      .clk(clk),
      .rst_n(rst_n),
      .noc_in_val (src_dcp_vr_noc2_val),
      .noc_in_data(src_dcp_vr_noc2_dat),
      .noc_in_rdy (src_dcp_vr_noc2_rdy),
      // DATA out to decoder
      .msg        (noc2_data),
      .msg_ack    (noc2_data_ack),
      .msg_val    (noc2_data_val)
  );
      //noc2decoder takes the data from the buffer and decode it to meaningful signals to the dcp
     dcp_nocdecoder #(
    .DATA_BUF_TOTAL_SIZE       (`DCP_NOC_RES_BUF_SIZE),
    .MSG_RES_TYPE              (`DCP_RES_NOC2)
      ) u_noc2decoder(
      .noc_data                  (noc2_data),
      .noc_data_val              (noc2_data_val),
      .noc_data_ack              (noc2_data_ack),

      .nocdecoder_dcp_val        (noc2decoder_dcp_val),
      .nocdecoder_dcp_ack        (noc2decoder_dcp_ack),
      .nocdecoder_dcp_mshrid     (noc2decoder_dcp_mshrid),
      .nocdecoder_dcp_length     (noc2decoder_dcp_length),
      .nocdecoder_dcp_reqtype    (noc2decoder_dcp_reqtype),
      .nocdecoder_dcp_data       (noc2decoder_dcp_data)
  );

////////////////////////////////
// NOC2 OUTGOING TRANSACTIONS///
////////////////////////////////

wire noc2buffer_noc2encoder_val;
wire noc2encoder_noc2buffer_ack;
wire [`DCP_NOC2_REQTYPE_WIDTH-1:0] noc2buffer_noc2encoder_type;
wire [`NOC_DATA_WIDTH-1:0] noc2buffer_noc2encoder_data;
wire [`DCP_MSHRID_WIDTH   -1:0] noc2buffer_noc2encoder_mshrid; 
wire [`PHY_ADDR_WIDTH      -1:0] noc2buffer_noc2encoder_address;
wire [`PACKET_HOME_ID_WIDTH-1:0] noc2buffer_noc2encoder_homeid;
wire [`MSG_SRC_FBITS_WIDTH -1:0] noc2buffer_noc2encoder_fbits;

dcp_noc2buffer u_noc2buffer (
    .clk                                   (clk),
    .rst_n                                 (rst_n),

    // input from DCP pipe
    .noc2buffer_val                    (dcp_noc2buffer_val),
    .noc2buffer_ack                    (dcp_noc2buffer_rdy),// ack to dcp
    .noc2buffer_type                   (dcp_noc2buffer_type),
    .noc2buffer_data                   (dcp_noc2buffer_data),
    .noc2buffer_mshrid                 (dcp_noc2buffer_mshrid),
    .noc2buffer_address                (dcp_noc2buffer_address),
    .noc2buffer_homeid                 (dcp_noc2buffer_homeid),
    .noc2buffer_fbits                  (dcp_noc2buffer_fbits),

    // output to noc2encoder
    .noc2buffer_noc2encoder_val        (noc2buffer_noc2encoder_val),
    .noc2buffer_noc2encoder_type       (noc2buffer_noc2encoder_type),
    .noc2buffer_noc2encoder_data       (noc2buffer_noc2encoder_data),
    .noc2buffer_noc2encoder_mshrid     (noc2buffer_noc2encoder_mshrid),
    .noc2buffer_noc2encoder_address    (noc2buffer_noc2encoder_address),
    .noc2buffer_noc2encoder_homeid     (noc2buffer_noc2encoder_homeid),
    .noc2buffer_noc2encoder_fbits      (noc2buffer_noc2encoder_fbits),
    // ack from noc2encoder
    .noc2encoder_noc2buffer_ack        (noc2encoder_noc2buffer_ack)
);

// noc2 encoder
dcp_noc2encoder u_noc2_encoder (
    .clk                         (clk),
    .rst_n                       (rst_n),
    // input from noc2buffer
    .noc2encoder_val         (noc2buffer_noc2encoder_val),
    .noc2encoder_type        (noc2buffer_noc2encoder_type),
    .noc2encoder_data        (noc2buffer_noc2encoder_data),
    .noc2encoder_mshrid      (noc2buffer_noc2encoder_mshrid),
    .noc2encoder_address     (noc2buffer_noc2encoder_address),
    .noc2encoder_homeid      (noc2buffer_noc2encoder_homeid),
    .noc2encoder_fbits       (noc2buffer_noc2encoder_fbits),
    .noc2encoder_ack         (noc2encoder_noc2buffer_ack),

    .chipid                      (chipid),
    .coreid_x                    (coreid_x),
    .coreid_y                    (coreid_y),

    // noc2 interface (send memory request)
    .noc2out_ready               (noc2out_ready),
    .noc2encoder_noc2out_val     (noc2out_valid),
    .noc2encoder_noc2out_data    (dcp_dst_vr_noc2_dat)
); 

//////////////////////////////////////////////////
// Get Responses from NOC3 - INCOMING TRANSACTIONS
//////////////////////////////////////////////////
wire [`DCP_NOC_RES_BUF_SIZE-1:0] noc3_data;
wire         noc3_data_val;
wire         noc3_data_ack;

// NOC3 input buffer
// NOC3 buffer gets all the flits of a packet before transmitting it to the DCP
dcp_nocbuffer_dec #(
      .DATA_BUF_TOTAL_SIZE    (`DCP_NOC_RES_BUF_SIZE)
  ) u_noc3_input_buffer (
      .clk            (clk),
      .rst_n          (rst_n),
      .noc_in_val     (src_dcp_vr_noc3_val),
      .noc_in_data    (src_dcp_vr_noc3_dat),
      .noc_in_rdy     (src_dcp_vr_noc3_rdy),
      .msg_ack        (noc3_data_ack),
      .msg            (noc3_data),
      .msg_val        (noc3_data_val)
  );
  
// noc3decoder takes the data from the buffer and decode it to meaningful signals to the DCP
     dcp_nocdecoder #(
    .DATA_BUF_TOTAL_SIZE    (`DCP_NOC_RES_BUF_SIZE), //Buffer Size
    .MSG_RES_TYPE           (`DCP_RES_NOC3)
      ) u_noc3decoder(
      .noc_data                  (noc3_data),
      .noc_data_val              (noc3_data_val),
      .noc_data_ack              (noc3_data_ack),

      .nocdecoder_dcp_val        (noc3decoder_dcp_val),
      .nocdecoder_dcp_ack        (noc3decoder_dcp_ack),
      .nocdecoder_dcp_mshrid     (noc3decoder_dcp_mshrid),
      .nocdecoder_dcp_length     (),
      .nocdecoder_dcp_reqtype    (noc3decoder_dcp_reqtype),
      .nocdecoder_dcp_data       (noc3decoder_dcp_data)
    );

endmodule
