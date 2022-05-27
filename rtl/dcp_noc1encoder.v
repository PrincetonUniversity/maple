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
//  Filename      : dcp_noc1encoder.v
//  Created On    : 2019-11-30
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   :
//
//
//==================================================================================================
`include "dcp.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif
module dcp_noc1encoder(
    input wire clk,
    input wire rst_n,

    // interface with noc1buffer
    input wire        noc1buffer_noc1encoder_val,
    output            noc1buffer_noc1encoder_ack,
   
    input wire [`DCP_UNPARAM_63_0] noc1buffer_noc1encoder_data_0,
    input wire [`DCP_UNPARAM_63_0] noc1buffer_noc1encoder_data_1,

    input wire [`MSG_TYPE_WIDTH        -1:0] noc1buffer_noc1encoder_type,
    input wire [`DCP_MSHRID_WIDTH      -1:0] noc1buffer_noc1encoder_mshrid,
    input wire [`DCP_PADDR_MASK            ] noc1buffer_noc1encoder_address,
    input wire noc1buffer_noc1encoder_prefetch,
    input wire noc1buffer_noc1encoder_non_cacheable,
    input wire [`MSG_DATA_SIZE_WIDTH   -1:0] noc1buffer_noc1encoder_size,
    input wire [`PACKET_HOME_ID_WIDTH-1:0]   noc1buffer_noc1encoder_homeid,
    input wire [`MSG_AMO_MASK_WIDTH-1:0] noc1buffer_noc1encoder_write_mask,

    // current chip position
    input wire [`NOC_CHIPID_WIDTH-1:0] chipid,
    input wire [`NOC_X_WIDTH     -1:0] coreid_x,
    input wire [`NOC_Y_WIDTH     -1:0] coreid_y,

    // ready at noc1
    input wire        noc1_out_ready,
    output reg        noc1_out_val,
    output reg [`DCP_UNPARAM_63_0] noc1_out_data
);

// The flit sending out this cycle
reg [`DCP_UNPARAM_63_0] flit;
reg [`DCP_NOC1_FLIT_STATE_WIDTH-1:0] flit_state;
reg [`DCP_NOC1_FLIT_STATE_WIDTH-1:0] flit_state_next;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
      flit_state <= 0;
    end else begin
      flit_state <= flit_state_next;
    end
end


always @ *
begin
    // simple alias for output
    noc1_out_data = flit;
    noc1_out_val = noc1buffer_noc1encoder_val;
end

// generate request signals
reg [`MSG_TYPE_WIDTH-1:0] req_type;
reg [`DCP_UNPARAM_63_0] req_data0;
reg [`DCP_UNPARAM_63_0] req_data1;
reg [`DCP_PADDR_MASK] req_address;
reg [`MSG_MSHRID_WIDTH-1:0] req_mshrid;
reg [`MSG_DATA_SIZE_WIDTH-1:0] req_size;
reg [`NOC_X_WIDTH-1:0] req_dest_l2_xpos;
reg [`NOC_Y_WIDTH-1:0] req_dest_l2_ypos;
reg [`NOC_CHIPID_WIDTH-1:0] req_dest_chipid;
reg [`MSG_AMO_MASK_WIDTH-1:0] req_write_mask;

always @ *
begin
    req_type = noc1buffer_noc1encoder_type;
    req_data0 = noc1buffer_noc1encoder_data_0;
    req_data1 = noc1buffer_noc1encoder_data_1;
    req_address = noc1buffer_noc1encoder_address;
    req_mshrid = {noc1buffer_noc1encoder_mshrid};
    req_size = noc1buffer_noc1encoder_size;
    req_dest_l2_xpos = noc1buffer_noc1encoder_homeid[`PACKET_HOME_ID_X_MASK];
    req_dest_l2_ypos = noc1buffer_noc1encoder_homeid[`PACKET_HOME_ID_Y_MASK];
    req_dest_chipid = noc1buffer_noc1encoder_homeid[`PACKET_HOME_ID_CHIP_MASK];
    req_write_mask = noc1buffer_noc1encoder_write_mask;
end

// translate req_ -> msg_
reg [`PHY_ADDR_WIDTH-1:0]              msg_address;
reg [`NOC_X_WIDTH-1:0]                 msg_dest_l2_xpos;
reg [`NOC_Y_WIDTH-1:0]                 msg_dest_l2_ypos;
reg [`NOC_CHIPID_WIDTH-1:0]            msg_dest_chipid;
reg [`NOC_FBITS_WIDTH-1:0]             msg_dest_fbits;
reg [`NOC_X_WIDTH-1:0]                 msg_src_xpos;
reg [`NOC_Y_WIDTH-1:0]                 msg_src_ypos;
reg [`NOC_CHIPID_WIDTH-1:0]            msg_src_chipid;
reg [`NOC_FBITS_WIDTH-1:0]             msg_src_fbits;
reg [`MSG_LENGTH_WIDTH-1:0]            msg_length;
reg [`MSG_TYPE_WIDTH-1:0]              msg_type;
reg [`MSG_MSHRID_WIDTH-1:0]            msg_mshrid;
reg [`MSG_MESI_BITS-1:0]               msg_mesi;
reg [`MSG_LAST_SUBLINE_WIDTH-1:0]      msg_last_subline;
reg [`MSG_OPTIONS_1]                   msg_options_1;
reg [`MSG_OPTIONS_2_]                  msg_options_2;
reg [`MSG_OPTIONS_3_]                  msg_options_3;
reg [`MSG_CACHE_TYPE_WIDTH-1:0]        msg_cache_type;
reg [`MSG_SUBLINE_VECTOR_WIDTH-1:0]    msg_subline_vector;
reg [`MSG_DATA_SIZE_WIDTH-1:0]         msg_data_size;
reg [5:0] t1_interrupt_cpuid;
always @ *
begin
    msg_length = 0;
    msg_cache_type = `MSG_CACHE_TYPE_DATA;
    msg_type = 0;
    msg_mesi = 0;
    msg_last_subline = 0;
    msg_subline_vector = 0; // always 0 for requests
    t1_interrupt_cpuid = 0;

    msg_address = req_address;
    msg_mshrid = req_mshrid;
    msg_data_size = req_size;

    // source are static
    msg_src_xpos = coreid_x;
    msg_src_ypos = coreid_y;
    msg_src_chipid = chipid;
    msg_src_fbits = `NOC_FBITS_DCP;
    msg_dest_fbits = `NOC_FBITS_L2;

    // default value for a message, will be overwritten by interrupt reqs
    msg_dest_l2_xpos = req_dest_l2_xpos;
    msg_dest_l2_ypos = req_dest_l2_ypos;
    msg_dest_chipid = req_dest_chipid;

    case (req_type)
      `DCP_AMO_OP_CAS1:
      begin
         msg_type = `MSG_TYPE_CAS_REQ;
         msg_length = 4; // 2 extra headers + 1 compare data + 1 swap data
      end
      `DCP_AMO_OP_CAS2:
      begin
         msg_type = `MSG_TYPE_CAS_REQ;
         msg_length = 4; // 2 extra headers + 1 compare data + 1 swap data
      end
      `DCP_AMO_OP_SWAP:
      begin
         msg_type = `MSG_TYPE_SWAP_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_ADD:
      begin
         msg_type = `MSG_TYPE_AMO_ADD_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_AND:
      begin
         msg_type = `MSG_TYPE_AMO_AND_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_OR:
      begin
         msg_type = `MSG_TYPE_AMO_OR_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_XOR:
      begin
         msg_type = `MSG_TYPE_AMO_XOR_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_MAX:
      begin
         msg_type = `MSG_TYPE_AMO_MAX_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_MAXU:
      begin
         msg_type = `MSG_TYPE_AMO_MAXU_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_MIN:
      begin
         msg_type = `MSG_TYPE_AMO_MIN_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end
      `DCP_AMO_OP_MINU:
      begin
         msg_type = `MSG_TYPE_AMO_MINU_REQ;
         msg_length = 3; // 2 extra headers + 1 swap data
      end

      `DREAM_SW_WB:
      begin
         msg_type = `MSG_TYPE_SWAPWB_REQ;
         msg_length = 4; // 2 extra headers + 2 swap data
      end

      // default is frequently used!!!
      default:
      begin
         msg_type = `MSG_TYPE_LOAD_NOSHARE_REQ;
         msg_length = 2; //only 2 extra headers
      end
    endcase
end

always @ *
begin
    msg_options_1 = 0;
    msg_options_2 = 0;
    msg_options_3 = 0;
    msg_options_2[`MSG_DATA_SIZE_] = msg_data_size;

    msg_options_2[`MSG_CACHE_TYPE_] = msg_cache_type;
    msg_options_2[`MSG_SUBLINE_VECTOR_] = msg_subline_vector;
    
    msg_options_3[`MSG_SDID_] = 0;
    msg_options_3[`MSG_LSID_] = 0;

    if (req_type == `DREAM_SW_WB) begin
      msg_options_2[`MSG_AMO_MASK0_] = req_write_mask[7:0];
      msg_options_3[`MSG_AMO_MASK1_] = req_write_mask[15:8];
    end

end


// flit filling logic
// translate msg_ -> flit
always @ *
begin
    flit[`NOC_DATA_WIDTH-1:0] = 0; // so that the flit is not a latch
    if (flit_state == `DCP_NOC1_REQ_HEADER_1)
    begin
      flit[`MSG_DST_CHIPID] = msg_dest_chipid;
      flit[`MSG_DST_X] = msg_dest_l2_xpos;
      flit[`MSG_DST_Y] = msg_dest_l2_ypos;
      flit[`MSG_DST_FBITS] = msg_dest_fbits;
      flit[`MSG_LENGTH] = msg_length;
      flit[`MSG_TYPE] = msg_type;
      flit[`MSG_MSHRID] = msg_mshrid;
      flit[`MSG_OPTIONS_1] = msg_options_1;
    end
    else if (flit_state == `DCP_NOC1_REQ_HEADER_2)
    begin
      flit[`MSG_ADDR_] = msg_address;
      flit[`MSG_OPTIONS_2_] = msg_options_2;
    end
    else if (flit_state == `DCP_NOC1_REQ_HEADER_3)
    begin
      flit[`MSG_SRC_CHIPID_] = msg_src_chipid;
      flit[`MSG_SRC_X_] = msg_src_xpos;
      flit[`MSG_SRC_Y_] = msg_src_ypos;
      flit[`MSG_SRC_FBITS_] = msg_src_fbits;
      flit[`MSG_OPTIONS_3_] = msg_options_3;
    end
    else if (flit_state == `DCP_NOC1_REQ_DATA_1)
    begin
      flit[`NOC_DATA_WIDTH-1:0] = req_data0;
    end
    else if (flit_state == `DCP_NOC1_REQ_DATA_2)
    begin
      flit[`NOC_DATA_WIDTH-1:0] = req_data1;
    end
end

always @ *
begin
    // next flit state logic
    if (noc1_out_val) begin
      if (noc1_out_ready) begin
         if (flit_state != msg_length)
            flit_state_next = flit_state + `DCP_NOC1_FLIT_STATE_WIDTH'd1;
         else
            flit_state_next = `DCP_NOC1_REQ_HEADER_1;
      end else
         flit_state_next = flit_state;
    end else begin
      flit_state_next = `DCP_NOC1_REQ_HEADER_1;
    end
end

assign noc1buffer_noc1encoder_ack = noc1buffer_noc1encoder_val && (flit_state == msg_length) && noc1_out_ready;

endmodule
