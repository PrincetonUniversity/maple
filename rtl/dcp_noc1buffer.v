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
//  Filename      : dcp_noc1buffer.v
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
`default_nettype none // DEFAULT_NETTYPE_NONE
`endif

module dcp_noc1buffer (
    input wire clk,
    input wire rst_n,

    input wire        dcp_noc1buffer_val,
    output            dcp_noc1buffer_rdy,

    input wire [`DCP_UNPARAM_63_0] dcp_noc1buffer_data_0,
    input wire [`DCP_UNPARAM_63_0] dcp_noc1buffer_data_1,
    input wire [`MSG_TYPE_WIDTH-1:0] dcp_noc1buffer_type,
    input wire [`DCP_MSHRID_WIDTH      -1:0] dcp_noc1buffer_mshrid,
    input wire [39                       :0] dcp_noc1buffer_address,
    input wire [`MSG_DATA_SIZE_WIDTH   -1:0] dcp_noc1buffer_size,
    input wire [`PACKET_HOME_ID_WIDTH-1:0]   dcp_noc1buffer_homeid,
    input wire [`MSG_AMO_MASK_WIDTH    -1:0] dcp_noc1buffer_write_mask,
    
    // encoder-buffer ack
    output reg noc1buffer_noc1encoder_val,
    input wire noc1buffer_noc1encoder_ack,

    output reg [`DCP_UNPARAM_63_0] noc1buffer_noc1encoder_data_0,
    output reg [`DCP_UNPARAM_63_0] noc1buffer_noc1encoder_data_1,
    output reg [`MSG_TYPE_WIDTH-1:0] noc1buffer_noc1encoder_type,
    output reg [`DCP_MSHRID_WIDTH      -1:0] noc1buffer_noc1encoder_mshrid,
    output reg [39:0                       ] noc1buffer_noc1encoder_address,
    output reg                               noc1buffer_noc1encoder_non_cacheable,
    output reg [`MSG_DATA_SIZE_WIDTH   -1:0] noc1buffer_noc1encoder_size,
    output reg                               noc1buffer_noc1encoder_prefetch,
    output reg [`PACKET_HOME_ID_WIDTH  -1:0] noc1buffer_noc1encoder_homeid,
    output reg [`MSG_AMO_MASK_WIDTH    -1:0] noc1buffer_noc1encoder_write_mask
);

wire dcp_noc1buffer_non_cacheable = 1'b0;
wire dcp_noc1buffer_prefetch = 1'b0;
// BUFFERS
reg [`DCP_COMMAND_BUFFER_LEN-1:0] command_buffer [0:`DCP_NOC1_BUF_NUM_SLOTS-1];
reg [`DCP_COMMAND_BUFFER_LEN-1:0] command_buffer_next [0:`DCP_NOC1_BUF_NUM_SLOTS-1];
reg [`DCP_UNPARAM_63_0] data_buffer_0 [0:`DCP_NOC1_BUF_NUM_DATA_SLOTS-1];
reg [`DCP_UNPARAM_63_0] data_buffer_0_next [0:`DCP_NOC1_BUF_NUM_DATA_SLOTS-1];
reg [`DCP_UNPARAM_63_0] data_buffer_1 [0:`DCP_NOC1_BUF_NUM_DATA_SLOTS-1];
reg [`DCP_UNPARAM_63_0] data_buffer_1_next [0:`DCP_NOC1_BUF_NUM_DATA_SLOTS-1];

reg [0:`DCP_NOC1_BUF_NUM_SLOTS-1] command_buffer_val;
reg [0:`DCP_NOC1_BUF_NUM_SLOTS-1] command_buffer_val_next;

reg [`DCP_NOC1_BUF_NUM_SLOTS_LOG-1:0] command_wrindex;
reg [`DCP_NOC1_BUF_NUM_SLOTS_LOG-1:0] command_wrindex_next;
reg [`DCP_NOC1_BUF_NUM_SLOTS_LOG-1:0] command_rdindex;
reg [`DCP_NOC1_BUF_NUM_SLOTS_LOG-1:0] command_rdindex_next;
reg [`DCP_NOC1_BUF_NUM_SLOTS_LOG-1:0] command_rdindex_plus1;
reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_wrindex;
reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_wrindex_next;
reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_wrindex_plus_1;
//reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_wrindex_plus_2;
reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_rdindex;
reg [`DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG-1:0] data_rdindex_plus1;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
      command_buffer[0] <= 0;
      command_buffer_val[0] <= 0;
      command_buffer[1] <= 0;
      command_buffer_val[1] <= 0;

      data_buffer_0[0] <= 0;
      data_buffer_0[1] <= 0;
      data_buffer_1[0] <= 0;
      data_buffer_1[1] <= 0;

      data_wrindex <= 0;
      command_wrindex <= 0;
      command_rdindex <= 0;

    end else begin

      command_buffer[0] <= command_buffer_next[0];
      command_buffer_val[0] <= command_buffer_val_next[0];
      command_buffer[1] <= command_buffer_next[1];
      command_buffer_val[1] <= command_buffer_val_next[1];

      data_buffer_0[0] <= data_buffer_0_next[0];
      data_buffer_0[1] <= data_buffer_0_next[1];
      data_buffer_1[0] <= data_buffer_1_next[0];
      data_buffer_1[1] <= data_buffer_1_next[1];

      data_wrindex <= data_wrindex_next;
      command_wrindex <= command_wrindex_next;
      command_rdindex <= command_rdindex_next;
    end
end

// Mostly related to writes
wire dcp_noc1buffer_hsk = dcp_noc1buffer_val && dcp_noc1buffer_rdy;
always @ *
begin
    command_buffer_next[0] = command_buffer[0];
    command_buffer_next[1] = command_buffer[1];
    data_buffer_0_next[0] = data_buffer_0[0];
    data_buffer_0_next[1] = data_buffer_0[1];
    data_buffer_1_next[0] = data_buffer_1[0];
    data_buffer_1_next[1] = data_buffer_1[1];

    command_wrindex_next = command_wrindex;
    data_wrindex_next   = data_wrindex;
    data_wrindex_plus_1 = data_wrindex + `DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG'd1;
    //data_wrindex_plus_2 = data_wrindex + `DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG'd2;

    if (dcp_noc1buffer_hsk) begin
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_REQTYPE] = dcp_noc1buffer_type;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_MSHRID]  = dcp_noc1buffer_mshrid;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_ADDRESS] = dcp_noc1buffer_address;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_NON_CACHEABLE] = dcp_noc1buffer_non_cacheable;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_SIZE]     = dcp_noc1buffer_size;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_PREFETCH] = dcp_noc1buffer_prefetch;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_DATA_INDEX] = data_wrindex;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_HOMEID] = dcp_noc1buffer_homeid;
        command_buffer_next[command_wrindex][`DCP_NOC1BUFFER_WRITE_MASK] = dcp_noc1buffer_write_mask;

        command_wrindex_next = command_wrindex + `DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG'd1;
        data_buffer_0_next[data_wrindex] = dcp_noc1buffer_data_0;
        data_buffer_1_next[data_wrindex] = dcp_noc1buffer_data_1;
        data_wrindex_next = data_wrindex_plus_1;
/* THE OTHER WAY TO DO THIS IS HAVING GENERIC DATA PAYLOADS
        if (dcp_noc1buffer_type == `DCP_AMO_OP_CAS1 || dcp_noc1buffer_type == `DCP_AMO_OP_CAS2) begin
           data_buffer_next[data_wrindex]        = dcp_noc1buffer_data_0;
           data_buffer_next[data_wrindex_plus_1] = dcp_noc1buffer_data_1;
           data_wrindex_next = data_wrindex_plus_2;
        end
        else if (dcp_noc1buffer_type == `DCP_AMO_OP...)
        begin
           data_buffer_next[data_wrindex] = dcp_noc1buffer_data_0;
           data_wrindex_next = data_wrindex_plus_1;
        end
*/
    end
end

// issue port to noc1encoder
always @ *
begin
    noc1buffer_noc1encoder_val = command_buffer_val[command_rdindex];
 
    noc1buffer_noc1encoder_type   = command_buffer[command_rdindex][`DCP_NOC1BUFFER_REQTYPE];
    noc1buffer_noc1encoder_mshrid = command_buffer[command_rdindex][`DCP_NOC1BUFFER_MSHRID];
    noc1buffer_noc1encoder_address  = command_buffer[command_rdindex][`DCP_NOC1BUFFER_ADDRESS];
    noc1buffer_noc1encoder_non_cacheable = command_buffer[command_rdindex][`DCP_NOC1BUFFER_NON_CACHEABLE];
    noc1buffer_noc1encoder_size     = command_buffer[command_rdindex][`DCP_NOC1BUFFER_SIZE];
    noc1buffer_noc1encoder_prefetch = command_buffer[command_rdindex][`DCP_NOC1BUFFER_PREFETCH];
    noc1buffer_noc1encoder_homeid = command_buffer[command_rdindex][`DCP_NOC1BUFFER_HOMEID];
    noc1buffer_noc1encoder_write_mask = command_buffer[command_rdindex][`DCP_NOC1BUFFER_WRITE_MASK];

    // Read the index of the data buffer, that was encoded in the command buffer
    data_rdindex = command_buffer[command_rdindex][`DCP_NOC1BUFFER_DATA_INDEX];
    //data_rdindex_plus1 = data_rdindex + `DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG'd1;
    //noc1buffer_noc1encoder_data_0 = data_buffer[data_rdindex];
    //noc1buffer_noc1encoder_data_1 = data_buffer[data_rdindex_plus1];
    noc1buffer_noc1encoder_data_0 = data_buffer_0[data_rdindex];
    noc1buffer_noc1encoder_data_1 = data_buffer_1[data_rdindex];
    
    
end

// handling valid array (and conflicts)
always @ *
begin
   
    if (dcp_noc1buffer_hsk && (command_wrindex == 0))
        command_buffer_val_next[0] = 1'b1;
    else if (noc1buffer_noc1encoder_ack && (command_rdindex == 0))
        command_buffer_val_next[0] = 1'b0;
    else
        command_buffer_val_next[0] = command_buffer_val[0];
   

    if (dcp_noc1buffer_hsk && (command_wrindex == 1))
        command_buffer_val_next[1] = 1'b1;
    else if (noc1buffer_noc1encoder_ack && (command_rdindex == 1))
        command_buffer_val_next[1] = 1'b0;
    else
        command_buffer_val_next[1] = command_buffer_val[1];
   

end

assign dcp_noc1buffer_rdy = !(&command_buffer_val);

always @ *
begin
    command_rdindex_plus1   = command_rdindex + `DCP_NOC1_BUF_NUM_DATA_SLOTS_LOG'd1;
    command_rdindex_next    = command_rdindex;

    if (noc1buffer_noc1encoder_ack) begin
        command_rdindex_next = command_rdindex_plus1;
    end
end
endmodule
