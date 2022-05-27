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


`include "dcp.h"

module dcp_noc2buffer (
    input  wire                                 clk,
    input  wire                                 rst_n,

    input  wire                                 noc2buffer_val,
    input  wire [`DCP_NOC2_REQTYPE_WIDTH-1:0]   noc2buffer_type,
    input  wire [`NOC_DATA_WIDTH-1:0]           noc2buffer_data,
    input  wire [`DCP_MSHRID_WIDTH-1:0]         noc2buffer_mshrid,
    input  wire [`PHY_ADDR_WIDTH-1:0]           noc2buffer_address,
    input  wire [`PACKET_HOME_ID_WIDTH-1:0]     noc2buffer_homeid,
    input  wire [`MSG_SRC_FBITS_WIDTH -1:0]     noc2buffer_fbits,

    output reg                                  noc2buffer_noc2encoder_val,
    output reg  [`DCP_NOC2_REQTYPE_WIDTH-1:0]   noc2buffer_noc2encoder_type,
    output reg  [`NOC_DATA_WIDTH-1:0]           noc2buffer_noc2encoder_data,
    output reg  [`DCP_MSHRID_WIDTH-1:0]         noc2buffer_noc2encoder_mshrid,
    output reg  [`PHY_ADDR_WIDTH-1:0]           noc2buffer_noc2encoder_address,
    output reg  [`PACKET_HOME_ID_WIDTH-1:0]     noc2buffer_noc2encoder_homeid,
    output reg  [`MSG_SRC_FBITS_WIDTH -1:0]     noc2buffer_noc2encoder_fbits,

    input  wire                                 noc2encoder_noc2buffer_ack,
    output reg                                  noc2buffer_ack

);


reg buffer_val;
reg buffer_val_next;
reg new_buffer;
reg [`DCP_NOC2_REQTYPE_WIDTH-1:0] noc2buffer_type_buf;
reg [`NOC_DATA_WIDTH-1:0] noc2buffer_data_buf;
reg [`DCP_MSHRID_WIDTH-1:0] noc2buffer_mshrid_buf;
reg [`PHY_ADDR_WIDTH-1:0] noc2buffer_address_buf;
reg [`PACKET_HOME_ID_WIDTH-1:0] noc2buffer_homeid_buf;
reg [`MSG_SRC_FBITS_WIDTH -1:0] noc2buffer_fbits_buf;

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        buffer_val <= 1'b0;
    end
    else
    begin
        buffer_val <= buffer_val_next;
        if (new_buffer)
        begin
            noc2buffer_type_buf <= noc2buffer_type;
            noc2buffer_data_buf <= noc2buffer_data;
            noc2buffer_mshrid_buf <= noc2buffer_mshrid;
            noc2buffer_address_buf <= noc2buffer_address;
            noc2buffer_homeid_buf <= noc2buffer_homeid;
            noc2buffer_fbits_buf <= noc2buffer_fbits;
        end
    end
end

always @ *
begin
    noc2buffer_noc2encoder_val = buffer_val;
    noc2buffer_noc2encoder_type = noc2buffer_type_buf;
    noc2buffer_noc2encoder_data = noc2buffer_data_buf;
    noc2buffer_noc2encoder_mshrid = noc2buffer_mshrid_buf;
    noc2buffer_noc2encoder_address = noc2buffer_address_buf;
    noc2buffer_noc2encoder_homeid = noc2buffer_homeid_buf;
    noc2buffer_noc2encoder_fbits  = noc2buffer_fbits_buf;
end

// val/ack logic

// buffer_ack is 1 only when req is valid, and buffer_val is 0 or noc2encoder_ack is 1
// ie buffer_ack = req && (!buffer_val || noc2encoder_ack);
// case 1: buffer not valid, accepting new req
// case 2: buffer is valid, but accepting new req because noc2 is done with current buffer

// logic for accepting new req: buffer_ack

// buffer_val_next
// = 1 when buffer_ack
// else = 0 when noc2buffer_ack
// else = buffer_val

always @ *
begin
    noc2buffer_ack = 1'b0;
    if (noc2buffer_val && (!buffer_val || noc2encoder_noc2buffer_ack))
        noc2buffer_ack = 1'b1;

    new_buffer = noc2buffer_ack;

    buffer_val_next = 1'b0;
    if (noc2buffer_ack)
        buffer_val_next = 1'b1;
    else if (noc2encoder_noc2buffer_ack)
        buffer_val_next = 1'b0;
    else
        buffer_val_next = buffer_val;
end

endmodule
