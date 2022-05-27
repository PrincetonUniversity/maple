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
//  Filename      : dcp_nocdecoder.v
//  Created On    : 2019-11-30
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Decoder of response messages coming from NOC
//
//
//==================================================================================================

`include "dcp.h"

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif
module dcp_nocdecoder #(
    parameter DATA_BUF_TOTAL_SIZE       = 512,
    parameter MSG_RES_TYPE              = 0
) (
    input wire [DATA_BUF_TOTAL_SIZE-1:0]   noc_data,
    input wire                             noc_data_val,
    output reg                             noc_data_ack,

    input wire                             nocdecoder_dcp_ack,
    output reg                             nocdecoder_dcp_val,
    output reg [`DCP_NOC_RES_DATA_SIZE-1:0] nocdecoder_dcp_data,
    output reg [`DCP_MSHRID_WIDTH    -1:0] nocdecoder_dcp_mshrid,
    output reg [`MSG_LENGTH_WIDTH    -1:0] nocdecoder_dcp_length,
    output reg [`MSG_TYPE_WIDTH      -1:0] nocdecoder_dcp_reqtype

);
    wire [`MSG_TYPE] msg_res_type = MSG_RES_TYPE;
    reg [`MSG_MSHRID_WIDTH-1:0] noc_mshrid;
    always @ *
    begin
        // set valid signal to dcp when decoder gets it from the buffer
        nocdecoder_dcp_val = noc_data_val && (nocdecoder_dcp_reqtype == msg_res_type);
        // Give ack to NOC when DCP accepts the message
        noc_data_ack = nocdecoder_dcp_ack;
    
        // these are shared by both requests and replies from L
        noc_mshrid             = noc_data[`MSG_MSHRID];
        nocdecoder_dcp_mshrid  = noc_mshrid[`DCP_MSHRID_WIDTH-1:0];
        nocdecoder_dcp_length  = noc_data[`MSG_LENGTH];
        nocdecoder_dcp_reqtype = noc_data[`MSG_TYPE];
        nocdecoder_dcp_data    = noc_data[DATA_BUF_TOTAL_SIZE-1:`NOC_DATA_WIDTH];
    end

endmodule
