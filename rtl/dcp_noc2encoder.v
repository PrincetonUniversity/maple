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

module dcp_noc2encoder (
    input  wire                                 clk,
    input  wire                                 rst_n,

    input  wire                                 noc2encoder_val,
    input  wire [`DCP_NOC2_REQTYPE_WIDTH-1:0]   noc2encoder_type,
    input  wire [`NOC_DATA_WIDTH-1:0]           noc2encoder_data,
    input  wire [`DCP_MSHRID_WIDTH-1:0]         noc2encoder_mshrid,
    input  wire [`PHY_ADDR_WIDTH-1:0]           noc2encoder_address,
    input  wire [`PACKET_HOME_ID_WIDTH-1:0]     noc2encoder_homeid,
    input  wire [`MSG_SRC_FBITS_WIDTH -1:0]     noc2encoder_fbits,

    input  wire [`NOC_CHIPID_WIDTH-1:0]         chipid,
    input  wire [`NOC_X_WIDTH-1:0]              coreid_x,
    input  wire [`NOC_Y_WIDTH-1:0]              coreid_y,

    input  wire                                 noc2out_ready,

    output reg                                  noc2encoder_ack,

    output reg                                  noc2encoder_noc2out_val,
    output reg  [`NOC_DATA_WIDTH-1:0]           noc2encoder_noc2out_data
);

reg [`NOC_DATA_WIDTH-1:0] flit;
reg [`DCP_NOC2_FLIT_STATE_WIDTH-1:0] flit_state;
reg [`DCP_NOC2_FLIT_STATE_WIDTH-1:0] flit_state_next;

reg [`PHY_ADDR_WIDTH-1:0] address;
reg [`NOC_X_WIDTH-1:0] dest_xpos;
reg [`NOC_Y_WIDTH-1:0] dest_ypos;
reg [`NOC_CHIPID_WIDTH-1:0] dest_chipid;
reg [`NOC_FBITS_WIDTH-1:0] dest_fbits;
reg [`NOC_X_WIDTH-1:0] src_xpos;
reg [`NOC_Y_WIDTH-1:0] src_ypos;
reg [`NOC_CHIPID_WIDTH-1:0] src_chipid;
reg [`NOC_FBITS_WIDTH-1:0] src_fbits;
reg [`MSG_LENGTH_WIDTH-1:0] msg_length;
reg [`MSG_TYPE_WIDTH-1:0] msg_type;
reg [`MSG_MSHRID_WIDTH-1:0] msg_mshrid;
reg [`MSG_OPTIONS_1] msg_options_1;
reg [`MSG_OPTIONS_2_] msg_options_2;
reg [`MSG_OPTIONS_3_] msg_options_3;
reg [`MSG_OPTIONS_4] msg_options_4;
reg response;
reg sending;

// Assume that Chunk Requests come from MSHR > 128
wire byte64 = noc2encoder_mshrid[`DCP_MSHRID_WIDTH-1];
wire [3:0] size = {1'b1, byte64, byte64};


reg [`NOC_DATA_WIDTH-1:0] noc2encoder_data_f;
always @ (posedge clk)
begin
    if (!rst_n)
    begin
        noc2encoder_data_f <= `NOC_DATA_WIDTH'd0;
    end
    else if (noc2encoder_val)
    begin
        noc2encoder_data_f <= noc2encoder_data;
    end
end

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        flit_state <= `DCP_NOC2_FLIT_STATE_WIDTH'd0;
    end
    else
    begin
        flit_state <= flit_state_next;
    end
end

always @ *
begin
    address = noc2encoder_address;
    dest_xpos = noc2encoder_homeid[`PACKET_HOME_ID_X_MASK];
    dest_ypos = noc2encoder_homeid[`PACKET_HOME_ID_Y_MASK];
    dest_chipid = noc2encoder_homeid[`PACKET_HOME_ID_CHIP_MASK];
    dest_fbits = noc2encoder_fbits;

    src_xpos = coreid_x;
    src_ypos = coreid_y;
    src_chipid = chipid;
    src_fbits = `NOC_FBITS_DCP;

    msg_options_1 = `MSG_OPTIONS_1_WIDTH'd0;
    msg_options_2 = `MSG_OPTIONS_2_WIDTH'd0;
    msg_options_3 = `MSG_OPTIONS_3_WIDTH'd0;
    msg_options_4 = `MSG_OPTIONS_4_WIDTH'd0;
    msg_length = `MSG_LENGTH_WIDTH'd0;
    msg_type = `MSG_TYPE_WIDTH'd0;
    msg_mshrid = noc2encoder_mshrid;
    response = 1'b0;
    sending = noc2encoder_val;
    noc2encoder_noc2out_val = sending;

    case (noc2encoder_type)
        `DCP_NOC2_LOAD_ACK:
        begin
            msg_type = `DCP_REQ_LOAD_ACK;
            msg_length = `DCP_NOC_ACK_LENGTH; 
            response = 1;
        end
        `DCP_NOC2_STORE_ACK:
        begin
            msg_type = `DCP_REQ_STORE_ACK;
            msg_length = `MSG_LENGTH_WIDTH'd0;
        end
        default:
        begin
            msg_type = `DCP_REQ_TLOAD;
            msg_length = `MSG_LENGTH_WIDTH'd2; // 2 extra req headers
            msg_options_2 = {4'd0, size, 8'd0};
            dest_fbits = `NOC_FBITS_MEM;
        end
    endcase

    // flit filling logic
    flit[`NOC_DATA_WIDTH-1:0] = `NOC_DATA_WIDTH'd0; // so that the flit is not a latch
    if (flit_state == `DCP_NOC2_REQ_HEADER_1) begin
        flit[`MSG_DST_CHIPID] = dest_chipid;
        flit[`MSG_DST_X] = dest_xpos;
        flit[`MSG_DST_Y] = dest_ypos;
        flit[`MSG_DST_FBITS] = dest_fbits;
        flit[`MSG_LENGTH] = msg_length;
        flit[`MSG_TYPE] = msg_type;
        flit[`MSG_MSHRID] = msg_mshrid;
        flit[`MSG_OPTIONS_1] = msg_options_1;
    end
    else if (!response && (flit_state == `DCP_NOC2_REQ_HEADER_2)) begin
        flit[`MSG_ADDR_] = address;
        flit[`MSG_OPTIONS_2_] = msg_options_2;
    end
    else if (!response && (flit_state == `DCP_NOC2_REQ_HEADER_3)) begin
        flit[`MSG_SRC_CHIPID_] = src_chipid;
        flit[`MSG_SRC_X_] = src_xpos;
        flit[`MSG_SRC_Y_] = src_ypos;
        flit[`MSG_SRC_FBITS_] = src_fbits;
        flit[`MSG_OPTIONS_3_] = msg_options_3;
    end
    else begin
        flit[`NOC_DATA_WIDTH-1:0] = noc2encoder_data_f;
    end
    noc2encoder_noc2out_data = flit;

    // ack logic to L2
    if (noc2encoder_val && (flit_state == msg_length) && noc2out_ready)
    begin
        noc2encoder_ack = 1'b1;
    end
    else
    begin
        noc2encoder_ack = 1'b0;
    end
end

wire eom;
assign eom = (flit_state == msg_length);
always @ *
begin
    // next flit state logic
    flit_state_next = flit_state;
    if (sending) begin
        if (noc2out_ready) begin
            if (eom) begin
                flit_state_next = `DCP_NOC2_REQ_HEADER_1;
            end else begin
                flit_state_next = flit_state + `DCP_NOC2_FLIT_STATE_WIDTH'd1;
            end
        end
    end
    else
    begin
        flit_state_next = `DCP_NOC2_REQ_HEADER_1;
    end

end
endmodule

