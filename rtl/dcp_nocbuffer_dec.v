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

module dcp_nocbuffer_dec #(
    parameter DATA_BUF_TOTAL_SIZE = 512
) (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              noc_in_val,
    input  wire [`NOC_DATA_WIDTH-1:0]        noc_in_data,
    input  wire                              msg_ack,
    output reg                               noc_in_rdy,
    output      [DATA_BUF_TOTAL_SIZE-1:0]    msg,
    output reg                               msg_val
);
localparam DATA_BUF_IN_SIZE = DATA_BUF_TOTAL_SIZE / `NOC_DATA_WIDTH;
reg [`DCP_NOC3_FLIT_STATE_WIDTH-1:0] index;
reg [`DCP_NOC3_FLIT_STATE_WIDTH-1:0] index_next;
reg [`MSG_LENGTH_WIDTH-1:0] msg_len;
reg [`DCP_NOC_STATE_WIDTH-1:0] state;
reg [`DCP_NOC_STATE_WIDTH-1:0] state_next;
reg [`NOC_DATA_WIDTH-1:0] buffer [DATA_BUF_IN_SIZE-1:0];
reg [`NOC_DATA_WIDTH-1:0] buffer_next [DATA_BUF_IN_SIZE-1:0];

// Reset logic & sequential
genvar i;
generate
for (i = 0; i < DATA_BUF_IN_SIZE; i = i + 1) begin : bufffer_gen
    always @ (posedge clk)
    begin
        if (!rst_n)
        begin
            buffer[i] <= 0;
        end else begin
            buffer[i] <= buffer_next[i];
        end
    end
end
endgenerate

always @ (posedge clk)
begin
    if (!rst_n)
    begin
        index <= 0;
        state <= 0;
    end else begin
        index <= index_next;
        state <= state_next;
    end
end

// Combinational
generate
for (i = 0; i < DATA_BUF_IN_SIZE; i = i + 1) begin : msg_gen
    assign msg[(i+1)*`NOC_DATA_WIDTH - 1 -: `NOC_DATA_WIDTH] = buffer[i];
end
endgenerate


always @ *
begin
    buffer_next[0] = buffer[0];
    if (state == `DCP_NOC_STATE_IDLE) begin
        if (noc_in_val)
        begin
           buffer_next[0] = noc_in_data;
        end
    end
    else if (state == `DCP_NOC_STATE_RECEIVING) begin
        if (noc_in_val && index == 0)
        begin
           buffer_next[0] = noc_in_data;
        end
    end
end

generate
for (i = 1; i < DATA_BUF_IN_SIZE; i = i + 1) begin : buffer_next_gen
    always @ *
    begin
        buffer_next[i] = buffer[i];
        if (state == `DCP_NOC_STATE_RECEIVING)
        begin
            if (noc_in_val && index == i)
            begin
                buffer_next[i] = noc_in_data;
            end
        end
    end
end
endgenerate

always @ *
begin
    index_next = index;
    state_next = 0;
    msg_val = 0;
    msg_len = 0;
    noc_in_rdy = 1'b0;

    if (state == `DCP_NOC_STATE_IDLE)
    begin
        noc_in_rdy = 1'b1;
        msg_len    = noc_in_data[`MSG_LENGTH];
        if (noc_in_val)
        begin
            if (msg_len == 0)
            begin
                state_next = `DCP_NOC_STATE_WAITING_ACK;
            end
            else
            begin
                state_next = `DCP_NOC_STATE_RECEIVING;
                index_next = index + `DCP_NOC3_FLIT_STATE_WIDTH'd1;
            end
        end
    end
    // receive data
    else if (state == `DCP_NOC_STATE_RECEIVING)
    begin
        noc_in_rdy = 1'b1;
        msg_len    = buffer[0][`MSG_LENGTH];
        if (noc_in_val)
        begin
            if (index == msg_len)
            begin
                state_next = `DCP_NOC_STATE_WAITING_ACK;
            end
            else
            begin
                state_next = `DCP_NOC_STATE_RECEIVING;
                index_next = index + `DCP_NOC3_FLIT_STATE_WIDTH'd1;
            end
        end
        else state_next = state;
    end
    // wait for decoder to read the message
    else if (state == `DCP_NOC_STATE_WAITING_ACK)
    begin
        noc_in_rdy = 1'b0;
        msg_val    = 1'b1;
        if (msg_ack)
            begin
                state_next = `DCP_NOC_STATE_IDLE;
                index_next = `DCP_NOC3_FLIT_STATE_WIDTH'd0;
            end
        else
            state_next = `DCP_NOC_STATE_WAITING_ACK;
    end
end
endmodule
