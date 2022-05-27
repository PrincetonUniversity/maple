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

`ifdef DEFAULT_NETTYPE_NONE
`default_nettype none
`endif

module custom_acc (
    input  wire clk,
    input  wire rst_n,

    input  wire                             config_hsk,     // Any opcode into Custom Acc
    input  wire [19:0]                      config_addr,    // noc1decoder_dcp_address
    input  wire [31:0]                      config_data_hi, // noc1decoder_dcp_data
    input  wire [31:0]                      config_data_lo,
    input  wire                             config_load,    // It is a Load if set, else a Store
    input  wire [`MSG_DATA_SIZE_WIDTH-1:0]  config_size,  

    // NOC1 - Outgoing noshare_load/swap_wb request to L2 
    input  wire                             noc1buffer_rdy,
    output wire                             noc1buffer_val,
    output wire [`MSG_TYPE_WIDTH-1:0]       noc1buffer_type,
    output wire [`DCP_MSHRID_WIDTH -1:0]    noc1buffer_mshrid,
    output wire [`DCP_PADDR_MASK       ]    noc1buffer_address,
    output wire [`DCP_UNPARAM_2_0      ]    noc1buffer_size,
    output wire [`DCP_UNPARAM_63_0     ]    noc1buffer_data_0,
    output wire [`DCP_UNPARAM_63_0     ]    noc1buffer_data_1,
    output wire [`MSG_AMO_MASK_WIDTH-1:0]   noc1buffer_write_mask,

    // NOC2 - Incoming noshare_load/swap_wb response from L2
    input  wire                              noc2decoder_val,
    input  wire [`DCP_MSHRID_WIDTH -1:0]     noc2decoder_mshrid,
    input  wire [`DCP_NOC_RES_DATA_SIZE-1:0] noc2decoder_data,

    // Allow Ariane Core to load data from the custom Acc
    output wire [63:0]                      read_to_ariane_data,
    output wire                             read_to_ariane_val
);

// For the custom Acc we should use the upper 64 bits of the mshrid space of the DCP tile
wire [5:0] mem_req_transid;
assign noc1buffer_mshrid = {2'b11, mem_req_transid};

//Default to only 16B loads, change if needed! 
// Use DREAM_SW_WB for Stores!
//`MSG_DATA_SIZE_64B also possible!
assign noc1buffer_type = `DREAM_NS_LOAD;
assign noc1buffer_size = `MSG_DATA_SIZE_16B;
// We can write up to 16B at a time
assign noc1buffer_data_0 = 64'd0;
assign noc1buffer_data_1 = 64'd0;
// Mask to determine whether we write each of the 16 bytes
assign noc1buffer_write_mask = 16'd0; 

wire        resp_val,resp_rdy;
wire [63:0] resp_data;
wire        cmd_val, busy;
wire [5:0]  cmd_opcode;
wire [63:0] cmd_config_data;

tight_acc_iface u_tight_acc(
    .clk   (clk),
    .rst_n (rst_n),
    // Request iface to memory hierarchy
    .mem_req_val     (noc1buffer_val),
    .mem_req_rdy     (noc1buffer_rdy),
    .mem_req_transid (mem_req_transid),
    .mem_req_addr    (noc1buffer_address),
    // Response iface from memory hierarchy
    .mem_resp_val     (noc2decoder_val),
    .mem_resp_transid (noc2decoder_mshrid[5:0]),
    .mem_resp_data    (noc2decoder_data),
    // Command iface to receive "instructions" and configurations
    .cmd_val,
    .cmd_opcode,
    .cmd_config_data,
    .busy,
    // Response iface to the core
    .resp_val,
    .resp_rdy,
    .resp_data
);

// FIFO BUFFER MANAGEMENT
// The commands from the core to the Accelerator are buffered here 
// If the buffer should never be full since the configuration pipeline is non-blocking,
// One should make sure in software that we don't fill the buffer
// Should the cmd be blocking 
fifo_buffer #(
    .SIZE ( 70 ), //size of begin and end
    .INFLIGHT_IDX  ( 4 ) // Index 4, so 16 entries!
) request_buffer (
    .clk      (clk),
    .rst_n    (rst_n),
    .val_i    (config_hsk && !config_load),
    .rdy_o    (),
    .data_i   ({config_addr[8:3],config_data_hi,config_data_lo}),

    .val_o    (cmd_val),
    .rdy_i    (!busy),
    .data_o   ({cmd_opcode,cmd_config_data})
);

// The responses from the Accelerator are buffered here to later be collected by the core
// If the buffer is full we cannot take more responses from the accelerator
fifo_buffer #(
    .SIZE ( 64 ), //size of begin and end
    .INFLIGHT_IDX  ( 2 ) // Index 2, so 4 entries!
) response_buffer (
    .clk      (clk),
    .rst_n    (rst_n),
    .val_i    (resp_val),
    .rdy_o    (resp_rdy),
    .data_i   (resp_data),
// Return any value when a load request comes, keep in mind that there is no ready signal,
// so when a new load request comes we have to give some value that is available
// We should also make sure in SW that by the time the core consumes, we have some data ready
    .val_o    (read_to_ariane_val),
    .rdy_i    (config_hsk && config_load),
    .data_o   (read_to_ariane_data)
);

endmodule