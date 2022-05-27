// ========== Copyright Header Begin ============================================
// Copyright (c) 2020 Princeton University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Princeton University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ========== Copyright Header End ============================================

//==================================================================================================
//  Filename      : fifo_buffer.v
//  Created On    : 2020-07-02
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Buffer entries to accumulate before requesting
//  Buffers in a FIFO fashion, without bypass 
//  (better timing and area, but 1 cycle delay between entrance and exit)
//==================================================================================================

module fifo_buffer
  #(
    // Configuration Parameters
    parameter INFLIGHT_IDX = 2,
    parameter SIZE = 64
  )
(
    // Clock + Reset
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          val_i,
    input  wire                          rdy_i,
    input  wire [SIZE-1:0]               data_i,

    output wire                          val_o,
    output wire [SIZE-1:0]               data_o, 
    output wire                          rdy_o
);

//==============================================================================
// Local Parameters
//==============================================================================

genvar j;
localparam INFLIGHT = 2**INFLIGHT_IDX;

reg [INFLIGHT    -1:0] buffer_val_r; 
reg [INFLIGHT_IDX-1:0] buffer_head_r;
reg [INFLIGHT_IDX-1:0] buffer_tail_r;
reg [SIZE-1:0] buffer_data_r [INFLIGHT-1:0];

// Hanshake Valid and Response is favorable (1)
wire in_hsk  = val_i && rdy_o;
wire out_hsk = val_o && rdy_i;

wire [INFLIGHT-1:0] add_buffer;
wire [INFLIGHT-1:0] clr_buffer;
assign add_buffer = ({{INFLIGHT-1{1'b0}}, 1'b1} << buffer_head_r) & {INFLIGHT{in_hsk}};
assign clr_buffer = ({{INFLIGHT-1{1'b0}}, 1'b1} << buffer_tail_r) & {INFLIGHT{out_hsk}};

always @(posedge clk) begin
    if (!rst_n) begin
        buffer_head_r <= {INFLIGHT_IDX{1'b0}};
        buffer_tail_r <= {INFLIGHT_IDX{1'b0}};
    end else begin
        if (in_hsk) begin
            buffer_head_r <= buffer_head_r + {{INFLIGHT_IDX-1{1'b0}}, 1'b1};
        end
        if (out_hsk) begin
            buffer_tail_r <= buffer_tail_r + {{INFLIGHT_IDX-1{1'b0}}, 1'b1};
        end
    end //end else
end

generate
    for ( j = 0; j < INFLIGHT; j = j + 1) begin: buffers_gen
        always @(posedge clk) begin
            if (!rst_n) begin
              buffer_val_r [j] <= 1'b0;
            end else begin
              buffer_val_r[j] <= add_buffer[j] || buffer_val_r[j] && !clr_buffer[j];
            end
        end

        always @(posedge clk) begin
            if (add_buffer[j]) begin
                buffer_data_r[j] <= data_i;
            end 
        end
    end
endgenerate

assign data_o = buffer_data_r[buffer_tail_r];
assign val_o  = |buffer_val_r;
assign rdy_o  = !(&buffer_val_r);

endmodule