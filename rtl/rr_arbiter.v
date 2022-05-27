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
//  Filename      : rr_arbiter.v
//  Created On    : 2020-29-12
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Round Robin Arbiter
// 
// README!
// MODE 1 should be used when we do not want the output to toggle once it has been accepted!
// SOURCES can only be 8, 4 or 2
//
//==================================================================================================

module rr_arbiter (
// Outputs
arb_src,
arb_src_oh,
arb_val,
// Inputs
clk, reset_n, stall, valid_source
);
parameter MODE = 0;
parameter SOURCES = 3;
localparam SOURCES_WIDTH = $clog2(SOURCES);
// MODE 1 should be used when we do not want the output to toggle once it has been accepted!

// Clock and reset
input wire                       clk;
input wire                       reset_n;

// Signal that arbitration should be stalled on the current
// selected source if it is valid. If it is no longer valid
// we can arbitrate to another source.
input wire                       stall;

// Input vector of valid sources to choose from when selecting
// the next source.
input wire [SOURCES-1:0]         valid_source;

// Vector with a bit set for the current source that holds
// the arbitration token.
output wire [SOURCES-1:0]        arb_src_oh;
output wire [SOURCES_WIDTH-1:0]  arb_src;
output wire                      arb_val;

// Vector with token bit
wire [SOURCES-1:0]               b;
wire [SOURCES-1:0]               valid_in;
reg [SOURCES-1:0]                token_bit_r;
reg [SOURCES-1:0]                token_bit_nxt;
reg token_val_r;

if (SOURCES == 8) begin
    assign arb_src = {|b[7:4], |b[7:6] || |b[3:2] ,b[1]||b[3]||b[5]||b[7]};
end else if (SOURCES == 4) begin
    assign arb_src = {|b[3:2], b[1]||b[3]};
end else begin
    assign arb_src = b[1];
end

generate if (MODE == 0) begin : simple_arb
    assign b = token_bit_nxt;
    assign arb_src_oh = valid_source & b;
    assign arb_val = |arb_src_oh;
    assign valid_in = valid_source;
end else begin : next_cycle_arb
    assign b = token_bit_r;
    assign arb_src_oh = b & {SOURCES{token_val_r}};
    assign arb_val = token_val_r;
    assign valid_in = valid_source & ~arb_src_oh;
end endgenerate

// Duplicate the enabled input vector and mask out the current source
// holding the token and all below. This makes it simple to find the next
// source that should get the token
wire [SOURCES-1:0]               mask;
wire [2*SOURCES-1:0]             masked_valids;
assign                           mask = ~(token_bit_r | (token_bit_r + {(SOURCES){1'b1}}));
assign                           masked_valids = {valid_in, valid_in & mask};

// Should we stall the token? If we have a stall and the token is at a
// valid source we must stall it. If the current selected source is not
// valid and there is another valid source we do not need to stall. In
// fact we should not stall it in order to avoid deadlocks when having a
// hierarchy of arbiters.
wire                             stall_token = stall && arb_val;

// Only enable arbitration when there is a valid request and no stall
wire                             arb_en = (|valid_in) && !stall_token;

// Loop termination value
reg                              found_bit_set;

// Search for the next input source that is valid and give it the token.
// If no other input sources are valid the token will remain where is was.
always @*
  begin : update_token
    integer i;

    // Default values
    token_bit_nxt = {SOURCES{1'b0}};
    found_bit_set = 1'b0;

    for (i=1; i < SOURCES*2; i=i+1)
      case (masked_valids[i] && !found_bit_set)
        1'b0 :;
        1'b1 : // Select a new source
          begin
            token_bit_nxt[i%SOURCES] = 1'b1;
            found_bit_set            = 1'b1;
          end
        default :
          begin
            token_bit_nxt = {SOURCES{1'bx}};
            found_bit_set =          1'bx;
          end
      endcase
  end

always @(posedge clk) begin
  if(!reset_n)
    token_val_r <= 1'b0;
  else
    token_val_r <= arb_en || token_val_r && stall;
end
always @(posedge clk) begin
  if(!reset_n)
    token_bit_r <= {{SOURCES-1{1'b0}}, 1'b1};
  else if (arb_en)
    token_bit_r <= token_bit_nxt;
end
endmodule // round_robin