// ========== Copyright Header Begin ============================================
// Copyright (c) 2019 Princeton University
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
//  Filename      : store_mshr.v
//  Created On    : 2019-12-30
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : Store pending requests
//  Store MSHR Unit is an ordered buffer for Store request (RMW, TLOAD A2E)
//  -The Decoupling pipe requests an entry and once this is acked, it issues the request to the NoC.
//  -Responses come OoO, but transactions leave the MSHR in order, when the HEAD is filled.
//  -For a transaction to leave the MSHR, a FIFO entry request has to be acked, that means we have space in corresponding FIFO to store the data.
//  -In the moment an entry is cleared in the MSHR, we send the Store ack to the Write buffer, so the Store can be considered as finished.
//  -Since the MSHR is bigger than the Write buffer, there is no risk of blocking the NOC used for Stores.
//  
//  This solution has the problem that the Store needs to wait to leave the MSHR to give the ack to the Write Buffer. Alternatively, we could
//  pre-allocate the FIFO entry inside the MSHR entry, to the MSHR has a guaranteed spot in the FIFO. Once this FIFO spot is acked, we could send
//  the ack response to the Write Buffer. That would cause that the WB keeps pushing to the MSHR, that is memory responses take to long, it may get full,
//  and therefore backpressure the NOC that communicates the Store from the WB to the MSHR. This is only a problem if responses come from that same NOC,
//  if it comes from a higher priority NOC, there is no problem, since responses would eventually come and MSHR would eventually drain cause the FIFO entry
//  is already booked. 
//  
//  Other option we could think of is to instead of having space inside the MSHR to store the reponse data, we are forced to book a FIFO entry before
//  requesting a MSHR entry. Problem is that if FIFO is full, we can have the case that the Store interface is blocking the NOC. Therefore we would need to
//  change the way we give the ACKs back to the WB. We could give them at the moment a FIFO entry is booked, if there are more than WB_SIZE empty slots in the FIFO.
//  In this option is complex to keep track of transactions that have given ACK and the ones that have not. Also FIFOs can have any arbitrary size, so if they are smaller
//  than WB_SIZE, we would run in deadlock issues.           
//==================================================================================================


module store_mshr 
  #(
    // Configuration Parameters
    parameter MB_SIZE_BITS = 5, // MISS BUFFER structure
    parameter MB_SIZE = 2 << MB_SIZE_BITS,
    parameter MB_DATA_BITS = 64
  )
(
    // Clock + Reset
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          mb_add_val, 
    output wire                          mb_add_rdy, 
    output wire                          mb_add_hsk, 
    output wire  [MB_SIZE_BITS-1:0]      mb_add_idx,

    output wire                          mb_clr_val, 
    input  wire                          mb_clr_rdy, 
    output wire  [MB_SIZE_BITS-1:0]      mb_clr_idx, 

    input  wire                          mb_fill_val,
    input  wire  [MB_SIZE_BITS-1:0]      mb_fill_idx,
    input  wire  [MB_DATA_BITS-1:0]      mb_fill_data
);

//==============================================================================
// Local Parameters
//==============================================================================


///////////////////////////
// Miss Buffer structure //
///////////////////////////

reg  [MB_SIZE-1:0][MB_DATA_BITS-1:0] mb_slot_data;
reg  [MB_SIZE-1:0] mb_slot_val;
reg  [MB_SIZE-1:0] mb_slot_pend;

wire [MB_SIZE-1:0] mb_fill_entry;
wire [MB_SIZE-1:0] mb_clr_entry;
wire [MB_SIZE-1:0] mb_add_entry;

wire               mb_clr_hsk;

reg [MB_SIZE_BITS-1:0 ] head;
reg [MB_SIZE_BITS-1:0 ] tail;

assign mb_add_rdy = !mb_slot_val[tail];
assign mb_add_hsk = mb_add_val && mb_add_rdy;
assign mb_add_idx = tail;

assign mb_clr_val = mb_slot_val[head] && !mb_slot_pend[head];
assign mb_clr_hsk = mb_clr_val && mb_clr_rdy;
assign mb_clr_idx = head;

genvar j;
generate
    for ( j = 0; j < MB_SIZE; j = j + 1) begin: mb_gen
        assign mb_fill_entry [j] = mb_fill_val && (j[MB_SIZE_BITS-1:0] == mb_fill_idx);
        assign mb_clr_entry[j] = mb_clr_val  && (j[MB_SIZE_BITS-1:0] == mb_clr_idx);
        assign mb_add_entry  [j] = mb_add_hsk  && (j[MB_SIZE_BITS-1:0] == mb_add_idx);

        always @(posedge clk) begin
              if (!rst_n) begin
                  mb_slot_val[j] <= 1'b0;
              end else begin
                  mb_slot_val[j] <= mb_add_entry[j] || mb_slot_val[j] && !mb_clr_entry[j];
              end
        end

        always @(posedge clk) begin
              mb_slot_pend[j] <= mb_add_entry[j] || mb_slot_pend[j] && !mb_fill_entry[j];
              if (mb_fill_entry[j]) begin
                  mb_slot_data[j] <= mb_fill_data;
              end
        end
    end //for
endgenerate
  
always @(posedge clk) begin
    if (!rst_n) begin
        head <= {MB_SIZE_BITS{1'b0}};
        tail <= {MB_SIZE_BITS{1'b0}};
    end else begin
        if (mb_add_hsk) begin // increment on allocation
              tail <= tail + {{MB_SIZE_BITS-1{1'b0}}, 1'b1};
        end
        if (mb_clr_hsk) begin //increment on commit
              head <= head + {{MB_SIZE_BITS-1{1'b0}}, 1'b1};
        end
    end //end else
end

endmodule
