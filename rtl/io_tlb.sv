// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: David Schaffenrath, TU Graz
// Author: Florian Zaruba, ETH Zurich
// Date: 21.4.2017
// Description: Translation Lookaside Buffer, SV39 fully set-associative

// Modified by Princeton University on June 16th, 2020
//==================================================================================================
//  Filename      : io_tlb.v
//  Created On    : 2020-06-16
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : TLB of IOMMU for IS Tile 
//==================================================================================================

`include "l15.tmp.h"
`include "is.h"

module io_tlb #(
      parameter TLB_ENTRIES = 4,
      parameter ASID_WIDTH  = 1,
      parameter VADDR = 64
  )(
    input  logic                    clk_i,    // Clock
    input  logic                    rst_ni,   // Asynchronous reset active low
    input  logic                    flush_i,  // Flush signal
    // Update TLB
    input  logic                    update_valid_i,
    input  logic [1:0]              update_size_i,
    input  logic [26:0]             update_vpn_i,
    input  logic [ASID_WIDTH-1:0]   update_asid_i,
    input  logic [`PTE_WIDTH-1:0]   update_content_i,
    // Lookup signals
    input  logic                    lu_access_i,
    input  logic [ASID_WIDTH-1:0]   lu_asid_i,
    input  logic [VADDR     -1:0]   lu_vaddr_i,
    output logic [`PTE_WIDTH-1:0]   lu_content_o,
    output logic                    lu_is_2M_o,
    output logic                    lu_is_1G_o,
    output logic                    lu_hit_o,

    input  logic                    tlb_snoop_val,
    output wire  [63:0]             tlb_snoop_entry              
);
localparam TLB_ENTRY_IDX = $clog2(TLB_ENTRIES);
    logic [TLB_ENTRY_IDX-1:0] snoop_idx;
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            snoop_idx <= {TLB_ENTRY_IDX{1'b0}};
        end else if (tlb_snoop_val) begin
            snoop_idx <= snoop_idx + 1'd1;
        end
    end

    // SV39 defines three levels of page tables
    struct packed {
      logic [ASID_WIDTH-1:0] asid;
      logic [8:0]            vpn2;
      logic [8:0]            vpn1;
      logic [8:0]            vpn0;
      logic                  is_2M;
      logic                  is_1G;
      logic                  valid;
    } [TLB_ENTRIES-1:0] tags_q, tags_n;

    logic [`PTE_WIDTH-1:0] content_q [TLB_ENTRIES-1:0];
    logic [`PTE_WIDTH-1:0] content_n [TLB_ENTRIES-1:0];
    logic [8:0] vpn0, vpn1, vpn2;
    logic [TLB_ENTRIES-1:0] lu_hit;     // to replacement logic
    logic [TLB_ENTRIES-1:0] replace_en; // replace the following entry, set by replacement strategy
    
    wire [`PTE_ADDR-1:0] pte_ppn;
    wire [9:0]           pte_flags;
    wire [30:0] tag = tags_q[snoop_idx];
    assign {pte_ppn,pte_flags} = content_q[snoop_idx];
    assign tlb_snoop_entry = {pte_flags[4:1], tag[30:3], pte_ppn, pte_flags[`PTE_D], tag[2:0]};
    //-------------
    // Translation
    //-------------
    always_comb begin : translation
        vpn0 = lu_vaddr_i[20:12];
        vpn1 = lu_vaddr_i[29:21];
        vpn2 = lu_vaddr_i[38:30];

        // default assignment
        lu_hit       = '{default: 0};
        lu_hit_o     = 1'b0;
        lu_content_o = '{default: 0};
        lu_is_1G_o   = 1'b0;
        lu_is_2M_o   = 1'b0;

        for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
            // first level match, this may be a giga page, check the ASID flags as well
            if (tags_q[i].valid && lu_asid_i == tags_q[i].asid && vpn2 == tags_q[i].vpn2) begin
                // second level
                if (tags_q[i].is_1G) begin
                    lu_is_1G_o = 1'b1;
                    lu_content_o = content_q[i];
                    lu_hit_o   = 1'b1;
                    lu_hit[i]  = 1'b1;
                // not a giga page hit so check further
                end else if (vpn1 == tags_q[i].vpn1) begin
                    // this could be a 2 mega page hit or a 4 kB hit
                    // output accordingly
                    if (tags_q[i].is_2M || vpn0 == tags_q[i].vpn0) begin
                        lu_is_2M_o   = tags_q[i].is_2M;
                        lu_content_o = content_q[i];
                        lu_hit_o     = 1'b1;
                        lu_hit[i]    = 1'b1;
                    end
                end
            end
        end
    end

    // ------------------
    // Update and Flush
    // ------------------
    always_comb begin : update_flush
        tags_n    = tags_q;
        for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
            content_n[i] = content_q[i];
            if (flush_i) begin
                // invalidate logic
                if (lu_asid_i == {ASID_WIDTH{1'b0}} ) // flush everything if ASID is 0
                    tags_n[i].valid = 1'b0;
                else if (lu_asid_i == tags_q[i].asid) // just flush entries from this ASID
                    tags_n[i].valid = 1'b0;

            // normal replacement
            end else if (update_valid_i & replace_en[i]) begin
                // update tag array
                tags_n[i] = '{
                    asid:  update_asid_i,
                    vpn2:  update_vpn_i [26:18],
                    vpn1:  update_vpn_i [17:9],
                    vpn0:  update_vpn_i [8:0],
                    is_1G: update_size_i[1],
                    is_2M: update_size_i[0],
                    valid: 1'b1
                };
                // and content as well
                content_n[i] = update_content_i;
            end
        end
    end

    // -----------------------------------------------
    // PLRU - Pseudo Least Recently Used Replacement
    // -----------------------------------------------
    logic[2*(TLB_ENTRIES-1)-1:0] plru_tree_q, plru_tree_n;
    always_comb begin : plru_replacement
        plru_tree_n = plru_tree_q;
        // The PLRU-tree indexing:
        // lvl0        0
        //            / \
        //           /   \
        // lvl1     1     2
        //         / \   / \
        // lvl2   3   4 5   6
        //       / \ /\/\  /\
        //      ... ... ... ...
        // Just predefine which nodes will be set/cleared
        // E.g. for a TLB with 8 entries, the for-loop is semantically
        // equivalent to the following pseudo-code:
        // unique case (1'b1)
        // lu_hit[7]: plru_tree_n[0, 2, 6] = {1, 1, 1};
        // lu_hit[6]: plru_tree_n[0, 2, 6] = {1, 1, 0};
        // lu_hit[5]: plru_tree_n[0, 2, 5] = {1, 0, 1};
        // lu_hit[4]: plru_tree_n[0, 2, 5] = {1, 0, 0};
        // lu_hit[3]: plru_tree_n[0, 1, 4] = {0, 1, 1};
        // lu_hit[2]: plru_tree_n[0, 1, 4] = {0, 1, 0};
        // lu_hit[1]: plru_tree_n[0, 1, 3] = {0, 0, 1};
        // lu_hit[0]: plru_tree_n[0, 1, 3] = {0, 0, 0};
        // default: begin /* No hit */ end
        // endcase
        for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
            automatic int unsigned idx_base, shift, new_index;
            // we got a hit so update the pointer as it was least recently used
            // UPDATE tree on ALLOCATION or on HIT
            if (update_valid_i && replace_en[i] || lu_hit[i] && lu_access_i) begin
                // Set the nodes to the values we would expect
                for (int unsigned lvl = 0; lvl < TLB_ENTRY_IDX; lvl++) begin
                  idx_base = $unsigned((2**lvl)-1);
                  // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
                  shift = TLB_ENTRY_IDX - lvl;
                  // to circumvent the 32 bit integer arithmetic assignment
                  new_index =  ~((i >> (shift-1)) & 32'b1);
                  plru_tree_n[idx_base + (i >> shift)] = new_index[0];
                end
            end
        end
        // Decode tree to write enable signals
        // Next for-loop basically creates the following logic for e.g. an 8 entry
        // TLB (note: pseudo-code obviously):
        // replace_en[7] = &plru_tree_q[ 6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,1}
        // replace_en[6] = &plru_tree_q[~6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,0}
        // replace_en[5] = &plru_tree_q[ 5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,1}
        // replace_en[4] = &plru_tree_q[~5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,0}
        // replace_en[3] = &plru_tree_q[ 4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,1}
        // replace_en[2] = &plru_tree_q[~4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,0}
        // replace_en[1] = &plru_tree_q[ 3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,1}
        // replace_en[0] = &plru_tree_q[~3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,0}
        // For each entry traverse the tree. If every tree-node matches,
        // the corresponding bit of the entry's index, this is
        // the next entry to replace.
        for (int unsigned i = 0; i < TLB_ENTRIES; i += 1) begin
            automatic logic en;
            automatic int unsigned idx_base, shift, new_index;
            en = 1'b1;
            for (int unsigned lvl = 0; lvl < TLB_ENTRY_IDX; lvl++) begin
                idx_base = $unsigned((2**lvl)-1);
                // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
                shift = TLB_ENTRY_IDX - lvl;

                // en &= plru_tree_q[idx_base + (i>>shift)] == ((i >> (shift-1)) & 1'b1);
                new_index =  (i >> (shift-1)) & 32'b1;
                if (new_index[0]) begin
                  en &= plru_tree_q[idx_base + (i>>shift)];
                end else begin
                  en &= ~plru_tree_q[idx_base + (i>>shift)];
                end
            end
            replace_en[i] = en;
        end
    end

    // sequential proces
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            tags_q      <= '{default: 0};
            plru_tree_q <= '{default: 0};
        end else begin
            tags_q      <= tags_n;
            plru_tree_q <= plru_tree_n;
        end
    end

    genvar j;
    generate for (j = 0; j < TLB_ENTRIES; j = j + 1) begin : content_gen 
        always_ff @(posedge clk_i or negedge rst_ni) begin
            if(~rst_ni) begin
                content_q[j] <= {`PTE_WIDTH{1'b0}};
            end else begin
                content_q[j] <= content_n[j];
            end
        end
    end endgenerate

    //--------------
    // Sanity checks
    //--------------

    //pragma translate_off
    `ifndef VERILATOR

    initial begin : p_assertions
      assert ((TLB_ENTRIES % 2 == 0) && (TLB_ENTRIES > 1))
        else begin $error("TLB size must be a multiple of 2 and greater than 1"); $stop(); end
      assert (ASID_WIDTH >= 1)
        else begin $error("ASID width must be at least 1"); $stop(); end
    end

    // Just for checking
    function int countSetBits(logic[TLB_ENTRIES-1:0] vector);
      automatic int count = 0;
      foreach (vector[idx]) begin
        count += vector[idx];
      end
      return count;
    endfunction

    assert property (@(posedge clk_i)(countSetBits(lu_hit) <= 1))
      else begin $error("More then one hit in TLB!"); $stop(); end
    assert property (@(posedge clk_i)(countSetBits(replace_en) <= 1))
      else begin $error("More then one TLB entry selected for next replace!"); $stop(); end

    `endif
    //pragma translate_on

endmodule
