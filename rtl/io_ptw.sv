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
// Date: 24.4.2017
// Description: Hardware-PTW

// Modified by Princeton University on June 16th, 2020, based on Ariane's MMU
//==================================================================================================
//  Filename      : io_ptw.v
//  Created On    : 2020-06-16
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : PTW of IOMMU for IS Tile 
//==================================================================================================

`include "l15.tmp.h"
`include "is.h"

module io_ptw #(
        parameter ASID_WIDTH = 1,
        parameter VADDR = 64
    )(
    input  logic                    clk_i,                  // Clock
    input  logic                    rst_ni,                 // Asynchronous reset active low
    input  logic                    flush_i,                // flush everything, we need to do this because
                                                            // actually everything we do is speculative at this stage
                                                            // e.g.: there could be a CSR instruction that changes everything
    input  logic                    tlb_en,                 // enable virtual memory translation for load/stores    // Config
    input  logic [ASID_WIDTH-1:0]   asid_i,
    input  logic [`PTE_ADDR-1:0]    satp_ppn_i, // ppn from satp
    input  logic                    waive_int,
    // to TLBs, update logic
    output logic                    ptw_active_o,
    output logic                    ptw_error_o,            // set when an error occurred
    input  logic                    update_rdy_i,
    output logic                    update_valid_o,
    output logic [1:0]              update_size_o,
    output logic [26:0]             update_vpn_o,
    output logic [ASID_WIDTH-1:0]   update_asid_o,
    output logic [`PTE_WIDTH-1:0]   update_content_o,
    output logic [VADDR-1:0]        update_vaddr_o,         // vaddr of the page that produced the error

    // from TLBs
    input  logic                    tlb_access_i,
    input  logic                    tlb_hit_i,
    input  logic [VADDR-1:0]        tlb_vaddr_i,
    input  logic                    is_store_i,         // this translation was triggered by a store

    // PTW memory interface to L15
    output logic                    l15_store,
    output logic                    l15_interrupt,
    output logic                    l15_val_pre,
    input  logic                    l15_ack,
    output logic [`L15_PADDR_HI:0]  l15_address,
    input  logic                    l15_rvalid,
    input  logic [63:0]             l15_rdata,

    // Performance counters
    output logic                    tlb_miss_o
);
    // input registers
    logic data_rvalid_q;
    logic [37:0] data_rdata_q;
    wire [63:0] data_rdata_n = {l15_rdata[7:0],l15_rdata[15:8],l15_rdata[23:16],l15_rdata[31:24],
                                l15_rdata[39:32],l15_rdata[47:40],l15_rdata[55:48],l15_rdata[63:56]};
    logic [`PTE_ADDR-1:0] pte_ppn;
    logic [9:0] pte_flags;
    assign {pte_ppn,pte_flags} = data_rdata_q;

    enum logic[2:0] {
      IDLE,
      WAIT_GRANT,
      PTE_LOOKUP,
      WAIT_RVALID,
      PROPAGATE_ERROR,
      WAIT_INTERRUPT_RESPONSE
    } state_q, state_d;

    // SV39 defines three levels of page tables
    enum logic [1:0] {
        LVL1, LVL2, LVL3
    } ptw_lvl_q, ptw_lvl_n;

    logic global_mapping_q, global_mapping_n;
    // register the ASID
    logic [ASID_WIDTH-1:0]  tlb_update_asid_q, tlb_update_asid_n;
    // register the VPN we need to walk, SV39 defines a 39 bit virtual address
    logic [VADDR-1:0] vaddr_q,   vaddr_n;
    // 4 byte aligned physical pointer
    logic[`L15_PADDR_HI:0] ptw_pptr_q, ptw_pptr_n;

    // Assignments
    assign update_vaddr_o  = vaddr_q;
    assign ptw_active_o    = (state_q != IDLE);

    // -----------
    // TLB Update
    // -----------
    assign update_vpn_o = vaddr_q[38:12];
    // update the correct page table level
    assign update_size_o = {ptw_lvl_q == LVL1, ptw_lvl_q == LVL2}; //1G, 2M
    // output the correct ASID
    assign update_asid_o = tlb_update_asid_q;
    // set the global mapping bit
    assign update_content_o = {pte_ppn, pte_flags | (global_mapping_q << 5)};

    assign l15_address = l15_interrupt ? 40'h9800000800 : ptw_pptr_q;

    //-------------------
    // Page table walker
    //-------------------
    // A virtual address va is translated into a physical address pa as follows:
    // 1. Let a be sptbr.ppn × PAGESIZE, and let i = LEVELS-1. (For Sv39,
    //    PAGESIZE=2^12 and LEVELS=3.)
    // 2. Let pte be the value of the PTE at address a+va.vpn[i]×PTESIZE. (For
    //    Sv32, PTESIZE=4.)
    // 3. If pte.v = 0, or if pte.r = 0 and pte.w = 1, stop and raise an access
    //    exception.
    // 4. Otherwise, the PTE is valid. If pte.r = 1 or pte.x = 1, go to step 5.
    //    Otherwise, this PTE is a pointer to the next level of the page table.
    //    Let i=i-1. If i < 0, stop and raise an access exception. Otherwise, let
    //    a = pte.ppn × PAGESIZE and go to step 2.
    // 5. A leaf PTE has been found. Determine if the requested memory access
    //    is allowed by the pte.r, pte.w, and pte.x bits. If not, stop and
    //    raise an access exception. Otherwise, the translation is successful.
    //    Set pte.a to 1, and, if the memory access is a store, set pte.d to 1.
    //    The translated physical address is given as follows:
    //      - pa.pgoff = va.pgoff.
    //      - If i > 0, then this is a superpage translation and
    //        pa.ppn[i-1:0] = va.vpn[i-1:0].
    //      - pa.ppn[LEVELS-1:i] = pte.ppn[LEVELS-1:i].
    always_comb begin : ptw
        // default assignments
        // PTW memory interface
        l15_val_pre     = 1'b0;
        l15_store       = 1'b0;
        l15_interrupt   = 1'b0;

        ptw_error_o           = 1'b0;
        update_valid_o        = 1'b0;
        ptw_lvl_n             = ptw_lvl_q;
        ptw_pptr_n            = ptw_pptr_q;
        state_d               = state_q;
        global_mapping_n      = global_mapping_q;
        // input registers
        tlb_update_asid_n     = tlb_update_asid_q;
        vaddr_n               = vaddr_q;
        tlb_miss_o           = 1'b0;

        case (state_q)

            IDLE: begin
                // by default we start with the top-most page table
                ptw_lvl_n        = LVL1;
                global_mapping_n = 1'b0;
                // if we got an DTLB miss
                if (tlb_en && tlb_access_i && !tlb_hit_i) begin
                    ptw_pptr_n          = {satp_ppn_i, tlb_vaddr_i[38:30], 3'b0};
                    tlb_update_asid_n   = asid_i;
                    vaddr_n             = tlb_vaddr_i;
                    state_d             = WAIT_GRANT;
                    tlb_miss_o         = 1'b1;
                end
            end

            WAIT_GRANT: begin
                // send a request out
                l15_val_pre = 1'b1;
                // wait for the WAIT_GRANT
                if (l15_ack) begin
                    state_d     = PTE_LOOKUP;
                end
            end

            PTE_LOOKUP: begin
                // we wait for the valid signal
                if (data_rvalid_q && update_rdy_i) begin

                    // check if the global mapping bit is set
                    if (pte_flags[`PTE_G])
                        global_mapping_n = 1'b1;

                    // -------------
                    // Invalid PTE
                    // -------------
                    // If pte.v = 0, or if pte.r = 0 and pte.w = 1, stop and raise a page-fault exception.
                    if (!pte_flags[`PTE_V] || (!pte_flags[`PTE_R] && pte_flags[`PTE_W]))
                        state_d = PROPAGATE_ERROR;
                    // -----------
                    // Valid PTE
                    // -----------
                    else begin
                        state_d = IDLE;
                        // it is a valid PTE
                        // if pte.r = 1 or pte.x = 1 it is a valid PTE
                        if (pte_flags[`PTE_R] || pte_flags[`PTE_X]) begin
                            // Valid translation found (either 1G, 2M or 4K entry)
                            // ------------
                            // Update DTLB
                            // ------------
                            // Check if the access flag has been set, otherwise throw a page-fault
                            // and let the software handle those bits.
                            // If page is not readable (there are no write-only pages)
                            // we can directly raise an error. This doesn't put a useless
                            // entry into the TLB.
                            if (pte_flags[`PTE_A] && (pte_flags[`PTE_R] || pte_flags[`PTE_X])) begin
                              update_valid_o = 1'b1;
                            end else begin
                              state_d   = PROPAGATE_ERROR;
                            end
                            // Request is a store: perform some additional checks
                            // If the request was a store and the page is not write-able, raise an error
                            // the same applies if the dirty flag is not set
                            if (is_store_i && (!pte_flags[`PTE_W] || !pte_flags[`PTE_D])) begin
                                update_valid_o = 1'b0;
                                state_d   = PROPAGATE_ERROR;
                            end
                            // check if the ppn is correctly aligned:
                            // 6. If i > 0 and pa.ppn[17 : 0] != 0, this is a misaligned superpage; stop and raise a page-fault
                            // exception.
                            if (ptw_lvl_q == LVL1 && pte_ppn[17:0] != '0) begin
                                state_d             = PROPAGATE_ERROR;
                                update_valid_o = 1'b0;
                            end else if (ptw_lvl_q == LVL2 && pte_ppn[8:0] != '0) begin
                                state_d             = PROPAGATE_ERROR;
                                update_valid_o = 1'b0;
                            end
                        // this is a pointer to the next TLB level
                        end else begin
                            // pointer to next level of page table
                            if (ptw_lvl_q == LVL1) begin
                                // we are in the second level now
                                ptw_lvl_n  = LVL2;
                                ptw_pptr_n = {pte_ppn, vaddr_q[29:21], 3'b0};
                            end

                            if (ptw_lvl_q == LVL2) begin
                                // here we received a pointer to the third level
                                ptw_lvl_n  = LVL3;
                                ptw_pptr_n = {pte_ppn, vaddr_q[20:12], 3'b0};
                            end

                            state_d = WAIT_GRANT;

                            if (ptw_lvl_q == LVL3) begin
                              // Should already be the last level page table => Error
                              ptw_lvl_n   = LVL3;
                              state_d = PROPAGATE_ERROR;
                            end
                        end
                    end
                end
                // we've got a data WAIT_GRANT so tell the cache that the tag is valid
            end
            // Propagate error to MMU/LSU
            PROPAGATE_ERROR: begin
                l15_val_pre = !waive_int;
                l15_store   = 1'b1;
                l15_interrupt = 1'b1;
                // wait for the WAIT_GRANT
                if (l15_ack || waive_int) begin
                    state_d = WAIT_INTERRUPT_RESPONSE;
                end
            end
            WAIT_INTERRUPT_RESPONSE: begin
                if (data_rvalid_q || waive_int) begin
                    state_d = IDLE;
                    ptw_error_o = 1'b1;
                end
            end
            // wait for the rvalid before going back to IDLE
            WAIT_RVALID: begin
                if (data_rvalid_q)
                    state_d = IDLE;
            end
            default: begin
                state_d = IDLE;
            end
        endcase

        // -------
        // Flush
        // -------
        // should we have flushed before we got an rvalid, wait for it until going back to IDLE
        if (flush_i) begin
            // on a flush check whether we are
            // 1. in the PTE Lookup check whether we still need to wait for an rvalid
            // 2. waiting for a grant, if so: wait for it
            // if not, go back to idle
            if ((state_q == PTE_LOOKUP && !data_rvalid_q) || ((state_q == WAIT_GRANT) && l15_ack))
                state_d = WAIT_RVALID;
            else
                state_d = IDLE;
        end
    end

    // sequential process
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            state_q            <= IDLE;
            ptw_lvl_q          <= LVL1;
            tlb_update_asid_q  <= '0;
            vaddr_q            <= '0;
            ptw_pptr_q         <= '0;
            global_mapping_q   <= 1'b0;
            data_rdata_q       <= '0;
            data_rvalid_q      <= 1'b0;
        end else begin
            state_q            <= state_d;
            ptw_pptr_q         <= ptw_pptr_n;
            ptw_lvl_q          <= ptw_lvl_n;
            tlb_update_asid_q  <= tlb_update_asid_n;
            vaddr_q            <= vaddr_n;
            global_mapping_q   <= global_mapping_n;
            data_rdata_q       <= data_rdata_n[37:0];
            data_rvalid_q      <= l15_rvalid || data_rvalid_q && !update_rdy_i;
        end
    end
endmodule
