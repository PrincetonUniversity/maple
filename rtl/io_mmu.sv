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
// Author: Florian Zaruba, ETH Zurich
// Date: 19/04/2017
// Description: Memory Management Unit for Ariane, contains TLB and
//              address translation unit. SV39 as defined in RISC-V
//              privilege specification 1.11-WIP

// Modified by Princeton University on June 16th, 2020, based on Ariane's MMU
//==================================================================================================
//  Filename      : io_mmu.v
//  Created On    : 2020-06-16
//  Revision      :
//  Author        : Marcelo Orenes Vera
//  Company       : Princeton University
//  Email         : movera@princeton.edu
//
//  Description   : IOMMU for IS Tile 
//==================================================================================================

`include "l15.tmp.h"
`include "is.h"

module io_mmu #(
      parameter DATA_TLB_ENTRIES      = 4,
      parameter ASID_WIDTH            = 1,
      parameter VADDR                 = 64
) (
        input  logic                            clk_i,
        input  logic                            rst_ni,

        /*AUTOSVA
        mmu: req --IN> res
        req_val = req_i
        res_val = req_i && hit_o

        l15_iface: l15_req --OUT> l15_res
        l15_req_val = io_mmu.l15_val_pre
        l15_req_rdy = l15_ack
        l15_res_val = l15_rvalid
        */

        // LSU interface
        // this is a more minimalistic interface because the actual addressing logic is handled
        // in the LSU as we distinguish load and stores, what we do here is simple address translation
        input  logic                            req_i,        // request address translation
        input  logic [VADDR-1:0]                vaddr_i,      // virtual address in
        input  logic                            is_store_i,   // the translation is requested by a store

        // if we need to walk the page table we can't grant in the same cycle
        // Cycle 0
        output logic                            hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
        output logic [`L15_PADDR_HI:0]          paddr_o,      // translated address
        // Cycle 1
        output logic                            valid_o,      // translation is valid
        output logic                            exc_val_o,    // address translation threw an exception
        output logic [VADDR-1:0]                ptw_addr_o,   // address that threw an exception

        // Update iface from DCP
        input wire [ASID_WIDTH-1:0]            asid_i,
        input wire                             flush_i,
        input wire                             tlb_disable,
        input wire                             tlb_conf_ptbase,
        input logic                            tlb_update,
        input logic [63:0]                     conf_data,

        // Performance counters
        output logic                            tlb_miss_o,
        input  logic                            tlb_snoop_val,
        output wire  [63:0]                     tlb_snoop_entry,

        // PTW memory interface
        output logic                    l15_store,
        output logic                    l15_interrupt,
        output logic                    l15_val,
        input  logic                    l15_ack,
        output logic [`L15_PADDR_HI:0]  l15_address,
        output logic [63:0]             l15_data,
        input  logic                    l15_rvalid,
        input  logic [63:0]             l15_rdata
);
    wire [9:0]  tlb_update_flags = 10'h3ff; // Entry bits {RSW,D,A,G,U,X,W,R,V}
    wire [1:0]  tlb_update_size = conf_data[63:62];
    wire [26:0] tlb_update_vpn  = conf_data[58:32];
    wire [`PTE_ADDR-1:0] tlb_update_content = conf_data[31:4];

    logic        daccess_err;   // insufficient privilege to access this data page (kernel page)
    logic        ptw_active;    // PTW is currently walking a page table
    logic        ptw_error;     // PTW threw an exception
    logic        exc_st_o;

    logic        update_valid;
    logic [VADDR-1:0] update_vaddr;
    logic [ASID_WIDTH-1:0]  update_asid;
    logic [1:0]  update_size;
    logic [26:0] update_vpn;
    logic [`PTE_WIDTH-1:0] update_content;

    logic [`PTE_WIDTH-1:0] tlb_content;
    logic        tlb_is_2M;
    logic        tlb_is_1G;
    logic        tlb_lu_hit;

    reg  tlb_conf_r;
    reg  waive_int_r;
    reg [`PTE_ADDR-1:0] tlb_ptbase_r;
    reg [33:0] tlb_intdata_r;
    wire tlb_en = tlb_conf_r;

    wire l15_val_pre;
    assign l15_val = l15_val_pre;

    io_tlb #(
        .TLB_ENTRIES     ( DATA_TLB_ENTRIES             ),
        .ASID_WIDTH      ( ASID_WIDTH                   ),
        .VADDR           ( VADDR                        )
    ) i_dtlb (
        .update_valid_i   ( tlb_update || update_valid  ),
        .update_asid_i    ( update_asid                 ),
        .update_vpn_i     ( tlb_update ? tlb_update_vpn : update_vpn          ),
        .update_content_i ( tlb_update ? {tlb_update_content,tlb_update_flags} : update_content  ),
        .update_size_i    ( tlb_update ? tlb_update_size : update_size        ),

        .lu_access_i      ( req_i                   ),
        .lu_asid_i        ( asid_i                      ),
        .lu_vaddr_i       ( vaddr_i                 ),
        .lu_content_o     ( tlb_content                ),

        .lu_is_2M_o       ( tlb_is_2M                  ),
        .lu_is_1G_o       ( tlb_is_1G                  ),
        .lu_hit_o         ( tlb_lu_hit                 ),
        .*
    );

    io_ptw  #(
        .ASID_WIDTH             ( ASID_WIDTH            ),
        .VADDR                  ( VADDR                 )
    ) i_ptw (
        .ptw_active_o           ( ptw_active            ),
        .ptw_error_o            ( ptw_error             ),

        .update_vaddr_o         ( update_vaddr          ),
        .update_valid_o         ( update_valid          ),
        .update_rdy_i           ( !tlb_update           ),
        .update_asid_o          ( update_asid           ),
        .update_vpn_o           ( update_vpn            ),
        .update_content_o       ( update_content        ),
        .update_size_o          ( update_size           ),

        .tlb_access_i          ( req_i             ),
        .tlb_hit_i             ( tlb_lu_hit        ),
        .tlb_vaddr_i           ( vaddr_i           ),
        .satp_ppn_i            (tlb_ptbase_r),
        .waive_int             (waive_int_r),
        .*
     );

    //-----------------------
    // Data Interface
    //-----------------------
    logic [VADDR-1:0] vaddr_n;
    logic [`PTE_WIDTH-1:0] tlb_pte_n, tlb_pte_q;
    logic [`PTE_ADDR- 1:0] pte_ppn_n;
    logic [`PTE_ADDR- 1:0] pte_ppn_q;
    logic [9:0]            pte_flags_n, pte_flags_q;
    logic        req_n,       req_q;
    logic        is_store_n,  is_store_q;
    logic        tlb_hit_n,      tlb_hit_q;
    logic        tlb_is_2M_n,    tlb_is_2M_q;
    logic        tlb_is_1G_n,    tlb_is_1G_q;

    assign {pte_ppn_q,pte_flags_q} = tlb_pte_q;
    assign {pte_ppn_n,pte_flags_n} = tlb_pte_n;
    // check if we need to do translation or if we are always ready (e.g.: we are not translating anything)
    assign hit_o = (tlb_en) ? tlb_lu_hit :  1'b1;

    // The data interface is simpler and only consists of a request/response interface
    always_comb begin : data_interface
        // save request and DTLB response
        vaddr_n           = vaddr_i;
        req_n             = req_i;
        tlb_pte_n         = tlb_content;
        tlb_hit_n         = tlb_lu_hit;
        is_store_n        = is_store_i;
        tlb_is_2M_n       = tlb_is_2M;
        tlb_is_1G_n       = tlb_is_1G;

        paddr_o           = vaddr_n[`L15_PADDR_HI:0];
        valid_o           = req_q;
        exc_val_o         = 1'b0;    // address translation threw an exception
        exc_st_o          = 1'b0;
        daccess_err       = !pte_flags_q[`PTE_U]; // this is not a user page but we are in user mode and trying to access it
        ptw_addr_o        = update_vaddr;
        // translation is enabled 
        if (tlb_en) begin
            valid_o = 1'b0;
            // 4K page
            paddr_o = {pte_ppn_n, vaddr_n[11:0]};
            // Mega page
            if (tlb_is_2M_q) begin
              paddr_o[20:12] = vaddr_n[20:12];
            end
            // Giga page
            if (tlb_is_1G_q) begin
                paddr_o[29:12] = vaddr_n[29:12];
            end
            // ---------
            // DTLB Hit
            // --------
            if (tlb_hit_q && req_q) begin
                valid_o = 1'b1;
                // this is a store
                if (is_store_q) begin
                    // check if the page is write-able and we are not violating privileges
                    // also check if the dirty flag is set (REVISIT exception if not dirty?)
                    if (!pte_flags_q[`PTE_W] || daccess_err || !pte_flags_q[`PTE_D]) begin
                        exc_st_o = 1'b1;
                        exc_val_o = 1'b1;
                    end
                // this is a load, check for sufficient access privileges - throw a page fault if necessary
                end else if (daccess_err) begin
                    // Should we raise exceptions while accessing kernel pages?
                    //exc_val_o = 1'b1;
                end
            end else
            // ---------
            // DTLB Miss
            // ---------
            // watch out for exceptions
            if (ptw_active) begin
                // page table walker threw an exception
                if (ptw_error) begin
                    // an error makes the translation valid
                    valid_o = 1'b1;
                    // the page table walker can only throw page faults
                    exc_st_o = is_store_q;
                    exc_val_o = 1'b1;
                end
            end
        end
    end
    // ----------
    // Registers
    // ----------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            req_q        <= '0;
            tlb_pte_q       <= '0;
            tlb_hit_q       <= '0;
            is_store_q   <= '0;
            tlb_is_2M_q     <= '0;
            tlb_is_1G_q     <= '0;
        end else begin
            req_q        <=  req_n;
            tlb_pte_q       <=  tlb_pte_n;
            tlb_hit_q       <=  tlb_hit_n;
            is_store_q   <=  is_store_n;
            tlb_is_2M_q     <=  tlb_is_2M_n;
            tlb_is_1G_q     <=  tlb_is_1G_n;
        end
    end

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            tlb_conf_r <= 1'b1;
        end else begin
            // Reset to 1'b1 during flush
            tlb_conf_r <= flush_i || tlb_conf_ptbase || tlb_conf_r && !tlb_disable;
        end
    end
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            waive_int_r   <= 1'b0;
            tlb_ptbase_r  <= 28'h0080004;
            // interrupt conf: Chip_id, Fbits, Y-ti, X-ti, type, threadid, ed/ra, source_id
            tlb_intdata_r <= {2'd0, 4'd0, 8'd3, 8'd0, 2'd0, 1'b0, 2'b10, 7'd2};
        end else if (tlb_conf_ptbase) begin
            waive_int_r   <= conf_data[63];
            tlb_ptbase_r  <= conf_data[27:0]; 
            tlb_intdata_r <= conf_data[61:28]; 
        end
    end

    // After a page fault, the MMU raises an interrupt by storing into the L15 as Non-cacheable
    // to a specific address (see inside PTW)
    // Note: MMU also uses the L15 (loads) to resolve TLB misses.

    assign l15_data = {4'h8, 3'd0, tlb_intdata_r[6], //bit 56 is source_id[6]
                      10'd0, tlb_intdata_r[33], 6'd0, tlb_intdata_r[32], //chipid 
                      tlb_intdata_r[31:10], // fbits, ypos, xpos, type
                      7'd0,  tlb_intdata_r[9], // threadid
                      tlb_intdata_r[8:7], tlb_intdata_r[5:0]}; // [7:6] edge/rising, [5:0] source id
endmodule
