#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"

#define NUM_WORDS 16
static uint64_t A,D;
static uint32_t Dp[16] = {0x00000000,0x00000001,0x00000002,0x00000003,0x00000004,0x00000005,0x00000006,0x00000007,0x00000008,0x00000009,0x0000000A,0x0000000B,0x0000000C,0x0000000D,0x0000000E,0x0000000F};
static uint32_t secret = 0xdeadbeef;
static uint32_t recovered = 0x00000000;
static uint32_t Ap[NUM_WORDS] = {0x33221100,
                                                 0x77665544,
                                                 0xBBAA9988,
                                                 0xFFEEDDCC,
                                                 0x11111111,
                                                 0x22222222,
                                                 0x33333333,
                                                 0x44444444,
                                                 0x55555555,
                                                 0x66666666,
                                                 0x77777777,
                                                 0x88888888,
                                                 0x99999999,
                                                 0xAAAAAAAA,
                                                 0xBBBBBBBB,
                                                 0xCCCCCCCC};


void _kernel_(uint32_t id, uint32_t core_num){
    A = (uint64_t)Ap | 0x7F00000000LL;
    D = (uint64_t)Dp | 0x7F00000000LL;
    uint64_t ppn = ((uint64_t)Ap >> 12);
    uint64_t vpn = ((uint64_t)A >> 12);
    uint64_t mmpage = 0x0LL;
    mmpage |= ppn << 4; // [31:4] PPN
    mmpage |= vpn << 32; // [58:32] VPN
    dec_disable_tlb(0);
    //dec_set_tlb_ptbase(0,0);
    //dec_set_tlb_mmpage(0, mmpage);
    
    // print32("Trojan secret data",secret);

    int secret_length = 32;
    int channel_width = 4;
    int iterations = secret_length/channel_width;
    for (int i=0;i<iterations;i++){
        ///CONTEXT SWITCH///
        dec_fifo_cleanup(1);
        ///PROCESS A///
        uint32_t mask = secret & 0x0000000F;
        secret = secret >> 4;
        void * addr = &Dp[mask];
        dec_set_base32(0,addr);

        ///CONTEXT SWITCH///
        dec_fifo_cleanup(1);

        ///PROCESS B///
        dec_fifo_init_conf(2, DCP_SIZE_64, 0,0,LOOP_TLOAD64); 
        dec_open_producer(0);dec_open_consumer(0);
        dec_load32_asynci_llc(0,0);
        uint32_t res = dec_consume32(0);
        recovered = recovered | (res << (i*4));
        // print32("D",res);
        dec_close_producer(0); dec_close_consumer(0);
    }
}

int main(int argc, char ** argv) {
    volatile static uint32_t amo_cnt = 0;
    uint32_t id, core_num;
    id = argv[0][0];
    core_num = argv[0][1];
    dec_fifo_init_conf(4, DCP_SIZE_64,0,0,LOOP_TLOAD64);
    dec_open_producer(0); dec_open_consumer(0);
    int timer_qid = 2;
    dec_open_producer(timer_qid); dec_open_consumer(timer_qid);
    _kernel_(0,1);
    uint64_t time = print_st(timer_qid);
    dec_close_producer(timer_qid); dec_close_consumer(timer_qid);

    print32("Recovered secret",recovered);
    printf("T: %d\n",(int)time);
    return 0;
}