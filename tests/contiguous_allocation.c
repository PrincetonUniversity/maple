//**************************************************************************
// Double-precision sparse matrix-vector multiplication benchmark
//--------------------------------------------------------------------------
#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#include "../maple/tests/data/spmv_data_tiny.h"

#ifndef NUM_A
    #define NUM_A 1
#endif
#define MODE 3  // 1 = FLUSH,  2 = LLC Load,  3 = PREFETCH

static uint32_t amo_cnt = 0;
static uint64_t pointer[C];
void _kernel_(uint32_t id, uint32_t core_num){
    if (id==0){
        printf("INIT: res=%d\n",dec_fifo_init(1,DCP_SIZE_128));
    
        #ifndef BARE_METAL
        pointer = alloc_pages(1);
        if (pointer == 0){
            printf("PAGE ALLOC FAILED\n");
            return 1;
        }
        #else
        uint64_t * pb = (uint64_t *)&pointer[0];
        printf("POINTER BASE%p\n",pb);
        #endif
        for (int i=0; i<C; i++){
            pointer[i] = x[i];
            printf("p[%d]=%d\n",i,pointer[i]);
        }
        #if MODE == 1
        uint64_t pb_flush = (uint64_t)pb & 0xffffffffc0 | 0xac00000000;
        uint64_t res = *(uint64_t *)(pb_flush); // Flush addr base, res=0
        printf("DATA COPIED SUCCESFULLY,%d\n",res);
        #endif
    }
    // barrier to make sure all tiles closed their fifos
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num != amo_cnt);

    dec_open_producer(id);
    for (int i = 0; i < R; i+=NUM_A) {
        for (int k=ptr[i]; k < ptr[i+1]; k++){
            #ifndef BARE_METAL
                dec_load64_async(0,((uint64_t *)MEM_REGION)+idx[k]);
            #else
                uint64_t * ptr = &pointer[idx[k]];
                #if MODE == 1
                dec_load64_async(id,ptr); //terminal load (to dram)
                #elif MODE == 2
                dec_load64_async_llc(id,ptr); //no_share load, loading from L2
                //dec_produce64(0,*ptr); //load po data and produce it to maple
                #elif MODE == 3
                dec_prefetch(id,ptr);
                #endif
            #endif
        }
    }
    
    //CONNECT
    dec_open_consumer(id);
    //COMPUTE
    for (int i = 0; i < R; i+=NUM_A){
        uint64_t yi0 = 0;
        uint32_t start = ptr[i];
        uint32_t end = ptr[i+1];
        uint64_t dat;
        for (uint32_t k=start; k < end; k++){
            #if MODE <=2
            dat = dec_consume64(id);
            #else
            dat = pointer[idx[k]];
            #endif
            yi0 += val[k]*dat;
        }
        printf("CONSUMED loop %d\n",i);
        if (yi0 != verify_data[i]){
            printf("MISCOMPARE: i=%d\n",i);
            return;
        }
    }
    __sync_synchronize;
    //DISCONNECT
    printf("CLOSING queues\n");
    dec_close_producer(id);
    dec_close_consumer(id);
    printf("CLOSED queues\n");
}

int main(int argc, char ** argv) {
    volatile static uint32_t amo_cnt = 0;
    uint32_t id, core_num;
    id = argv[0][0];
    core_num = argv[0][1];
    if (id == 0) init_tile(NUM_A);
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num != amo_cnt);
    _kernel_(id,core_num);
    return 0;
}

