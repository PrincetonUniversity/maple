//**************************************************************************
// Double-precision sparse matrix-vector multiplication benchmark
//--------------------------------------------------------------------------
#include <stdio.h>
#include "util.h"
#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#if SIZE == 5
    #include "../maple/tests/data/spmv_data_sq_big.h"
#elif SIZE == 4
    #include "../maple/tests/data/spmv_data_sq_small.h"
#elif SIZE == 3
    #include "../maple/tests/data/spmv_data_big.h"
#elif SIZE == 2 
    #include "../maple/tests/data/spmv_data_small.h"
#else
    #include "../maple/tests/data/spmv_data_tiny.h"
#endif
// Check response
#define RES 1

#ifndef NUM_A
    #define NUM_A 1
#endif
#ifndef NUM
    #define NUM NUM_A
#endif

#define RUN   (NUM * 1)
#define RUN_1 (RUN + 1)
#define R_RUN (R - RUN)

// WORKLOAD MODE
#ifndef MODE
    #define MODE 1 // 1 = LIMA, 2 = LIMA_LLC, 2 = LIMA PREFETCH,  3 = SW PREFETCH
#endif
#if MODE == 1
#define TYPE LOOP_TLOAD64
#elif MODE == 2
#define TYPE LOOP_LLC_64
#else
#define TYPE LOOP_PREFETCH
#endif

#ifndef TLB
    #define TLB 0
#endif

uint64_t prod_loop (uint32_t id, int i){
    uint64_t accum = 0;
    uint64_t dat;
    #ifdef PRI
    printf("C\n");
    #endif
    for (uint32_t k=ptr[i]; k < ptr[i+1]; k++){
        #if MODE <=2
        dat = dec_consume64(id);
        #else
        dat = x[idx[k]];
        #endif
        accum += val[k]*dat;
    }
    return accum;
}
void _kernel_(uint32_t id, uint32_t core_num){
    dec_open_producer(id);
    dec_open_consumer(id);
    uint32_t count = 0;
    for (int i = id; i < RUN; i+=NUM) {
        #ifdef PRI
        printf("P\n");
        #endif
        #if MODE <=3
        dec_loop(id, ptr[i],ptr[i+1]);
        #else
        for (int m = ptr[i]; m < ptr[i+1]; m++) PREFF(x[idx[m]]);
        #endif
    }
    //COMPUTE
    for (int i = id; i < R_RUN; i+=NUM){
        uint64_t accum = prod_loop(id,i);
        #ifdef RES
        if (accum != verify_data[i]) {printf("M%d-%d\n",i,ptr[i]); printf("R%d-%d\n",accum,verify_data[i]);result--;return;}
        #endif
        #if MODE <=3
        dec_loop(id, ptr[i+RUN],ptr[i+RUN_1]);
        #else
        for (int m = ptr[i+RUN]; m < ptr[i+RUN_1]; m++) PREFF(x[idx[m]]);
        #endif
    }
    uint32_t sta = R%2==0 ? id : (NUM-1)-id;
    for (int i = sta + R_RUN; i < R; i+=NUM){
        uint64_t accum = prod_loop(id,i);
        #ifdef RES
        if (accum != verify_data[i]) {printf("M%d-%d\n",i,ptr[i]); printf("R%d-%d\n",accum,verify_data[i]);result--;return;}
        #endif
    }
    __sync_synchronize;
    //DISCONNECT
    //printf("F\n");
    volatile uint32_t close = dec_close_producer(id);
    volatile uint32_t close2 = dec_close_consumer(id);
    printf("Z%d\n",count);
}

int main(int argc, char ** argv) {
    uint64_t size = DCP_SIZE_64;
    volatile static uint32_t amo_cnt = 0;
    #if NUM == 2 
    size=DCP_SIZE_64;
    #elif NUM > 2 
    size=DCP_SIZE_32;
    #endif
    
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];

    printf("ID: %d, of %d\n", id,core_num);
    
    if (id == 0){
        #if TLB == 1
        // TLB=1 Forces to use the TLB eveb with Bare-metal, having a 1-1 translation
        dec_fifo_init_conf(NUM, size,(uint64_t *)((uint64_t)x | 0x7F00000000LL),idx,TYPE);
        dec_set_tlb_ptbase(0,0);
        #else
        dec_fifo_init_conf(NUM, size,x,idx,TYPE);
        #endif
    }
    
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num != amo_cnt);
    _kernel_(id, core_num);
    #ifdef STAT
        if (id == 0) print_st(0);
    #endif
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num*2 != amo_cnt);
    return result;
    
    #else

    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);

    // 'Touch' walks over the pages to load them into the page table
    touch(idx,NNZ); touch64(x,C);
    dec_fifo_init_conf(NUM, size,x,idx,TYPE); 
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        printf("ID: %d\n", ide);
        _kernel_(ide, core_num);
    }
    print_st(0); 
    #endif
return 0;
}