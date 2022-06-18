
//**************************************************************************
// Double-precision sparse matrix-vector multiplication benchmark
//--------------------------------------------------------------------------

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
#define RES 1
#define FINE 1
//#define DOUBLEP 1

#ifndef NUM_A
    #define NUM_A 1
#endif
#ifndef NUM_E
    #define NUM_E 1
#endif

#if NUM_A != NUM_E
    // NUM is the number of the opened FIFO Basically equals NUM_A * NUM_E
    #define NUM (NUM_A * NUM_E)
    #define MAP 1
#else
    // If we have same amount of A and E, FIFO count is NUM_A 
    #define NUM NUM_A
#endif


void _kernel_(uint32_t id, uint32_t core_num){
    uint32_t exec_id=id-NUM_A;

    if(id < NUM_A) {
        #ifdef MAP
        for (int k = 0; k < NUM_E; k++){
            dec_open_producer(id+k*NUM_A);
            dec_set_base64(id+k*NUM_A,x);
        }
        #else
        dec_open_producer(id);
        //Set the Base pointer, without it we would need to push &x[idx[k]]
        dec_set_base64(id,x);

        #endif
        for (int i = id; i < R; i+=NUM_A) {
            #ifdef MAP
            uint32_t fif = i%NUM;
            #else 
            uint32_t fif = id;
            #endif
            #ifdef PRI
            printf("P\n");
            #endif

            int end = ptr[i+1];
            int endm1 = end-1;
            for (int k=ptr[i]; k < end; k++){
                #ifdef DOUBLEP
                if (k!=endm1){
                    dec_load64_asynci(fif,((uint64_t)idx[k]) << 32 | ((uint64_t)idx[k+1]) ); 
                    k++;
                } else {
                #endif
                    #ifdef PRI
                    printf("D\n");
                    #endif
                    dec_load64_asynci_llc(fif,idx[k]);
                    //dec_produce64(fif,x[idx[k]]);
                    //dec_prefetchi(fif, idx[k]);
                #ifdef DOUBLEP
                }
                #endif
            }
        }
        __sync_synchronize;
        //DISCONNECT
        #ifdef MAP
        for (int k = 0; k < NUM_E; k++) dec_close_producer(id+k*NUM_A);
        #else
        dec_close_producer(id);
        #endif
    } else if (exec_id < NUM_E){
        //CONNECT
        #ifdef MAP
        for (int k = 0; k < NUM_A; k++) dec_open_consumer(exec_id+NUM_E*k);
        #else
        dec_open_consumer(exec_id);
        #endif
        //COMPUTE
        for (int i = exec_id; i < R; i+=NUM_E){
            #ifdef PRI
            printf("C\n");
            #endif
            #ifdef MAP
            uint32_t fifo = i%NUM;
            #else 
            uint32_t fifo = exec_id;
            #endif
            uint64_t yi0 = 0;
            uint32_t start = ptr[i];
            uint32_t end = ptr[i+1];
            uint64_t dat;
            for (uint32_t k=start; k < end; k++){
                #ifdef PRI
                printf("S\n");
                #endif
                dat = dec_consume64(fifo);
                //dat = x[idx[k]];
                yi0 += val[k]*dat;
            }
            #ifdef RES
            if (yi0 != verify_data[i]) {LK;printf("M%d-%d\n",i,ptr[i]); ULK; return;}
            #endif
        }
        __sync_synchronize;
        //DISCONNECT
        #ifdef MAP
        for (int k = 0; k < NUM_A; k++) dec_close_consumer(exec_id+NUM_E*k);
        #else
        dec_close_consumer(exec_id);
        #endif
    }
}
int main(int argc, char ** argv) {

#ifdef BARE_METAL
    // synchronization variable
    volatile static uint32_t amo_cnt = 0;
    volatile static uint32_t amo_cnt2 = 0;
    uint32_t id, core_num;
    id = argv[0][0];
    core_num = argv[0][1];
    if (id == 0) init_tile(NUM);
    LK;printf("ID: %d of %d\n", id, core_num);ULK
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num != amo_cnt);
    _kernel_(id,core_num);
    // barrier to make sure all tiles closed their fifos
    ATOMIC_OP(amo_cnt2, 1, add, w);
    while(core_num != amo_cnt2);
    if (id == 0) print_stats_fifos(NUM);
    return result;
#else
    uint32_t core_num = NUM_A+NUM_E;
    #include <omp.h>
    omp_set_num_threads(core_num);
    touch64(x,C);
    init_tile(NUM);
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        LK;printf("ID: %d\n", ide);ULK;
        #pragma omp barrier
        _kernel_(ide, core_num);
    }
    print_stats_fifos(NUM);
#endif
return 0;
}


