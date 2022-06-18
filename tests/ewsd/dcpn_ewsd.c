#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#if SIZE >= 3
    #include "../maple/tests/data/ewsd_data_big.h"
#elif SIZE == 2 
    #include "../maple/tests/data/ewsd_data_small.h"
#else
    #include "../maple/tests/data/ewsd_data_tiny.h"
    #define RES 1
#endif

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

//Type of parallelization
#define FINE 1
//#define DOUBLEP 1
//#define DOUBLEC 1

void _kernel_(uint32_t id, uint32_t core_num){
    uint32_t exec_id=id-NUM_A;
    uint32_t shape_0 = G_shape[0];
    uint32_t shape_1 = G_shape[1];

    if(id < NUM_A) {
        #ifdef MAP
        for (int k = 0; k < NUM_E; k++){
            dec_open_producer(id+k*NUM_A);
        }
        #else
        dec_open_producer(id);
        #endif

        //COMPUTE
        uint32_t col, dense_idx, dense_idx2;
        #ifdef FINE
        for (uint32_t i = id; i < shape_0; i+=NUM_A) {
        #else
        uint32_t rem = shape_0%NUM_A;
        uint32_t job = shape_0/NUM_A;
        uint32_t last = (id == (NUM_A-1));
        uint32_t init = id*job;
        for (uint32_t i = init; i < init + job + last*rem; i++){
        #endif
            #ifdef MAP
            uint32_t fif = i%NUM;
            #else 
            uint32_t fif = id;
            #endif
            #ifdef PRI
            printf("P\n");
            #endif
            uint32_t end = G_indptr[i+1];
            uint32_t endm1 = end-1;
            for (uint32_t j = G_indptr[i]; j < end; j++) {
                dense_idx = i*shape_1+G_indices[j];
                #ifdef DOUBLEP
                if (j!=endm1){
                    dense_idx2 = i*shape_1+G_indices[j+1];
                    dec_load32_asynci(fif, ((uint64_t)&M[dense_idx]) << 32 | ((uint64_t)&M[dense_idx2]) ); j++;
                } else{
                #endif
                    #ifdef PRI
                    printf("D\n");
                    #endif
                    dec_load32_async(fif,&M[dense_idx]);
                    //dec_produce32(id,M[dense_idx]);
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
    } else {
        //CONNECT
        dec_open_consumer(exec_id);
        //COMPUTE
        uint32_t sparse, dense;
        uint64_t dense2; 
        
        #ifdef FINE
        for (uint32_t i = exec_id; i < shape_0; i+=NUM_E){
        #else
        uint32_t rem = shape_0%NUM_E;
        uint32_t job = shape_0/NUM_E;
        uint32_t last = (exec_id == (NUM_E-1));
        uint32_t init = exec_id*job;
        for (uint32_t i = init; i < init + job + last*rem; i++){
        #endif
            #ifdef PRI
            printf("C\n");
            #endif

            uint32_t end = G_indptr[i+1];
            uint32_t endm1 = end-1;
            for (uint32_t j = G_indptr[i]; j < end; j++) {
                uint32_t dat = G_data[j];
                #ifdef DOUBLEC
                if (j!=endm1){
                    dense2 = dec_consume64(exec_id);
                    uint32_t dat1 = dat;
                    dat = G_data[j+1];
                    dense = (uint32_t)(dense2 >> 32);
                    result_data[j] = dat1 * (uint32_t)(dense2);
                    j++;
                }else{
                #endif
                    #ifdef PRI
                    printf("S\n");
                    #endif
                    dense = dec_consume32(exec_id);
                    //dense = M[i*shape_1+G_indices[j]];
                #ifdef DOUBLEC
                }
                #endif
                result_data[j] = dat * dense;
                result_indices[j] = G_indices[j];
                #ifdef RES
                if (result_data2[j] != result_data[j]){
                    printf("M%d-%d\n",j,G_indptr[j]);
                    printf("R%d-%d\n",result_data[j],result_data2[j]);
                    }
                #endif
            }
        }
        __sync_synchronize;
        //DISCONNECT
        dec_close_consumer(exec_id);
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
    touch(M,G_shape[0]*G_shape[1]);
    init_tile(NUM);
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        printf("ID: %d\n", ide);
        #pragma omp barrier
        _kernel_(ide, core_num);
    }
    print_stats_fifos(NUM);
#endif
return 0;
}

