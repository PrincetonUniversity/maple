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

#ifndef NUM
    #define NUM NUM_A
#endif

#define RUN   (NUM * 5)
#define RUN_1 (RUN + 1)

// WORKLOAD MODE
#ifndef MODE
    #define MODE 1 // 1 = LIMA, 2 = LIMA_LLC, 2 = LIMA PREFETCH,  3 = SW PREFETCH
#endif
#if MODE == 1
#define TYPE LOOP_TLOAD32
#elif MODE == 2
#define TYPE LOOP_LLC_32
#else
#define TYPE LOOP_PREFETCH
#endif

void _kernel_(uint32_t id, uint32_t core_num){
    uint32_t col, dense_idx;
    uint32_t shape_1 = G_shape[1];
    uint32_t shape_0 = G_shape[0];
    dec_open_producer(id);
    dec_open_consumer(id);

    for (uint32_t i = id; i < RUN; i+=NUM) {
        #ifdef PRI
        printf("P\n");
        #endif
        #if MODE <=3
        dec_loop(id, G_indptr[i],G_indptr[i+1]);
        #else
        for (int m = G_indptr[i]; m < G_indptr[i+1]; m++) PREFF(M[G_indices[m]]);
        #endif
    } 
    //COMPUTE
    uint32_t sparse, dense;
    uint64_t dense2; 
    for (uint32_t i = id; i < shape_0; i+=NUM){
        uint32_t end = G_indptr[i+1];
        uint32_t endm1 = end-1;
        for (uint32_t j = G_indptr[i]; j < end; j++) {
            uint32_t dat = G_data[j];
            #if MODE <=2
            if (j!=endm1){
                dense2 = dec_consume64(id);
                uint32_t dat_pre = dat;
                dat = G_data[j+1];
                dense = (uint32_t)(dense2 >> 32);
                result_data[j] = dat_pre * (uint32_t)(dense2);
                j++;
            }else{
                dense = dec_consume32(id);
            }
            #else
            dense_idx = i*shape_1+G_indices[j];
            dense = M[dense_idx];
            
            #endif
            result_data[j] = dat * dense;
            result_indices[j] = G_indices[j];
            #ifdef RES
            if (result_data2[j] != result_data[j]){
                printf("R%d-%d\n",result_data[j],result_data2[j]);
                }
            #endif
        } //end for
        if (i<(shape_0-RUN)){
            #ifdef PRI
            printf("P\n");
            #endif
            #if MODE <=3
            dec_loop(id, G_indptr[i+RUN],G_indptr[i+RUN_1]);
            #else
            for (int m = G_indptr[i+RUN]; m < G_indptr[i+RUN_1]; m++) PREFF(M[G_indices[m]]);
            #endif
        }
    }
    __sync_synchronize;
    //DISCONNECT
    volatile uint32_t close = dec_close_producer(id);
    volatile uint32_t close2 = dec_close_consumer(id);
}

int main(int argc, char ** argv) {
    uint64_t size = DCP_SIZE_64;
    #if NUM == 2 
    size=DCP_SIZE_64;
    #elif NUM > 2 
    size=DCP_SIZE_32;
    #endif
    
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];
    if (id == 0) dec_fifo_init_conf(NUM, size,M,G_indices,TYPE);
    _kernel_(id, core_num);
    if (id == 0) print_st(0);
    return result;
    #else
    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);
    touch(M,G_shape[0]*G_shape[1]);
    touch(G_indices,E);
    dec_fifo_init_conf(NUM, size,M,G_indices,TYPE); 
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        //printf("ID: %d\n", ide);
        _kernel_(ide, core_num);
    }
    print_st(0); 
    #endif
return 0;
}
