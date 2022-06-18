#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#if SIZE == 5
#include "../maple/tests/data/spmm_data_sq495.h"
#elif SIZE == 4
#include "../maple/tests/data/spmm_data_sq818.h"
#elif SIZE == 3
    #include "../maple/tests/data/spmm_data_big.h"
#elif SIZE == 2 
    #include "../maple/tests/data/spmm_data_small.h"
#else
    #include "../maple/tests/data/spmm_data_tiny.h"
#endif

#ifndef NUM
    #define NUM NUM_A
#endif

#define RUN   (NUM * 5)
#define RUN_1 (RUN + 1)
// Check response
#define RES 1
// WORKLOAD MODE
#ifndef MODE
    #define MODE 1  // 1 = LIMA,  2 = LIMA PREFETCH,  3 = SW PREFETCH
#endif
#if MODE == 1
#define TYPE LOOP_TLOAD64
#else
#define TYPE LOOP_PREFETCH
#endif

void _kernel_(uint32_t id, uint32_t core_num){

    uint32_t A_nrows = A_shape[0];
    uint32_t B_ncols = B_shape[1];
    uint64_t aux;

    dec_open_producer(id);
    dec_open_consumer(id);
    for(uint32_t j = id; j < RUN; j+=NUM) {
        #ifdef PRI
        printf("P\n");
        #endif
        #if MODE <=2
        dec_loop(id, B_indptr[j],B_indptr[j+1]);
        #else
        for (int m = B_indptr[j]; m < B_indptr[j+1]; m++) PREFF(A_indptr[B_indices[m]]);
        #endif
    }
    //COMPUTE
    for(uint32_t j = id; j < B_ncols; j+=NUM) {
        #ifdef PRI
        printf("C\n");
        #endif
        for(uint32_t k = B_indptr[j]; k < B_indptr[j+1]; k++) {
            #if MODE <=1
            aux = dec_consume64(id);
            uint32_t A_start = aux;
            uint32_t A_end = aux >> 32;
            #else 
            uint32_t A_start = A_indptr[B_indices[k]];
            uint32_t A_end = A_indptr[B_indices[k+1]];
            #endif
            uint32_t B = B_data[k];
            for(uint32_t m = A_start; m < A_end; m++) {
                uint32_t idx = A_indices[m];
                uint32_t A = A_data[m];
                spa[j*A_nrows+idx] += A * B;
            }
        }
        uint32_t tmp_C_indptr = C_indptr[j];
        uint32_t tmp_bias = bias[j];
        for(uint32_t i = 0; i < A_nrows; i++) {
          uint32_t tmp_spa = spa[j*A_nrows + i];
          if(tmp_spa) {
            tmp_spa += tmp_bias;
            if(tmp_spa) {
                tmp_C_indptr++;
                tmp_C_indices[j*A_nrows + i] = i;
                spa[j*A_nrows + i] = tmp_spa;
            }
          }
        }
        C_indptr[j+1] = tmp_C_indptr;
        if (j<(B_ncols-RUN)){
            #ifdef PRI
            printf("P\n");
            #endif
            #if MODE <=2
            dec_loop(id, B_indptr[j+RUN],B_indptr[j+RUN_1]);
            #else
            for (int m = B_indptr[j+RUN]; m < B_indptr[j+RUN_1]; m++) PREFF(A_indptr[B_indices[m]]);
            #endif
        }

     } //end j

    __sync_synchronize;
    //DISCONNECT
    volatile uint32_t close = dec_close_producer(id);
    volatile uint32_t close2 = dec_close_consumer(id);
}

int main(int argc, char ** argv) {
    uint64_t size = DCP_SIZE_128;
    #if NUM == 2 
    size=DCP_SIZE_64;
    #elif NUM > 2 
    size=DCP_SIZE_32;
    #endif
    
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];
    if (id == 0) dec_fifo_init_conf(NUM, size,A_indptr,B_indices,TYPE);
    _kernel_(id, core_num);
    if (id == 0) print_st(0);
    #else
    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);
    touch(A_indptr,A_shape[1]+1); touch(B_indices,BNNZ);
    dec_fifo_init_conf(NUM, size,A_indptr,B_indices,TYPE);
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



