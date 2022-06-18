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
    uint32_t A_nrows = A_shape[0];
    uint32_t B_ncols = B_shape[1];
    uint64_t aux;

    if(id < NUM_A) {
        uint32_t C_idx = 0;
        C_indptr[0] = 0;
        dec_open_producer(id);

        for(uint32_t j = id; j < B_ncols; j+=NUM_A){
            #ifdef PRI
            printf("P\n");
            #endif
            for(uint32_t k = B_indptr[j]; k < B_indptr[j+1]; k++) {
                uint32_t l = B_indices[k];
                uint32_t A_start = A_indptr[l];
                uint32_t A_end = A_indptr[l+1];
                aux = A_end;
                #ifdef PRI
                printf("A\n");
                #endif
                //dec_produce64(id, aux << 32 | (uint64_t)A_start);
                dec_produce32(id,A_start);
                dec_produce32(id,A_end);
                for(uint32_t m = A_start; m < A_end; m++){
                    uint32_t idx = A_indices[m];
                    uint32_t A = A_data[m];
                    aux = A;
                    dec_produce32(id,idx);
                    dec_produce32(id,A);
                    //dec_produce64(id, aux << 32 | (uint64_t) idx);
                    #ifdef PRI
                    printf("B\n");
                    #endif
                    //dec_load32_async(id,&A_indices[m]);
                    //dec_load32_async(id,&A_data[m]);
              }
          }
        }
        __sync_synchronize;
        //DISCONNECT
        dec_close_producer(id);
    } else {
        dec_open_consumer(exec_id);

        for(uint32_t j = exec_id; j < B_ncols; j+=NUM_E) {
            #ifdef PRI
            printf("C\n");
            #endif
            for(uint32_t k = B_indptr[j]; k < B_indptr[j+1]; k++) {
                #ifdef PRI
                printf("K\n");
                #endif
                //aux = dec_consume64(exec_id);
                //uint32_t A_start = aux;
                uint32_t A_start = dec_consume32(exec_id);
                //uint32_t A_end = aux >> 32;
                uint32_t A_end = dec_consume32(exec_id);
                uint32_t B = B_data[k];
                for(uint32_t m = A_start; m < A_end; m++) {
                    #ifdef PRI
                    printf("M\n");
                    #endif
                    // FASTER TO LOAD 64bits at once than 2 times 32 bits
                    //aux = dec_consume64(exec_id);
                    //uint32_t idx = (uint32_t)aux;
                    //uint32_t A = (uint32_t)(aux >> 32);
                    uint32_t idx = dec_consume32(exec_id);
                    uint32_t A = dec_consume32(exec_id);
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

       } //end j
     __sync_synchronize;

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
#else
    uint32_t core_num = NUM_A+NUM_E;
    #include <omp.h>
    omp_set_num_threads(core_num);
    touch(A_data,NNZ);
    touch(A_indices,NNZ);
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
