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

#define FINE 1

void _kernel_(uint32_t id, uint32_t core_num){
    uint32_t shape_0 = G_shape[0];
    uint32_t shape_1 = G_shape[1];
    uint32_t col, dense_idx, dense, start, end;
    #ifdef FINE
    for (uint32_t i = id; i < shape_0; i+=core_num){
    #else
    uint32_t rem = shape_0%core_num;
    uint32_t job = shape_0/core_num;
    uint32_t last = (id == (core_num-1));
    uint32_t init = id*job;
    for (uint32_t i = init; i < init + job + last*rem; i++){
    #endif
      start = G_indptr[i];
      end = G_indptr[i+1];

      for (uint32_t j = start; j < end; j++) {
        col = G_indices[j];
        dense_idx = i*shape_1+col;
        dense = M[dense_idx];
        result_data[j] = G_data[j] * dense;
        result_indices[j] = col;
        if (result_data2[j] != result_data[j]){
          printf("M%d-%d\n",j,G_indptr[j]);
          printf("R%d-%d\n",result_data[j],result_data2[j]);
          result=1;}
      }
    }
}

int main(int argc, char ** argv) {
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];
    start_doall(id, core_num);
    return result;
    #else
    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        //printf("ID:%d\n",ide);
        start_doall(ide, core_num);
    }
    #endif
return 0;
}
