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

void _kernel_(uint32_t id, uint32_t core_num) {
    uint32_t A_nrows = A_shape[0];
    uint32_t B_ncols = B_shape[1];
  // iterate through each column in B
  for(uint32_t j = id; j < B_ncols; j+=core_num) {
    //for(uint32_t i = 0; i < A_nrows; i++) spa[j*A_nrows + i] = 0;
    // iterate through each entry in the column
    uint32_t B_start = B_indptr[j];
    uint32_t B_end = B_indptr[j+1];
    for(uint32_t k = B_start; k < B_end; k++) {
      uint32_t B_idx = B_indices[k];
      uint32_t B = B_data[k];

      // find corresponding column in A for entry and iterate through each entry in that column
      uint32_t A_start = A_indptr[B_idx];
      uint32_t A_end = A_indptr[B_idx+1];
      for(uint32_t m = A_start; m < A_end; m++) {
        uint32_t A_idx = A_indices[m];
        uint32_t A = A_data[m];
        spa[j*A_nrows + A_idx] += A * B;
      }
    }
    uint32_t tmp_bias = bias[j];
    uint32_t tmp_C_indptr = C_indptr[j];
    for(uint32_t i = 0; i < A_nrows; i++) {
      uint32_t tmp_spa = spa[j*A_nrows + i];
      if(tmp_spa) {
        tmp_spa += tmp_bias;
        if(tmp_spa) {
          tmp_C_indptr++;
          tmp_C_indices[j*A_nrows + i] = i;
          spa[j*A_nrows + i] = tmp_spa;
          //tmp_spa = 0;
        }
      }
      //spa[j*A_nrows + i] = tmp_spa;
    }
    C_indptr[j+1] = tmp_C_indptr;
  }  
}

int main(int argc, char ** argv) {
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];
    start_doall(id, core_num);
    #else
    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        printf("ID:%d\n",ide);
        start_doall(ide, core_num);
    }
    #endif
return 0;
}
