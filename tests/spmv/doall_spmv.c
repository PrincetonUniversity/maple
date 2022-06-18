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

void _kernel_(uint32_t id, uint32_t core_num){
    #ifndef FINE
    int rem = R%core_num;
    int job = R/core_num;
    int last = (id == (core_num-1));
    int init = id*job;
    for (int i = init; i < init + job + last*rem; i++)
    #else
    for (int i = id; i < R; i+=core_num)
    #endif
    {
        int k;
        uint64_t yi0 = 0;
        for (k=ptr[i]; k < ptr[i+1]; k++) yi0 += val[k]*x[idx[k]];
        //data[i] = yi0;
        #ifdef RES
        if (yi0 != verify_data[i]) printf("M\n",i);
        #endif
    }
}

int main(int argc, char ** argv) {
    uint32_t id, core_num;
    #ifdef BARE_METAL
    id = argv[0][0];
    core_num = argv[0][1];
    LK;printf("ID: %d of %d\n", id, core_num);ULK
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


