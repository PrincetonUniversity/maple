#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#if SIZE == 5
  #include "../maple/tests/data/bfs_data_youtube.h"
#elif SIZE == 4
  #include "../maple/tests/data/bfs_data_amazon.h"
#elif SIZE == 3
  #include "../maple/tests/data/bfs_data_big.h"
#elif SIZE == 2 
#include "../maple/tests/data/bfs_data_small.h"
#else
    #include "../maple/tests/data/bfs_data_tiny.h"
#endif

void _kernel_(uint32_t id, uint32_t core_num){
    volatile static uint32_t sync0 = 0;
    volatile static uint32_t sync1 = 0;
    volatile static uint32_t sync2 = 0;
    
    static uint32_t hop = 1;
    static uint32_t hopm = 0;
    volatile static uint32_t num_changed = 1;
    
    #ifdef LOOP // Undefined by default, to compare a single Epoch with DCP 
    while (num_changed > 0) {
    #endif   
        for (uint32_t n = id; n < R; n+=core_num) {
          if (ret_prop[n] == -1) {
            for (unsigned int e = node_array[n]; e < node_array[n+1]; e++) {
              uint32_t edge_index = edge_array[e];
              uint32_t to_reduce = ret_prop[edge_index];
              if (to_reduce == hopm) {
                ret_tmp[n] = hop;
              }
            }
          }     
        }
        #ifdef LOOP
        if (id==0) sync2=0;
        ATOMIC_OP(sync1, 1, add, w);
        while(sync1 != core_num); 
    
        if (id==0){ hopm = hop; hop += 1;num_changed=0;}
        ATOMIC_OP(sync0, 1, add, w);
        while(sync0 != core_num); 
    
        // Apply Phase
        for (uint32_t n = id; n < R; n+=core_num) {
          if (ret_tmp[n] == hopm) {
            num_changed += 1;
            ret_prop[n] = ret_tmp[n];
          }
        }    
        if (id==0) {sync0=0; sync1=0;};
        ATOMIC_OP(sync2, 1, add, w);
        while(sync2 != (core_num));
    }
    #endif
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
