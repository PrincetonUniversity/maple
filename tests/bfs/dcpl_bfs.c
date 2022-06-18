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

// A single Epoch of the Breadth-First-Search (Pull)
void _kernel_(uint32_t id, uint32_t core_num){
    uint32_t hop = 1;
    uint32_t hopm = hop-1;
    dec_open_producer(id);
    dec_open_consumer(id);
    #ifdef PRI
    printf("S\n");
    #endif
    for (uint32_t i = id; i < RUN; i+=NUM) {
        if (ret_prop[i] == -1) {
            #ifdef PRI
            printf("A\n");
            #endif
            #if MODE <= 3
            dec_loop(id, node_array[i],node_array[i+1]);
            #else
            for (int m = node_array[i]; m < node_array[i+1]; m++) PREFF(ret_prop[edge_array[m]])
            #endif
        }
    }
    uint32_t dat;
    for (uint32_t i = id; i < R; i+=NUM){
        if (ret_prop[i] == -1) {
            #ifdef PRI
            printf("C\n");
            #endif
            uint32_t end = node_array[i+1];
            uint32_t endm1 = end-1;
            for (uint32_t k=node_array[i]; k < end; k++){
                #if MODE <= 2
                if (k!=endm1){
                    uint64_t aux = dec_consume64(id);
                    k++;
                    dat = (uint32_t)(aux >> 32);
                    if ( (uint32_t)aux==hopm || dat==hopm) ret_tmp[i] = hop;
                }else{
                    if (dec_consume32(id) == hopm) ret_tmp[i] = hop;
                }
                #else
                if (ret_prop[edge_array[k]] == hopm) ret_tmp[i] = hop;
                #endif
            } //endfor k
        } //endif 


        if (i<(R-RUN)){
            if (ret_prop[i+RUN] == -1) {
                #ifdef PRI
                printf("P\n");
                #endif
                #if MODE <= 3
                dec_loop(id, node_array[i+RUN],node_array[i+RUN_1]);
                #else
                for (int m = node_array[i+RUN]; m < node_array[i+RUN_1]; m++) PREFF(ret_prop[edge_array[m]])
                #endif
            }
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
    if (id == 0) dec_fifo_init_conf(NUM, size,ret_prop,edge_array,TYPE);
    _kernel_(id, core_num);
    if (id == 0) print_st(0);
    #else
    core_num = NUM_A;
    #include <omp.h>
    omp_set_num_threads(core_num);
    touch(ret_prop,R); touch(edge_array,E);
    dec_fifo_init_conf(NUM, size,ret_prop,edge_array,TYPE);
    #pragma omp parallel
    {
        uint32_t ide = omp_get_thread_num();
        printf("ID: %d:%d\n", ide,ret_prop[1]+ret_prop[1025]);
        _kernel_(ide, core_num);
    }
    print_st(0); 
    #endif
return 0;
}



