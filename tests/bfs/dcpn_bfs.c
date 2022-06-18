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
#define FINE 1
static uint32_t * po;

// A single Epoch of the Breadth-First-Search (Pull)
void _kernel_(uint32_t id, uint32_t core_num){
uint32_t exec_id=id-NUM_A;
uint32_t hop = 1;
uint32_t hopm = 2;//0

if(id < NUM_A) {
    //CONNECT
    dec_open_producer(id);
    #ifdef PRI
    printf("S\n");
    #endif

    //SUPPLY MODE
    #ifdef FINE
    for (uint32_t i = id; i < R; i+=NUM_A) {
    #else
    int rem = R%NUM_A;
    int job = R/NUM_A;
    int last = (id == (NUM_A-1));
    int init = id*job;
    for (uint32_t i = init; i < init + job + last*rem; i++){
    #endif
        if (ret_prop[i] == -1) {
            #ifdef PRI
            printf("P\n");
            #endif
            for (uint32_t e = node_array[i]; e < node_array[i+1]; e++) {
              uint32_t edge_index = edge_array[e];
              dec_load32_async(id,&ret_prop[edge_index]);
              //dec_produce32(id,ret_prop[edge_index]);
            }
        }     
    }
    __sync_synchronize;
    //DISCONNECT
    dec_close_producer(id);

} else {
    uint32_t dat;
    //CONNECT 
    dec_open_consumer(exec_id);

    //COMPUTE
    #ifdef FINE
    for (uint32_t i = exec_id; i < R; i+=NUM_E){
    #else
    int rem = R%NUM_E;
    int job = R/NUM_E;
    int last = (exec_id == (NUM_E-1));
    int init = exec_id*job;
    for (uint32_t i = init; i < init + job + last*rem; i++){
    #endif
        if (ret_prop[i] == -1) {
            #ifdef PRI
            printf("C\n");
            #endif
            uint32_t end = node_array[i+1];
            //uint32_t endm1 = end-1;
            for (uint32_t k=node_array[i]; k < end; k++){
                /* ONE CAN CONSUME TWO WORDS AT A TIME
                if (k!=endm1){
                    uint64_t aux = dec_consume64(exec_id);
                    k++;
                    dat = (uint32_t)(aux >> 32);
                    if ( (uint32_t)aux==hopm || dat==hopm) ret_tmp[i] = hop;
                }else{
                */
                    dat = dec_consume32(exec_id);
                    if (dat == hopm) ret_tmp[i] = hop;
                //}
            } //endfor k
        } //endif 
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
    LK;printf("ID: %d, of %d\n", id,core_num);ULK;

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
    //touch(ret_prop,R);
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
