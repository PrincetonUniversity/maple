
#include <stdio.h>
#include "util.h"
#include "../maple/api/dcp_maple.h"
#include "../maple/tests/data/spmv_data_sq_small.h"

#ifndef NUM_A
    #define NUM_A 1
#endif

void _kernel_(uint32_t id, uint32_t core_num){
    dec_open_producer(id);
    dec_open_consumer(id);
    // We are telling LIMA to load the preset base addresses at dec_fifo_init_conf
    // from a range of [0,R)
    dec_loop(id,0,R);
    for (int i = id; i < R; i+=NUM_A){
        uint32_t dat = dec_consume32(id);
        if (ptr[i]!=dat) printf("D:%d\n",dat);
    }
}

int main(int argc, char ** argv) {
    volatile static uint32_t amo_cnt = 0;
    uint32_t id, core_num;
    id = argv[0][0];
    core_num = argv[0][1];
    // Setting up Array B base_addr as ptr. Operation Loop Produce means that 
    // when we do dec_loop, it will push the A+B[i] data directly into the queue
    // instead of loading the address &(A+B[i]), which would be LOOP_TLOAD32 (see dcpn.h line 68)
    if (id == 0) dec_fifo_init_conf(NUM_A, DCP_SIZE_64,0,ptr,LOOP_PRODUCE);
    ATOMIC_OP(amo_cnt, 1, add, w);
    while(core_num != amo_cnt);
    _kernel_(id,core_num);
    return 0;
}
 

