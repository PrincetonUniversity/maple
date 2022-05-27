
#ifndef __DCP_ALLOC_H
#define __DCP_ALLOC_H

#include <linux/kernel.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <endian.h>
#include <string.h>

#define MEM_REGION 0x00bf000000
#define PG_SIZE ((unsigned long)(1 << 12))

void * dec_malloc(unsigned long size) {
    // mmap /dev/mem so we can access the allocated memory
    int32_t fd = open("/dev/mem", O_RDWR);
    if (fd == -1) {
        perror("Page Alloc FD open Error");
        return 0;
    }
    uint64_t mmap_start = (uint64_t)MEM_REGION;
    //fprintf(stdout, "Start address %p\n", mmap_start);

    uint64_t * mmapped = (uint64_t *)mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, mmap_start);
    if (mmapped == (uint64_t *)-1) {
        perror("Page Alloc MMAP Error");
        return 0;
    }
    fprintf(stdout, "Page mmap-ed: address %p with size %lu\n", mmapped, size);
    return mmapped;
}

uint64_t * alloc_pages(unsigned int pages) {
    return (uint64_t *)dec_malloc(pages*PG_SIZE);
}

uint64_t alloc_tile(uint64_t tiles,uint64_t * base) {
    // mmap /dev/mem so we can access the allocated memory
    int32_t fd = open("/dev/mem", O_RDWR);
    if (fd == -1) {
        perror("Tile alloc FD open Error");
        return 0;
    }
    for (uint64_t i = 0; i < tiles; i++){
        //fprintf(stdout, "Start address %p\n", mmap_start);
        uint64_t * maped = (uint64_t *)mmap(NULL, (unsigned long)(PG_SIZE), PROT_READ|PROT_WRITE, MAP_SHARED, 
                                fd, base[i]);
        if (maped == (uint64_t *)-1) {
            perror("Tile alloc MMAP Error");
            return i;
        }
        //fprintf(stdout, "Tile %d mmap-ed: address %p\n",i, maped);
        base[i] = (uint64_t) maped;
    }
    // get result
    //fprintf(stdout, "Trying to read address %p\n", mmapped);
    //uint64_t data_val = *mmapped;
    //fprintf(stdout, "Result %x\n", data_val);  
    return tiles;
}

#endif
