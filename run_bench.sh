#!/bin/bash

cd $PITON_ROOT/build
if [ ! -d res ]; then
  mkdir -p res;
fi

mask="0x00000001"
print="" #"-gcc_args=-DPRI=1"

##1 type (doall, dcpn, dcpl)
##2 kernel
##3 num_tiles
##4 NUM_A
##5 NUM_E
##6 SIZE dataset (1:Tiny, 2:SMALL 3:BIG)
##7 MODE 1:LIMA Loading from DRAM; 2:LIMA Loading from LLC; 3:LIMA Prefetching into LLC; 4:Software Prefetching 
run(){
    sims -sys=manycore -ariane -decoupling -vcs_run $1_$2.c -x_tiles=$3 -y_tiles=1 -finish_mask=$mask -rtl_timeout=10000000 -asm_diag_root=../maple -gcc_args=-DBARE_METAL -gcc_args=-DNUM_A=$4 -gcc_args=-DNUM_E=$5 -gcc_args=-DSIZE=$6 -gcc_args=-DMODE=$7 $print
    cp fake_uart.log res/$1_$2_N$4_D$6.txt
    cp sims.log res/$1_$2_N$4_D$6_sims.txt
}

if [ $# -eq 0 ]
then
    echo "No arguments supplied, Loop over options"
    for k in 1 #Size
    do
        for i in bfs #spmm ewsd bfs
        do # [type][name][tiles][A][E][Size][Mode]
            #run "doall" $i 3 1 1 $k 1
            #run "dcpl"  $i 1 1 1 $k 2
            #run "dcpl"  $i 3 2 1 $k 2
            run "dcpn"  $i 3 1 1 $k 1
        done
    done
else
    run $1 $2 $3 $4 $5 $6 $7
fi
