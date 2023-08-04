#!/bin/bash

cd $PITON_ROOT/build
if [ ! -d res ]; then
  mkdir -p res;
fi

# Running Other tests
tests=("dma" "dcp_uni" "contiguous_allocation" "custom_acc" "autocc")
name=${tests[$1]}
sims -sys=manycore -ariane -decoupling -vcs_run ${name}.c -x_tiles=1 -y_tiles=1 -finish_mask=0x0001 -gcc_args=-DBARE_METAL -rtl_timeout=1000000 -asm_diag_root=../maple
cp fake_uart.log res/${name}.txt
cp sims.log res/${name}_sims.txt

echo "======================\n"
cat res/${name}.txt