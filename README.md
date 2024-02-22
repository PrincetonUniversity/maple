# MAPLE (Memory Access Parallel Load Engine)
<img align="right" width="375" height="400" src="https://user-images.githubusercontent.com/55038083/175697160-4008adf9-8ddc-4374-9eb6-13f0d375c581.png">

This is the repository for the RTL and API of the [research paper "Tiny but Mighty: Designing and Realizing Scalable Latency Tolerance for Manycore SoCs"](https://dl.acm.org/doi/abs/10.1145/3470496.3527400
), to appear at the 49th International Symposium on Computer Architecture 

The correct citation for this work is:

```
@inproceedings{maple,
  author = {Orenes-Vera, Marcelo and Manocha, Aninda and Balkind, Jonathan and Gao, Fei and Arag\'{o}n, Juan L. and Wentzlaff, David and Martonosi, Margaret},
  title = {Tiny but Mighty: Designing and Realizing Scalable Latency Tolerance for Manycore SoCs},
  year = {2022},
  isbn = {9781450386104},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  url = {https://doi.org/10.1145/3470496.3527400},
  doi = {10.1145/3470496.3527400},
  booktitle = {Proceedings of the 49th Annual International Symposium on Computer Architecture},
  pages = {817–830},
  numpages = {14},
  series = {ISCA '22}
}
```

## Overview

In this repository you can find:

An **outline of the RTL files** can be found at *rtl/Flist.dcp*
DCP stands for 'decoupling from processor', as it is an RTL block that can be interacted with through the MAPLE API.

The **MAPLE API** is located at *api/dcp_maple.h*, whereas the *api/dcp_shared_memory.h* implements the API of the decoupling functions using shared memory, as a way to compare the improvements of the specialized MAPLE hardware to mitigate memory latency.

The *tests* folder contains the benchmarks in subfolders and four other programs to test MAPLE features. Note that MAPLE can also perform basic DMA as shown in dma.c

## Installation

    git clone git@github.com:PrincetonUniversity/openpiton.git;
    cd openpiton;
    git checkout openpiton-maple;
    git clone git@github.com:PrincetonUniversity/maple.git;
    source piton/ariane_setup.sh;
    source piton/ariane_build_tools.sh;

### Building RTL
We are now going to build a basic prototype of 4 tiles (2 Arianes and 2 MAPLE tile in between)
Currently the frequency of MAPLE tiles is one every two tiles. This can be configured.

    cd build;
    sims -sys=manycore -ariane -decoupling -vcs_build -x_tiles=3 -y_tiles=1 -config_rtl=MINIMAL_MONITORING;

### Running basic test
    cd $PITON_ROOT/maple;

Runs test #0. (Four basic tests are provided within this run_test.sh script)

    ./run_test.sh 0;

### Troubleshooting
If the build process fails due to a python problem make sure that your python command is defined and pointing to python2

    which python

If it's not defined or pointing to python3, then change the usage of python for python2 in the following files by running the following commands from the openpiton folder (not the build folder)
    
    sed -i 's/python/python2/' piton/design/chip/tile/ariane/bootrom/Makefile;
    sed -i 's/python/python2/' piton/design/chip/tile/ariane/openpiton/bootrom/linux/Makefile;



## Running programs

### Running feature tests

To run the basic tests we can use the script **run_test.sh <test_id>**
    
where ***test_id*** is the index of the 4 test types 

tests=("dma" "dcp_uni" "contiguous_allocation" "custom_acc")

For example to run the dma test we do

    ./run_test.sh 0

### Running benchmarks

To run the benchmarks we can use the script **run_bench.sh <type> <name> <tiles> <access_threads> <execute_threads> <dataset_size><mode>**

where:

 ***type*** is either *dcpn* (dcp normal), *dcpl* (dcp lima prefetching) or *doall* (for traditional homogeneous parallelism)

 ***name*** is either *spmv* (sparse matrix vector mul), *spmm* (sparse matrix matrix mul), *bfs* (breadth-first-seach) or *ewsd* (element-wise sparse dense multiplication, aka, SDHP)

 ***tiles*** is the number of tiles, counting MAPLE and Ariane core tiles.

 ***access_threads*** is the number of cores that are going to be behaving as supplier in decoupling, or the total number of core threads in prefetching or doall

***execute_threads*** (only relevant for decoupling, aka dcpn) is the number of cores that are going to be behaving as consumer in decoupling.

***size_dataset*** (1:Tiny, 2:Small 3:Big)

***mode*** (1:LIMA Loading from DRAM; 2:LIMA Loading from LLC; 3:LIMA Prefetching into LLC; 4:Software Prefetching )

For example, the next command runs *prefetching with LIMA*, for SPMV, using 3 tiles (2 arianes and 1 MAPLE tile), 2 processing threads, execute threads not relevant, and size small, and LIMA loading from DRAM.

    ./run_bench.sh "dcpl" spmv 3 2 0 2 1

The following example does the same for *doall*

    ./run_bench.sh "doall" spmv 3 2 0 2 1

And the following example does the same for *decoupling*

    ./run_bench.sh "dcpn" spmv 3 1 1 2 1

The print output and the simulation trace are places into the *build/res* folder


## Videos of FPGA demos
Decoupling with four tiles
- https://youtu.be/elkQcMFSvoo

Decoupling and prefetching on top of Linux
- https://youtu.be/YRbsjqzlTOM
