# MAPLE (Memory Access Parallel Load Engine)

This is the repository for the RTL and API of the research paper "Tiny but Mighty: Designing and Realizing Scalable Latency Tolerance for Manycore SoCs", to appear at the 49th International Symposium on Computer Architecture 

An outline of the RTL files can be found at rtl/Flist.dcp
DCP stands for 'decoupling from processor', as it is an RTL block that can be interacted with through the MAPLE API.

The MAPLE API is located at api/dcp_maple.h, whereas the api/dcp_shared_memory.h implements the API of the decoupling functions using shared memory, as a way to compare the improvements of the specialized MAPLE hardware to mitigate memory latency.

Soon we will also upload the tests and datasets used for the paper, together with the instructions to connect MAPLE with the openpiton manycore framework.
