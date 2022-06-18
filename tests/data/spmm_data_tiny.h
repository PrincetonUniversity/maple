
#define NNZ 11
#define BNNZ 15
uint32_t A_shape[2] = {5, 5};
uint32_t A_indptr[6] = {0, 3, 6, 7, 9, 11};
uint32_t A_indices[11] = {1, 2, 4, 0, 2, 3, 0, 3, 4, 0, 1};
uint32_t A_data[11] = {4, 5, 1, 2, 2, 2, 3, 2, 1, 1, 1};

uint32_t B_shape[2] = {5, 6};
uint32_t B_indptr[7] = {0, 4, 6, 7, 11, 12, 15};
uint32_t B_indices[15] = {0, 1, 3, 4, 0, 3, 3, 0, 1, 2, 3, 0, 0, 2, 3};
uint32_t B_data[15] = {4, 2, 7, 9, 9, 7, 9, 2, 1, 3, 2, 2, 4, 8, 1};

uint32_t C_data_res[30] = {13, 25, 24, 18, 11, 36, 45, 14, 16, 18, 9, 11, 8, 12, 6, 4, 8, 10, 2, 24, 16, 20, 2, 5};

uint32_t bias[6] = {0, 0, 0, 0, 0, 0};

uint32_t C_indptr[7];
uint32_t C_indices[30];
uint32_t C_data[30];
uint32_t spa[30];
uint32_t tmp_C_indices[30];
uint32_t tmp_C_data[30];
