static uint32_t G_shape[2] = {5, 5};
#define E 11 
static uint32_t G_indptr[6] = {0, 3, 5, 7, 9, 11};
static uint32_t G_indices[11] = {1, 2, 4, 0, 4, 0, 1, 2, 3, 0, 3};
//static uint32_t G_indices2[11] = {1, 2, 4, 5, 9, 10, 11, 17, 18, 20, 23};
static uint32_t G_data[11] = {2, 3, 1, 4, 1, 5,  2,   2,  2,  1,  1};
uint32_t result_data2[11]  = {2, 6, 4, 20, 9, 50, 22, 34, 36, 20, 23};

static uint32_t M[25] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};

uint32_t result_shape[2] = {5, 5};
uint32_t result_indptr[6] = {0, 0, 0, 0, 0, 0};
uint32_t result_indices[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t result_data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

