
#ifndef CUDA_CUH
#define CUDA_CUH

#include <cuda_runtime.h>
#include <npp.h>
#include <cstdint>
#include <cstdio>



__global__ void __cerias_kernel(float * gray_img,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * mask,
                             int2 * output,
                             int * counter,
                             int width, int height);


extern "C" void cerias_kernel(float * gray_img,
                             Npp32f * integral,
                             Npp64f * integral_sq,
                             uint8_t * mask,
                             int2 * output,
                             int * counter,
                             int width, int height);

#endif