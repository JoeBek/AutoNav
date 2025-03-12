/**
 * Cuda kernels for fast Line detection processing
 * 
 */

#include <cuda_runtime.h>
#include <npp.h>

// the window has to be odd
#define HALF_WINDOW_SIZE 7 // this produces a 15 x 15 window
#define WINDOW_SIZE  2 * HALF_WINDOW_SIZE + 1
#define WINDOW_SIZE_SQ  WINDOW_SIZE * WINDOW_SIZE
#define SIGMA_THRESHOLD  15
#define MEW_THRESHOLD 200


// dim3 block (16,16,1)
// dim3 grid(COLS, ROWS)
__global__ void cerias_kernel (
        float *gray_img,
        Npp32f *integral,
        Npp64f *integral_sq,
        uint8_t *brightness_mask,
        int2 *output,
        int *counter,
        int width, int height
    ) 
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;


    // coordinates not in brightness mask, pixel not in line
    if (x >= width || y >= height || !brightness_mask[y * width + x])
        return;

    // assemble the window
    int x1 = max(0, x - HALF_WINDOW_SIZE);
    int y1 = max(0, y - HALF_WINDOW_SIZE);
    int x2 = min(gridDim.x - 1, x + HALF_WINDOW_SIZE);
    int y2 = min(gridDim.y - 1, y + HALF_WINDOW_SIZE);

    // get intensity std. div of pixels in the window

    // get integral image areas from window

    // y * width + x unwraps rows into 1d and adds remaining cols
    float sum_intensity = static_cast<float>(integral[y2*width+ x2] - integral[y1*width + x2]
                        - integral[y2*width + x1] + integral[y1*width + x1]);

    float sum_intensity_sq = static_cast<float>(integral_sq[y2*width+ x2] - integral_sq[y1*width + x2]
                        - integral_sq[y2*width + x1] + integral_sq[y1*width + x1]);



    float num_pixels = static_cast<float>(WINDOW_SIZE_SQ);

    float mew = sum_intensity / num_pixels;

    float sigma = sqrt( (sum_intensity_sq - (sum_intensity * sum_intensity)/ num_pixels) / (num_pixels - 1.0f));

    if (sigma < SIGMA_THRESHOLD && mew > MEW_THRESHOLD) {

        int index = atomicAdd(counter, 1);
        output[index] = make_int2(x, y);

    }

}


