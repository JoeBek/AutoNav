
#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "cuda.cuh"
#include <utility>
#include <opencv2/opencv.hpp>

static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
                         };


#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

std::pair<int2*, int*> detect_line_pixels(const cv::Mat&);
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img);



#endif