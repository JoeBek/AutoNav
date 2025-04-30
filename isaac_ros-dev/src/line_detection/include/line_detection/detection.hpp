
#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "cuda.cuh"
#include <utility>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

[[maybe_unused]] static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("line_logging"), "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
                         };


#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

namespace lines {
std::pair<int2*, int*> detect_line_pixels(const cv::Mat&);
}
std::pair<Npp32f *, Npp64f *> __get_integral_image(const cv::Mat &gray_img);



#endif