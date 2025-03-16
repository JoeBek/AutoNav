cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(line_detector LANGUAGES CXX CUDA)

# debugging
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CUDA_FLAGS_DEBUG "-g -G -O0")

# find packages
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CUDA_NVCC_FLAGS} "-g -G") # enable cuda-gdb
endif()


# set arch and nvcc flags
set(CMAKE_CUDA_ARCHITECTURES 60)
set(CMAKE_CUDA_FLAGS "${CUDA_NVCC_FLAGS} --generate-code=arch=compute_60,code=sm_60")


#set_source_files_properties(cuda.cu PROPERTIES LANGUAGE CUDA)
#set_source_files_properties(detection.cpp PROPERTIES LANGUAGE CUDA)




# Add executable
cuda_add_executable(line_detector main.cpp detection.cpp cuda.cu)


# Include directories
target_include_directories(line_detector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(line_detector PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link OpenCV libraries
target_link_libraries(line_detector 
                      ${OpenCV_LIBS}
                      ${CUDA_LIBRARIES}
                      nppist
                        # For image processing
                    
)

target_compile_features(line_detector PRIVATE cxx_std_11)
