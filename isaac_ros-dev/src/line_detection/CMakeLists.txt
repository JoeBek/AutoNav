cmake_minimum_required(VERSION 3.8)
project(line_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# set arch and nvcc flags
set(CMAKE_CUDA_ARCHITECTURES 60)
set(CMAKE_CUDA_FLAGS "${CUDA_NVCC_FLAGS} --generate-code=arch=compute_60,code=sm_60")


# create executable target
file(GLOB SRC_FILES src/*.cpp src/*.cu) # glob haha
cuda_add_executable(line_detector ${SRC_FILES})


# set include paths
ament_target_dependencies(line_detector rclcpp autonav_interfaces cv_bridge OpenCV tf2_ros tf2_eigen Eigen3 image_geometry tf2_geometry_msgs)

# target local library
target_include_directories(line_detector PRIVATE
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${image_geometry_INCLUDE_DIRS} 
 ${tf2_geometry_msgs_INCLUDE_DIRS})

target_include_directories(line_detector PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)



# link targeted libraries

target_link_libraries(line_detector 
                      ${CUDA_LIBRARIES}
                      nppist
                        # For image processing
)




# install
install(TARGETS 
    line_detector
    DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
