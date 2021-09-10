cmake_minimum_required(VERSION 3.5.1)
project(sensor_img_processing)

set(CMAKE_CXX_STANDARD 14)
# It prevents the decay to C++98 when the compiler does not support C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# It disables the use of compiler-specific extensions
# e.g. -std=c++14 rather than -std=gnu++14
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# If no need to debug, do not uncomment this
#SET(CMAKE_BUILD_TYPE "DEBUG")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    cv_bridge
    pcl_ros
)

add_message_files(
    FILES
    Object.msg
    ObjectArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

# Try to find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    # If the package has been found, several variables will
    # be set, you can find the full list with descriptions
    # in the OpenCVConfig.cmake file.
    # Print some message showing some of them
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}" \n)
else ()
    message(FATAL_ERROR "Could not locate OpenCV" \n)
endif()

# set(Torch_DIR libtorch/share/cmake/Torch)
set(Torch_DIR /home/libtorch/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "include path: ${TORCH_INCLUDE_DIRS}" \n)
else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()

# link_directories(${TORCH_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime
    DEPENDS
    LIBRARIES ${PROJECT_NAME}
)

file(GLOB SOURCE_FILES src/sensor_processing_lib/*.cpp)

add_executable(sensor_setup src/sensor_setup_node.cpp  ${SOURCE_FILES})
add_dependencies(sensor_setup ${catkin_EXPORTED_TARGETS} sensor_img_processing_generate_messages_cpp)
target_link_libraries (
    sensor_setup
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
)

## install_isolated/share/sensor_img_processing/  contains launch, config, weight folders
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY weight DESTINATION share/${PROJECT_NAME}/)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
    sensor_setup
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)