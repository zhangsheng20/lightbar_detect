project(lightbar_detect)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_COMPILER g++)
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-std=c++11)
message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(CMAKE_BUILD_TYPE "DEBUG")
find_package(OpenCV REQUIRED)



aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(
   ${PROJECT_NAME}
    ${OpenCV_LIBS}
    -lpthread
    )
