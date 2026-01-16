cmake_minimum_required(VERSION 3.29.0)
project(COMPREHENSIVE_TRYING)
set(CMAKE_CXX_STANDARD 11) # 使用C++11标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 强制使用所选标准
set(CMAKE_CXX_EXTENSIONS ON) # 启用编译器特定的扩展

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin) # 可执行文件输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin) # 动态库文件输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib) # 静态库文件输出目录

# 设置路径变量
set(IMU_PATH ${CMAKE_SOURCE_DIR}/src/IMU)
set(COMMON_PATH ${CMAKE_SOURCE_DIR}/src/COMMON)

# 三方库
set(EIGEN3_INCLUDE_DIR "D:/my_it/Eigeon/eigen-3.4.0")
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加子目录
add_subdirectory(src)
add_subdirectory(external)
