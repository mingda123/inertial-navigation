# 添加头文件库路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加子目录
add_subdirectory(IMU)
add_subdirectory(COMMON)

# 链接生成可执行文件
add_executable(PureImu PureImu.cpp)

target_include_directories(PureImu
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/COMMON
)

# 链接静态/动态库
target_link_libraries(PureImu
    PUBLIC
    imu
    simpleini
)