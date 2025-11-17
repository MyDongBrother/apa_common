cmake_minimum_required(VERSION 3.10)
project(odometry_module)

# C++14 + PIC
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 引入通用配置
include(${PROJECT_BASEDIR}/script/common.cmake)

# 根目录头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_INCLUDE_PATHS})

# 递归查找所有 .c 和 .cpp
file(GLOB_RECURSE C_SRCS  *.c)
file(GLOB_RECURSE CPP_SRCS *.cpp)

# 强制所有 .c 用 C++ 编译
foreach(cfile ${C_SRCS})
    set_source_files_properties(${cfile} PROPERTIES LANGUAGE CXX)
endforeach()

# 创建共享库
add_library(odometry SHARED ${C_SRCS} ${CPP_SRCS})

# 编译选项
target_compile_options(odometry PRIVATE
    -Wno-unused-function
    -Wno-subobject-linkage
)

# 链接选项
target_link_options(odometry PRIVATE -fPIC)

# 输出 lib 路径
set_target_properties(odometry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 安装路径
install(TARGETS odometry
        ARCHIVE DESTINATION apa_include/lib/${BUILDTYPE}
        LIBRARY DESTINATION apa_include/lib/${BUILDTYPE}
        RUNTIME DESTINATION apa_include/lib/${BUILDTYPE})
