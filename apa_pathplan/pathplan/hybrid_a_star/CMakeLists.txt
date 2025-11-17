cmake_minimum_required(VERSION 3.10)
project(hybrid_module)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 设置源码目录
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB C_SRCS ${SRC_DIR}/*.c)
file(GLOB CPP_SRCS ${SRC_DIR}/*.cpp)

# 强制 .c 文件用 C++ 编译器编译
foreach(c_file ${C_SRCS})
    set_source_files_properties(${c_file} PROPERTIES LANGUAGE CXX)
endforeach()

# 头文件路径
include(${PROJECT_BASEDIR}/script/common.cmake)

# 添加当前目录作为头文件路径（等价于 -I ./）
include_directories(${SRC_DIR})
include_directories(${PROJECT_INCLUDE_PATHS})

if(BUILDTYPE STREQUAL "arm")
elseif(BUILDTYPE STREQUAL "x86")
    #  OpenCV
    set(OpenCV_DIR
    ${PROJECT_BASEDIR}/extlib/opencv/x86/lib/cmake/opencv4
    )
elseif(BUILDTYPE STREQUAL "ndk")
    # opencv配置
    set(OpenCV_DIR "${PROJECT_BASEDIR}/extlib/opencv/ndk/sdk/native/jni")

endif()

# OpenCV 配置
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 创建共享库
add_library(hybrid SHARED ${C_SRCS} ${CPP_SRCS})

# 编译选项（等价于 CFLAGS 和 CPPFLAGS）
target_compile_options(hybrid PRIVATE
    -Wno-unused-function
    -Wno-subobject-linkage
)

# 链接选项
target_link_options(hybrid PRIVATE -fPIC)
target_link_libraries(hybrid PRIVATE ${OpenCV_LIBS})

# 输出路径
set_target_properties(hybrid PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 装载路径
install(TARGETS hybrid
        ARCHIVE DESTINATION apa_include/lib/${BUILDTYPE}
        LIBRARY DESTINATION apa_include/lib/${BUILDTYPE}
        RUNTIME DESTINATION apa_include/lib/${BUILDTYPE})
