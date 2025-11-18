cmake_minimum_required(VERSION 3.10)
project(commons_module)
set(TARGET_NAME commons)

# 设置源码目录
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE C_SRCS   ${SRC_DIR}/*.c)
file(GLOB_RECURSE CPP_SRCS ${SRC_DIR}/*.cpp ${SRC_DIR}/*.cc ${SRC_DIR}/*.cxx ${SRC_DIR}/*.C)

# 强制 .c 文件用 C++ 编译器编译
foreach(c_file ${C_SRCS})
    set_source_files_properties(${c_file} PROPERTIES LANGUAGE CXX)
endforeach()

# 通用cmake
include(${PROJECT_BASEDIR}/apa_runtime/common.cmake)

# 添加当前目录作为头文件路径（等价于 -I ./）
include_directories(${SRC_DIR})
include_directories(${PROJECT_INCLUDE_PATHS})

# 创建共享库
add_library(${TARGET_NAME} SHARED ${C_SRCS} ${CPP_SRCS})

# 指定编译选项，只针对三方库解决编译报错问题
target_compile_options(${TARGET_NAME} PRIVATE
)

# 链接选项
target_link_options(${TARGET_NAME} PRIVATE -fPIC)

# 输出路径
set_target_properties(${TARGET_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 装载路径
install(TARGETS ${TARGET_NAME}
        ARCHIVE DESTINATION apa_runtime/apa_common/${BUILDTYPE}
        LIBRARY DESTINATION apa_runtime/apa_common/${BUILDTYPE}
        RUNTIME DESTINATION apa_runtime/apa_common/${BUILDTYPE})
