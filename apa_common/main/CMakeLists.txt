cmake_minimum_required(VERSION 3.10)
project(apa_pk)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB C_SRCS ${SRC_DIR}/*.c)
file(GLOB CPP_SRCS ${SRC_DIR}/*.cpp)

# .c 文件用 C++ 编译器编译
foreach(c_file ${C_SRCS})
    set_source_files_properties(${c_file} PROPERTIES LANGUAGE CXX)
endforeach()

# 包含头文件路径
include(${PROJECT_BASEDIR}/script/common.cmake)
include_directories(${SRC_DIR} ${PROJECT_INCLUDE_PATHS})

# 生成可执行文件，源文件全用上
add_executable(apa_pk_${BUILDTYPE} ${C_SRCS} ${CPP_SRCS})

# 编译选项
target_compile_options(apa_pk_${BUILDTYPE} PRIVATE
    -Wno-unused-function
    -Wno-subobject-linkage
)

# 链接选项
target_link_options(apa_pk_${BUILDTYPE} PRIVATE -fPIC)

# 输出路径
set_target_properties(apa_pk_${BUILDTYPE} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

#库文件路径
set(LIBDIR ${PROJECT_BASEDIR}/apa_include/lib/${BUILDTYPE})
set(EXTLIBDIR ${PROJECT_BASEDIR}/extlib/lib/${BUILDTYPE})

# 三方库链接路径（先清空）
set(EXTLIBS)
if(${BUILDTYPE} STREQUAL "x86")
    list(APPEND EXTLIBS
        ${EXTLIBDIR}/libhobotlog.so
        ${EXTLIBDIR}/libapa_msgs.so
        ${EXTLIBDIR}/libjson.so
        ${PROJECT_BASEDIR}/extlib/apa_msgs/lib/x86/libbmi_msgs.so
        ${PROJECT_BASEDIR}/extlib/fastdds/x86/lib/libfastcdr.so
        ${PROJECT_BASEDIR}/extlib/fastdds/x86/lib/libfastrtps.so
        -lpthread
    )
elseif(${BUILDTYPE} STREQUAL "ndk")
    list(APPEND EXTLIBS
        ${EXTLIBDIR}/libapa_msgs.so
        ${EXTLIBDIR}/libfastcdr.so
        ${EXTLIBDIR}/libfastrtps.so
        ${EXTLIBDIR}/libglog.so
        ${EXTLIBDIR}/libjson.so
        # ${EXTLIBDIR}/libprotobuf-lite.so
        ${EXTLIBDIR}/libprotobuf.so
        # ${EXTLIBDIR}/libprotoc.so
        ${EXTLIBDIR}/libraincom_com.so
        # ${EXTLIBDIR}/libraincom_tools.so
        # ${EXTLIBDIR}/libtinyxml2.so
        # ${EXTLIBDIR}/libyaml-cpp.so
        ${PROJECT_BASEDIR}/extlib/apa_msgs/lib/ndk/libbmi_msgs.so
    )
endif()

target_link_libraries(apa_pk_${BUILDTYPE}
    ${LIBDIR}/libpathexecute.so
    ${LIBDIR}/libsensort2s.so
    ${LIBDIR}/libdistctrl.so
    ${LIBDIR}/liblocation.so
    ${LIBDIR}/libodometry.so
    ${LIBDIR}/libobjavoid.so
    ${LIBDIR}/libpathplan.so
    ${LIBDIR}/libsensorpreproc.so
    ${LIBDIR}/libslotdetect.so
    ${LIBDIR}/libsensorfusion.so
    ${LIBDIR}/libcommon.so
    ${LIBDIR}/libeth.so
    ${LIBDIR}/libpk_uart.so
    ${LIBDIR}/libapa_dds.so
    ${LIBDIR}/librte.so
    ${LIBDIR}/libhybrid.so
    ${LIBDIR}/libcanrecv.so
    ${LIBDIR}/libstatemanage.so
    ${LIBDIR}/libuicomm.so
    ${LIBDIR}/libavmcomm.so
    ${EXTLIBS}
)


install(TARGETS apa_pk_${BUILDTYPE}
    RUNTIME DESTINATION apa_include/bin/${BUILDTYPE}
)
