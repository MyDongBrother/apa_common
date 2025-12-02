###############################################################################
# @brief 设置基础编译选项（C++14，位置无关代码）
###############################################################################
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###############################################################################
# @brief 通用编译 Flags，根据编译器类型做额外处理
###############################################################################
set(COMMON_C_FLAGS "-fPIC -Wno-unused-function -Wno-unknown-pragmas -g -Wall")
set(COMMON_CXX_FLAGS "-fPIC -Wno-unused-function -Wno-unknown-pragmas -g -Wall")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # GCC 特有警告关闭
    set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-subobject-linkage")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Clang 特有警告关闭
    set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-but-set-variable")
endif()

set(CMAKE_C_FLAGS "${COMMON_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_CXX_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${COMMON_LINK_FLA}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###############################################################################
# @brief 加载 APA runtime 通用头文件路径
###############################################################################
set(PROJECT_INCLUDE_PATHS "")
list(APPEND PROJECT_INCLUDE_PATHS
    ${PROJECT_BASEDIR}/apa_runtime/include
    ${PROJECT_BASEDIR}/apa_runtime/include/apa_common
    ${PROJECT_BASEDIR}/apa_runtime/include/apa_services
    ${PROJECT_BASEDIR}/apa_runtime/include/apa_msgs
)

###############################################################################
# @brief Debug 模式添加调试宏
###############################################################################
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_VERSION)
endif()

###############################################################################
# @brief 多平台编译配置（arm / x86 / ndk）
###############################################################################
if(BUILDTYPE STREQUAL "arm")

    ###########################################################################
    # ARM 平台（嵌入式）构建
    ###########################################################################
    add_definitions(-DAPA_PLATFORM_ARM)

    list(APPEND PROJECT_INCLUDE_PATHS
        /opt/aarch64-linux-gnu/include
        /opt/aarch64-linux-gnu/aarch64-linux-gnu/include
    )

    set(CMAKE_AR aarch64-linux-gnu-ar)

elseif(BUILDTYPE STREQUAL "x86")

    ###########################################################################
    # Linux（X86）平台构建
    ###########################################################################
    add_definitions(-DAPA_PLATFORM_X86)

    # x86 平台专用 include
    list(APPEND PROJECT_INCLUDE_PATHS
        ${PROJECT_BASEDIR}/apa_runtime/fastdds/x86/include
        ${PROJECT_BASEDIR}/apa_runtime/fastcdr/x86/include
    )

    # 链接 runtime（services, msgs, fastdds, fastcdr）
    set(APA_DEVS_RUNTIME
        ${PROJECT_BASEDIR}/apa_runtime/apa_common/x86/libcommons.so
        ${PROJECT_BASEDIR}/apa_runtime/apa_services/x86/libservices.so
        ${PROJECT_BASEDIR}/apa_runtime/apa_msgs/x86/libbmi_msgs.so
        ${PROJECT_BASEDIR}/apa_runtime/fastdds/x86/lib/libfastrtps.so.2.6.6
        ${PROJECT_BASEDIR}/apa_runtime/fastcdr/x86/lib/libfastcdr.so.1.0.24
    )

    # Python3（一般用于 matplotlib 调试）
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    include_directories(${Python3_INCLUDE_DIRS})
    set(APA_DEVS_PYPLOT ${Python3_LIBRARIES})

    # OpenCV（x86 可直接 find_package）
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(APA_DEVS_OPENCV ${OpenCV_LIBS})

elseif(BUILDTYPE STREQUAL "ndk")

    ###########################################################################
    # Android NDK（arm64-v8a）构建
    ###########################################################################
    add_definitions(
        -DAPA_PLATFORM_NDK       # 平台宏
        -DDNCNN_VULKAN=ON        # 启用 NDK 下的 Vulkan 加速
        -Wno-deprecated
    )

    # NDK + runtime 头文件路径
    list(APPEND PROJECT_INCLUDE_PATHS
        /opt/android-ndk-r26b/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include
        ${PROJECT_BASEDIR}/apa_runtime/fastdds/ndk/include
        ${PROJECT_BASEDIR}/apa_runtime/fastcdr/ndk/include
        ${PROJECT_BASEDIR}/apa_runtime/opencv/ndk/include
    )

    set(CMAKE_AR llvm-ar)
    set(CMAKE_RANLIB llvm-ranlib)

    # OpenCV NDK：手工指定 so 文件
    file(GLOB APA_DEVS_OPENCV
        ${PROJECT_BASEDIR}/apa_runtime/opencv/ndk/lib/*.so
    )

    #平台依赖库
    file(GLOB PLTFORM_EXT_LIBS "${PROJECT_BASEDIR}/apa_runtime/extlib/ndk/*.so")

    # 链接 runtime
    set(APA_DEVS_RUNTIME
        ${PROJECT_BASEDIR}/apa_runtime/apa_common/ndk/libcommons.so
        ${PROJECT_BASEDIR}/apa_runtime/apa_services/ndk/libservices.so
        ${PROJECT_BASEDIR}/apa_runtime/apa_msgs/ndk/libbmi_msgs.so
        ${PROJECT_BASEDIR}/apa_runtime/fastdds/ndk/lib/libfastrtps.so
        ${PROJECT_BASEDIR}/apa_runtime/fastcdr/ndk/lib/libfastcdr.so
        ${PLTFORM_EXT_LIBS}
    )

    # NDK 不支持 Python matplotlib
    set(APA_DEVS_PYPLOT "")
endif()

