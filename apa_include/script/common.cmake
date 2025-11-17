# 设置通用编译参数
set(COMMON_C_FLAGS "-fPIC -Wno-unused-function -Wno-unknown-pragmas -g -Wall")
set(COMMON_CXX_FLAGS "-fPIC -Wno-unused-function -Wno-unknown-pragmas -g -std=c++17 -Wno-subobject-linkage -Wall")

# 设置编译器和工具链变量
set(CMAKE_C_FLAGS "${COMMON_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_CXX_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${COMMON_LINK_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 初始化 include 路径变量
set(PROJECT_INCLUDE_PATHS "")
set(SDKTARGETSYSROOT "")

# BASEDIR 是项目根目录
set(BASEDIR ${CMAKE_SOURCE_DIR})

# 添加 apa_include 路径
list(APPEND PROJECT_INCLUDE_PATHS
    ${PROJECT_BASEDIR}/apa_include/include/apa_common
    ${PROJECT_BASEDIR}/apa_include/include/apa_control
    ${PROJECT_BASEDIR}/apa_include/include/apa_pathplan
    ${PROJECT_BASEDIR}/apa_include/include/apa_odometry
    ${PROJECT_BASEDIR}/apa_include/include/apa_sensorfusion
    ${PROJECT_BASEDIR}/apa_include/include/apa_slotdetect
    ${PROJECT_BASEDIR}/apa_include/include/apa_statemanage
    ${PROJECT_BASEDIR}/apa_include/include/apa_device
    ${PROJECT_BASEDIR}/apa_include/include/apa_location
    ${PROJECT_BASEDIR}/apa_include/include/apa_rte
)

# 添加 extlib 下的头文件路径
list(APPEND PROJECT_INCLUDE_PATHS
    ${PROJECT_BASEDIR}/extlib/include/hobotlog/include
    ${PROJECT_BASEDIR}/extlib/include/zmq/inc
    ${PROJECT_BASEDIR}/extlib/include/apa_msgsv3
    ${PROJECT_BASEDIR}/extlib/include/raincom_com/include
    ${PROJECT_BASEDIR}/extlib/apa_msgs/include
)

# 多平台配置
if(BUILDTYPE STREQUAL "arm")
    list(APPEND PROJECT_INCLUDE_PATHS
        /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/include
        /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/aarch64-linux-gnu/include
        ${PROJECT_BASEDIR}/extlib/jsoncpp
        ${PROJECT_BASEDIR}/extlib/jsoncpp/inc
    )
    set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
    set(CMAKE_AR aarch64-linux-gnu-ar)

elseif(BUILDTYPE STREQUAL "x86")
    list(APPEND PROJECT_INCLUDE_PATHS
        ${PROJECT_BASEDIR}/extlib/jsoncpp
        ${PROJECT_BASEDIR}/extlib/jsoncpp/inc
        ${PROJECT_BASEDIR}/extlib/include/apa_msgs
        ${PROJECT_BASEDIR}/extlib/opencv/x86/include/opencv4
        ${PROJECT_BASEDIR}/extlib/apa_msgs/lib/x86
    )
    set(CMAKE_C_COMPILER gcc)
    set(CMAKE_CXX_COMPILER g++)
    add_definitions(-DSIMULATE)

    # 依赖库路径
    link_directories(${PROJECT_BASEDIR}/extlib/apa_msgs/lib/x86)

elseif(BUILDTYPE STREQUAL "ndk")
    list(APPEND PROJECT_INCLUDE_PATHS
        $ENV{NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/include
        $ENV{NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include
        $ENV{NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/c++/v1
        $ENV{NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/android
        ${PROJECT_BASEDIR}/extlib/jsoncpp
        ${PROJECT_BASEDIR}/extlib/jsoncpp/inc
        ${PROJECT_BASEDIR}/extlib/include/apa_msgs
        ${PROJECT_BASEDIR}/extlib/opencv/ndk/sdk/native/jni/include
    )
    set(CMAKE_C_COMPILER aarch64-linux-android31-clang)
    set(CMAKE_CXX_COMPILER aarch64-linux-android31-clang++)
    set(CMAKE_AR llvm-ar)
    set(CMAKE_RANLIB llvm-ranlib)
    set(ANDROID_NDK_ABI_NAME "arm64-v8a")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{NDK_HOME}/build/cmake/android.toolchain.cmake")
    add_definitions(
        -DANDROID_ABI="arm64-v8a"
        -DANDROID_PLATFORM=android-26
        -DDNCNN_VULKAN=ON
        -Wno-deprecated
    )

    # 依赖库路径
    link_directories(${PROJECT_BASEDIR}/extlib/apa_msgs/ndk/x86)
endif()


# 根据构建类型设置 DEBUG_VERSION
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_VERSION)
endif()
