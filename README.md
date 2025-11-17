1、目录结构说明：
Apa_PK                 泊车应用代码
    extlib             外部库, 不需要编译
    apa_include        对外提供的头文件（所有人可见）
    apa_common         功能实现源码（APA基础功能模块, 受控，设置访问权限）
    apa_control        功能实现源码（APA控制模块, 受控，设置访问权限）
    apa_pathplan       功能实现源码（APA规划模块, 受控，设置访问权限）
    apa_sensorfusion   功能实现源码（APA障碍物处理模块, 受控，设置访问权限）
    apa_slotdetect     功能实现源码（APA车位管理模块, 受控，设置访问权限）
    apa_statemanage    功能实现源码（APA状态管理及告警模块, 受控，设置访问权限）
    apa_device      功能实现源码（CAN/ETH/SPI底层通信模块, 受控，设置访问权限）
    apa_location    功能实现源码（adas/apa定位模型, 受控，设置访问权限）
    apa_rte         功能实现源码（adas/apa RTE接口模块, 受控，设置访问权限）

build.sh               增量编译脚本
include.mk             头文件路径配置文件
common.mk              编译链接选项全局配置文件，如果有修改编译选项需求，优先修改本模块内的makefile

2、编译说明
前提条件: 准备好Ubutn18.06环境，仿真环境要求安装gnu c++

A、在本地Ubunt18环境上安装；
交叉编译器路径： SOC/OS/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz

将交叉编译器压缩包下载到本地Ubunt18环境上，解压；

B、更新代码到本地Ubuntu环境下
src下只会下载和工作范围相关的目录；

C、修改编译脚本build.sh/rebuild.sh common.mk中设置的交叉编译路径；
    例子：export PATH=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin
    将opt修改为A中解压后的本地存放路径

D、将编译脚本目录从include/script拷贝到工程根目录

E、代码编译
E.1 进入编译脚本所在路径，将脚本文件属性设置为可执行(对应的命令：chmod +x *.sh  )

E.2 执行编译脚本：
               ./build.sh arm        j3版本编译
               ./build.sh x86        x86版本编译
               ./build.sh a1000      a1000版本编译
               ./build.sh ndk        andriod版本编译

E.3 编译结果确认
编译后生成的库文件位于编译脚本同级目录下output目录；(output/arm x86 bst)
只会更新下载的工作相关模块二进制库文件，其他功能使用配置库上下载的库文件；

3、功能验证
将output/arm目录下所有库文件下载到j3开发板上即可验证；

4、代码及二进制库文件上传
源码修改更新到svn后，需要将更新后的库文件拷贝到Apa_PK/lib/arm  Apa_PK/lib/x86 Apa_PK/lib/bst ，并上传