# 仓库描述

本仓库是 APA（自动泊车辅助）系统的基础组件集合，提供泊车核心功能所依赖的通用能力，包括线程调度、数据通信、时间同步、日志管理以及 AVM 数据源处理等模块。这些模块通常作为更高层 APA 算法、感知、路径规划与控制的基础支撑库，确保系统具备高可靠性、可移植性与一致的运行时行为。

# 项目概览

本仓库包含 APA（自动泊车辅助）视觉服务相关的多个功能模块，目录结构如下：

* **avm_source**：AVM（全景影像）源数据处理模块。
* **build.sh**：构建脚本，用于编译整个工程。
* **CMakeLists.txt**：项目主 CMake 构建文件。
* **data_bus**：数据总线模块，负责模块间消息传递。
* **log_manager**：日志管理模块，用于系统日志收集与输出。
* **python_plot**：用于可视化调试的 Python 绘图脚本。
* **thread_manager**：线程与任务调度模块。
* **time_sync**：时间同步模块，用于不同传感器/线程间时序对齐。

---

## 子仓库（Git Subtree）

本仓库支持通过 `git subtree` 的方式集成到上层项目中。

### 添加方式

```bash
git remote add apa_services https://github.com/MyDongBrother/apa_services.git

git subtree add --prefix=apa_services apa_services main --squash
```

如需更新：

```bash
git subtree pull --prefix=apa_services apa_services main --squash
```

## 构建方式

在项目根目录执行：

```bash
./build.sh
```

或使用 CMake 方式：

```bash
mkdir build && cd build
cmake ..
make -j
```

---

## 运行

构建产物可在 `build/` 目录中找到，具体可执行文件视功能模块而定。

---

## 开发规范

* 默认使用 **C++14**。
* 文件命名、类型命名、变量命名等遵循项目约定。
* 使用 POSIX 接口进行系统调用。

---

## 联系

如需进一步说明，可继续扩展 README 内容以匹配团队需求。
