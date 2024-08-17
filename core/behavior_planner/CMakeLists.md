这段`CMakeLists.txt`文件是用于配置和编译`behavior_planner`项目的CMake脚本。它定义了项目的构建设置、依赖项和生成的库。以下是详细解析：

### 1. **基本配置**
   - **`cmake_minimum_required(VERSION 2.8)`**：指定CMake的最低版本要求为2.8。
   - **`project(behavior_planner)`**：定义项目名称为`behavior_planner`。

### 2. **编译选项**
   - **`set(CMAKE_VERBOSE_MAKEFILE "true")`**：开启详细的Makefile输出，以便在编译时显示更多的信息，有助于调试。
   - **`set(CMAKE_BUILD_TYPE "Release")`**：设置构建类型为`Release`，意味着编译时会进行优化。
   - **`set(CMAKE_CXX_FLAGS "-std=c++11  -g")`**：设置C++编译标志，使用C++11标准，并开启调试信息生成（`-g`）。
   - **`set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")`**：为Release构建类型设置优化选项`-O3`，并启用所有警告`-Wall`。
   - **`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")`**：将前面的编译选项整合到一起。

### 3. **查找依赖项**
   - **`find_package(catkin REQUIRED COMPONENTS ...)`**：查找并导入`catkin`包，它是ROS的构建系统。这里指定了项目依赖的ROS组件，包括`roscpp`（C++客户端库）、`visualization_msgs`和`sensor_msgs`（消息类型）。
   - **`find_package(...)`**：查找并导入项目依赖的其他包，如`common`、`vehicle_model`、`semantic_map_manager`、`forward_simulator`和`route_planner`。

### 4. **设置模块路径**
   - **`set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)`**：指定CMake模块的路径为项目根目录下的`cmake`文件夹。

### 5. **Catkin包配置**
   - **`catkin_package(...)`**：定义了该包的导出配置，包括：
     - `INCLUDE_DIRS`：指定导出的头文件目录。
     - `LIBRARIES`：指定导出的库，包括`behavior_planner_lib`和`behavior_planner_ros`。

### 6. **包含目录**
   - **`include_directories(...)`**：指定编译时需要包含的目录。包括项目自己的`inc`目录以及所有依赖包的头文件目录。

### 7. **添加库**
   - **`add_library(behavior_planner_lib ...)`**：创建一个名为`behavior_planner_lib`的库，包含`behavior_planner.cc`和`map_adapter.cc`源文件。
   - **`target_link_libraries(behavior_planner_lib ...)`**：链接`behavior_planner_lib`库所需的其他库，包括`hkust_pl_common`、`hkust_pl_rp`等。

   - **`add_library(behavior_planner_ros ...)`**：创建另一个名为`behavior_planner_ros`的库，包含`behavior_server_ros.cc`源文件。
   - **`target_link_libraries(behavior_planner_ros ...)`**：链接`behavior_planner_ros`库所需的库，包括`behavior_planner_lib`和`hkust_smm_ros`。

### 8. **安装指令（已注释）**
   - **`install(...)`**：该部分指令用于安装头文件，但目前已被注释掉。这可能是因为这个包还在开发阶段，或者是暂时不需要安装头文件供其他包使用。

### 总结
- 这个`CMakeLists.txt`文件主要用于配置`behavior_planner`项目的编译和构建过程。它定义了项目的依赖项、编译选项、库的生成和链接等重要步骤。通过这个配置文件，开发者可以方便地管理项目的构建过程，确保所有依赖关系都得到正确处理。


# catkin_package
这段代码是`CMakeLists.txt`文件中的一部分，用于在ROS项目中定义`catkin_package`，即将某个ROS包作为一个`catkin`包导出。它指定了该包的头文件目录和库文件，供其他ROS包使用。

### 代码解释

```cmake
catkin_package(
    INCLUDE_DIRS inc ${route_planner_INCLUDE_DIRS}
    LIBRARIES behavior_planner_lib behavior_planner_ros
)
```

#### 1. **`INCLUDE_DIRS`**
   - **`INCLUDE_DIRS inc ${route_planner_INCLUDE_DIRS}`**：
     - **`inc`**：表示当前包中包含头文件的目录。这个路径将会被其他依赖该包的项目或包用来寻找头文件。
     - **`${route_planner_INCLUDE_DIRS}`**：表示从`route_planner`包中继承过来的头文件目录。这意味着`behavior_planner`包不仅自己有头文件可以提供给其他包，还可以通过继承来提供`route_planner`包的头文件。

#### 2. **`LIBRARIES`**
   - **`LIBRARIES behavior_planner_lib behavior_planner_ros`**：
     - **`behavior_planner_lib`**：这个库文件是在上文中通过`add_library`命令创建的，包含了行为规划器的核心逻辑。
     - **`behavior_planner_ros`**：这个库文件同样是在上文中通过`add_library`命令创建的，包含了与ROS系统的交互逻辑。

### 整体作用

- **作用**：这段代码将`behavior_planner`包的头文件目录和库文件导出，供其他ROS包使用。`catkin_package`是`catkin`工作空间中每个包都可能包含的一个配置部分，它告诉`catkin`构建系统该包有哪些资源需要导出，以便其他包能够正确链接和包含这些资源。
  
- **典型应用**：在ROS工作空间中，多个包可能相互依赖。通过`catkin_package`声明，`behavior_planner`包将其资源公开，其他包可以在自己的`CMakeLists.txt`文件中通过`find_package(behavior_planner REQUIRED)`找到并使用这些资源。


# include_directories

这段代码是`CMakeLists.txt`文件中的一部分，用于指定项目编译时所需的头文件目录。`include_directories`命令告诉CMake在编译时应该在哪些目录中查找头文件。以下是对该段代码的详细解析：

```cmake
include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
)
```

### 1. **`inc`**
   - **`inc`**：表示当前项目中的`inc`目录。这个目录通常包含项目的头文件。例如，如果你在`inc`目录中有一个`behavior_planner.h`文件，所有包含这个文件的源文件都会从这个目录中查找它。

### 2. **`${catkin_INCLUDE_DIRS}`**
   - **`${catkin_INCLUDE_DIRS}`**：这是由`catkin`生成的一个变量，包含所有`catkin`包的头文件路径。当你在`CMakeLists.txt`文件中使用`find_package(catkin REQUIRED COMPONENTS ...)`时，`catkin_INCLUDE_DIRS`会被设置为这些包的头文件路径。这些路径通常包括标准的ROS消息类型和服务类型的头文件。

### 3. **`${common_INCLUDE_DIRS}`**
   - **`${common_INCLUDE_DIRS}`**：这个变量是通过`find_package(common REQUIRED)`获取的，表示`common`包的头文件目录。`common`包可能包含了项目中常用的一些通用功能或工具类。

### 4. **`${route_planner_INCLUDE_DIRS}`**
   - **`${route_planner_INCLUDE_DIRS}`**：表示`route_planner`包的头文件目录。这个包可能负责路径规划的功能，`behavior_planner`包需要访问它的接口和数据结构。

### 5. **`${semantic_map_manager_INCLUDE_DIRS}`**
   - **`${semantic_map_manager_INCLUDE_DIRS}`**：表示`semantic_map_manager`包的头文件目录。这个包可能用于管理和处理语义地图数据，行为规划器需要从该包中获取地图相关信息。

### 6. **`${vehicle_model_INCLUDE_DIRS}`**
   - **`${vehicle_model_INCLUDE_DIRS}`**：表示`vehicle_model`包的头文件目录。这个包可能包含车辆动力学模型或仿真车辆行为的相关代码，行为规划器会利用这些模型进行决策。

### 7. **`${forward_simulator_INCLUDE_DIRS}`**
   - **`${forward_simulator_INCLUDE_DIRS}`**：表示`forward_simulator`包的头文件目录。这个包可能负责前向仿真或其他相关功能，规划器可能需要调用这些仿真功能来预测未来的车辆行为。

### 总结
- **目的**：`include_directories`命令是为了确保在编译源文件时，CMake能够找到所有必要的头文件。如果源代码中有对这些头文件的引用，CMake会根据这里指定的路径去查找并包含它们。
- **重要性**：这对于跨多个包的项目尤其重要，因为不同的功能模块通常会分布在不同的ROS包中，而这些包的头文件路径需要在编译时正确配置，以便所有依赖关系都能被正确解析。