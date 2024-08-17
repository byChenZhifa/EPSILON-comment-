
# test_ssc_with_eudm_ros.launch
`test_ssc_with_eudm_ros.launch` 是一个ROS launch文件，它用于启动和配置EPSILON系统中的相关节点。这个文件通常是用于测试SSC（Spatio-Temporal Semantic Corridor）和EUDM（Efficient Uncertainty-aware Decision Making）模块的ROS环境中的一部分。通过分析这个launch文件，我们可以了解系统如何组织这些组件来进行规划测试。

### 1. **Launch文件的作用**
   - **启动多个节点**：ROS的launch文件可以同时启动多个ROS节点。它通过定义每个节点的启动参数和依赖关系，确保它们能够正确地在指定的顺序下启动。
   - **配置参数**：launch文件中可以设置参数，指定各个节点的运行配置，比如加载哪些配置文件、使用哪些话题（topics）等。

### 2. **文件内容解读**
   虽然没有直接显示文件内容，但根据文件名和路径推测，该文件可能包含以下内容：

   - **SSC相关节点**：启动和配置与SSC相关的节点，用于路径规划和轨迹生成。这些节点可能需要接收感知数据并生成可供车辆执行的轨迹。
   - **EUDM相关节点**：启动与决策规划相关的节点，这些节点使用不确定性感知的算法，来决定车辆的下一步行动。
   - **话题和服务配置**：配置ROS话题，用于在不同节点之间传递消息。例如，感知数据、路径信息、控制指令等。
   - **参数加载**：加载配置文件或参数服务器中的参数，影响系统的运行行为。例如，车辆动力学模型参数、路径规划算法参数等。

### 3. **如何使用这个文件**
   - **运行方法**：在ROS工作空间中，可以通过以下命令启动该launch文件：
     ```bash
     roslaunch planning_integrated test_ssc_with_eudm_ros.launch
     ```
   - **调试和测试**：开发人员可以使用这个launch文件来调试SSC和EUDM模块，观察系统在不同环境下的行为，并通过调整参数来优化性能。

### 4. **关联组件**
   - **SSC**：该模块负责生成车辆在复杂城市环境中的时空语义走廊，通过规划安全且高效的路径来引导车辆行驶。
   - **EUDM**：决策模块，结合环境的不确定性，通过评估不同可能性来选择最佳驾驶策略。

通过使用`test_ssc_with_eudm_ros.launch`文件，开发者能够方便地测试EPSILON系统中的规划功能，尤其是在动态和复杂的城市环境中，确保车辆能够做出安全且高效的驾驶决策。

这段`ROS launch`文件的内容配置了一个节点，用于测试EPSILON系统中的SSC（Spatio-Temporal Semantic Corridor）和EUDM（Efficient Uncertainty-aware Decision Making）模块。以下是对该launch文件的详细解析：


# code
### 1. **参数定义**
   - **`<arg>` 标签**：用于定义参数，这些参数可以在启动时动态指定，也可以在文件中提供默认值。
     - `arena_info_static_topic` 和 `arena_info_dynamic_topic`：定义静态和动态场地信息的话题名称，默认值分别为`/arena_info_static`和`/arena_info_dynamic`。
     - `ctrl_topic`：控制信号的话题名称，默认值为`/ctrl/agent_0`。
     - `playground`：指定测试环境的配置文件，可以选择不同的场景，例如`highway_v1.0`，`ring_tiny_v1.0`等。当前选择的是`highway_v1.0`。

### 2. **节点配置**
   - **`<node>` 标签**：启动一个ROS节点。
     - **`pkg="planning_integrated"`**：指定节点所属的ROS包名称为`planning_integrated`。
     - **`type="test_ssc_with_eudm"`**：节点的可执行文件名称，这个可执行文件负责测试SSC和EUDM模块。
     - **`name="test_ssc_with_eudm_0"`**：ROS节点的名称。
     - **`output="screen"`**：将节点的输出信息打印到终端屏幕上。

### 3. **节点参数**
   - **`param` 标签**：为节点设置参数。
     - **`ego_id`**：自车ID，设定为`0`，表示这是自车的唯一标识。
     - **`agent_config_path`**：指定代理配置文件的路径，该路径由`playgrounds`目录下的指定场景（通过`playground`参数）中的`agent_config.json`文件决定。
     - **`bp_config_path`**：行为规划器（Behavior Planner）的配置文件路径，位于`eudm_planner`包的配置文件夹下。
     - **`ssc_config_path`**：时空语义走廊规划器（SSC Planner）的配置文件路径，位于`ssc_planner`包的配置文件夹下。
     - **`desired_vel`**：期望速度，设定为`20.0`（单位：m/s）。
     - **`use_sim_state`**：指定是否使用模拟状态，设定为`true`。

### 4. **话题重映射**
   - **`<remap>` 标签**：将节点内部使用的话题名称重映射到全局的话题名称。
     - **`~arena_info_static`** 重映射到 `$(arg arena_info_static_topic)`，即 `/arena_info_static`。
     - **`~arena_info_dynamic`** 重映射到 `$(arg arena_info_dynamic_topic)`，即 `/arena_info_dynamic`。
     - **`~ctrl`** 重映射到 `$(arg ctrl_topic)`，即 `/ctrl/agent_0`。

### 5. **总结**
这个launch文件的主要作用是启动一个用于测试的节点，该节点集成了SSC和EUDM两个模块。通过指定不同的场景配置文件和规划器配置文件，用户可以在不同的虚拟环境中测试这些模块的性能和行为。参数和话题的重映射使得这个测试环境可以灵活适应不同的测试需求。