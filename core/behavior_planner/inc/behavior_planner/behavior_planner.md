### 详细解析 `behavior_planner.h` 和 `behavior_planner.cc`

#### 1. `behavior_planner.h` 文件解析

这个头文件定义了 `BehaviorPlanner` 类的接口和成员变量，该类继承自 `Planner`，是行为规划器的核心类，负责处理自动驾驶车辆的决策逻辑。以下是主要部分的解析：

- **类声明和别名**：
  - `BehaviorPlanner` 继承了 `Planner` 类，并定义了几个常用的类型别名，如 `State`（车辆状态）、`Lane`（车道信息）等。

- **公共方法**：
  - **`Name()`**: 返回行为规划器的名称。
  - **`Init(const std::string config)`**: 初始化行为规划器，加载配置文件，设置一些默认值如 `behavior_` 的初始状态。
  - **`RunOnce()`**: 执行一次规划操作，包含行为决策、参考车道的构建和路径规划调用。
  - **`set_map_interface(BehaviorPlannerMapItf* itf)`**: 设置地图接口，用于获取车辆状态和环境信息。
  - **`set_user_desired_velocity(const decimal_t desired_vel)`**: 设置用户期望的速度，主要用于控制车辆的行驶速度。
  - **`set_hmi_behavior(const LateralBehavior& hmi_behavior)`**: 设置人为驾驶模式下的横向行为（如变道、保持车道等）。
  - **`set_autonomous_level(int level)`**: 设置自动驾驶的级别，影响决策逻辑。
  - **`set_sim_resolution(const decimal_t sim_resolution)`** 和 **`set_sim_horizon(const decimal_t sim_horizon)`**: 设置仿真的分辨率和范围，控制仿真计算的精细程度。
  - **`set_use_sim_state(bool use_sim_state)`**: 设置是否使用仿真状态。
  - **`RunRoutePlanner(const int nearest_lane_id)`**: 调用路径规划模块，根据当前车道ID进行路径规划。
  - **`RunMpdm()`**: 执行MPDM（多策略决策模型）的行为决策。
  
- **受保护方法**：
  - **`ConstructReferenceLane`**: 根据当前的横向行为构建参考车道。
  - **`MultiBehaviorJudge`**: 评估多个可能的行为并选择最优行为。
  - **`OpenloopSimForward`** 和 **`MultiAgentSimForward`**: 用于前向模拟，分别模拟自车和多车的运动轨迹。
  - **`EvaluateMultiPolicyTrajs`** 和 **`EvaluateSinglePolicyTraj`**: 评估多个策略轨迹，选择最佳的轨迹。

- **成员变量**：
  - **`map_itf_`**: 地图接口，用于获取当前的车辆状态和环境信息。
  - **`behavior_`**: 当前车辆的行为状态。
  - **`p_route_planner_`**: 路径规划器的指针。
  - **`sim_param_`**: 仿真参数，控制前向模拟的具体设置。

#### 2. `behavior_planner.cc` 文件解析

这个源文件实现了 `behavior_planner.h` 中声明的 `BehaviorPlanner` 类的所有方法，具体功能如下：

- **`Name()`**: 返回行为规划器的名称，这里返回一个简单的字符串 "Generic behavior planner"。
  
- **`Init(const std::string config)`**: 初始化行为规划器，加载配置文件，初始化路径规划器对象，并设置行为的初始状态。

- **`RunMpdm()`**: 执行MPDM（多策略决策模型）的核心方法：
  - 调用 `MultiBehaviorJudge` 方法评估多个可能的行为，并根据其结果更新 `behavior_`。
  - 如果MPDM成功执行，更新行为规划的相关数据并输出调试信息。

- **`RunRoutePlanner(const int nearest_lane_id)`**: 调用路径规划器，获取当前的道路网络信息，并根据车辆的位置和最近的车道ID进行路径规划。

- **`RunOnce()`**: 是行为规划器的核心方法，执行一次完整的规划流程：
  - 获取自车的车道ID和车辆状态。
  - 如果使用仿真状态，调用 `RunRoutePlanner` 进行路径规划。
  - 判断当前的横向行为，并根据结果更新 `behavior_`。
  - 执行MPDM来确定最优的行为和速度，并构建参考车道。

- **`MultiBehaviorJudge`**: 评估多个可能的行为并选择最优的行为：
  - 获取相关的车辆信息和预测行为。
  - 执行前向模拟，评估每个行为的轨迹。
  - 选择最优的轨迹和行为，并返回相应的速度和行为。

- **`OpenloopSimForward()` 和 `MultiAgentSimForward()`**: 用于前向模拟，分别模拟自车和多车的运动轨迹，通过仿真预测未来的运动状态。

- **`SimulateEgoBehavior()`**: 模拟自车的行为，根据当前状态和目标行为生成相应的轨迹。

- **`EvaluateMultiPolicyTrajs()` 和 `EvaluateSinglePolicyTraj()`**: 评估多个策略轨迹，计算每个轨迹的安全性、效率等指标，选择最优的轨迹。

- **`ConstructReferenceLane()`**: 根据当前的横向行为构建参考车道，考虑了车道保持、变道等情况。

- **`JudgeBehaviorByLaneId()`**: 根据车道ID判断当前的行为。

- **`UpdateEgoLaneId()` 和 `GetPotentialLaneIds()`**: 更新自车的车道ID，并获取可能的车道选项，用于行为决策。

### 总结

`behavior_planner.h` 和 `behavior_planner.cc` 文件定义并实现了一个复杂的自动驾驶行为规划器。通过这些代码，行为规划器能够在复杂的驾驶环境中实时做出合理的决策，确保车辆的安全和效率。整个代码结构设计清晰，功能模块划分明确，各个部分之间的交互逻辑合理。



# UpdateEgoBehavior

函数 `BehaviorPlanner::UpdateEgoBehavior` 的作用是更新自车（Ego Vehicle）的横向行为（Lateral Behavior），确保当前的行为状态与实际的车道位置一致，并处理在变道过程中的行为转换。该函数根据当前行为和由车道ID推断出的行为，决定是否保持当前行为、完成变道或者在某些情况下取消变道。

### 具体解析

```cpp
ErrorType BehaviorPlanner::UpdateEgoBehavior(
    const LateralBehavior& behavior_by_lane_id) {
```

这个函数的参数 `behavior_by_lane_id` 是根据车道ID推断出的横向行为，函数的作用是根据当前的行为状态和这个推断的行为，更新 `behavior_` 对象中的横向行为。

### 函数逻辑

- **行为为 `kLaneKeeping`**:
  - 如果当前行为是 `kLaneKeeping`（保持车道），且推断的行为也是 `kLaneKeeping`，则保持当前行为不变。
  - 如果推断的行为是 `kUndefined`，表明可能发生了非逻辑性的车道跳跃，则保持当前的 `kLaneKeeping` 行为不变。
  - 其他情况（即 `kLaneChangeLeft` 或 `kLaneChangeRight`），表示当前行为判断出错，此时将行为设置为 `kUndefined`，表明系统行为未定义。

- **行为为 `kLaneChangeLeft`**:
  - 如果推断的行为是 `kLaneKeeping`，则表示车辆仍在变道过程中，不做改变。
  - 如果推断的行为是 `kLaneChangeLeft`，表示变道成功，行为更新为 `kLaneKeeping` 并解除人为驾驶锁定（`lock_to_hmi_` 设置为 `false`）。
  - 如果推断的行为是 `kUndefined`，可能表示在变道过程中发生了车道ID的跳跃，取消变道行为，更新为 `kLaneKeeping` 并解除锁定。
  - 其他情况表示行为判断出错，更新行为为 `kUndefined` 并解除锁定。

- **行为为 `kLaneChangeRight`**:
  - 与 `kLaneChangeLeft` 类似，只是方向相反。处理逻辑相同。

### 返回值

- 函数返回一个 `ErrorType`，表示更新行为的结果。该函数内部默认返回 `kSuccess`，表明行为更新正常。

### 总结

该函数的主要作用是根据车辆当前的横向行为和道路情况（推断出的行为）来更新车辆的行为状态，确保车辆在变道或保持车道过程中行为的合理性。函数中对不同行为状态的处理逻辑严密，能够有效处理自动驾驶过程中可能遇到的各种情况，确保车辆行为的安全和一致性。




# GetDesiredVelocityOfTrajectory 



### 函数 `BehaviorPlanner::GetDesiredVelocityOfTrajectory` 的作用

`BehaviorPlanner::GetDesiredVelocityOfTrajectory` 函数的作用是计算一段轨迹（即一系列车辆状态）中适合的目标速度。这个速度通常基于轨迹中的最小速度和车辆在该轨迹上行驶时的最大法向加速度来确定。

### 函数签名

```cpp
ErrorType BehaviorPlanner::GetDesiredVelocityOfTrajectory(
    const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel) {
```

- **参数 `vehicle_vec`**: 
  - 这是一个 `vec_E<common::Vehicle>` 类型的向量，表示一段轨迹中每个时间点上的车辆状态序列。

- **参数 `vel`**: 
  - 这是一个指向 `decimal_t` 类型的指针，用于存储计算出的目标速度。

- **返回值**: 
  - 函数返回一个 `ErrorType` 类型的值，表示计算过程是否成功。正常情况下返回 `kSuccess`。

### 详细解析

```cpp
decimal_t min_vel = kInf;
decimal_t max_acc_normal = 0.0;
for (auto& v : vehicle_vec) {
    auto state = v.state();
    auto acc_normal = fabs(state.curvature) * pow(state.velocity, 2);
    min_vel = acc_normal > max_acc_normal ? state.velocity : min_vel;
}
*vel = min_vel;
```

- **`min_vel = kInf`** 和 **`max_acc_normal = 0.0`**:
  - `min_vel` 初始化为一个非常大的值（无穷大），用来寻找轨迹中的最小速度。
  - `max_acc_normal` 初始化为 `0.0`，用来记录轨迹中最大的法向加速度。

- **循环遍历 `vehicle_vec`**:
  - 对于轨迹中的每一个车辆状态（`v`），首先获取其状态 `state`。
  - `acc_normal` 计算车辆在该状态下的法向加速度。法向加速度是通过曲率（`state.curvature`）和速度（`state.velocity`）的平方计算得出的。

- **更新 `min_vel`**:
  - 如果当前状态的法向加速度 `acc_normal` 大于之前记录的最大法向加速度 `max_acc_normal`，那么更新 `min_vel` 为当前状态的速度。否则，保持 `min_vel` 不变。

- **设置输出速度**:
  - 最终，`min_vel` 被赋值给输出参数 `vel`，表示在该轨迹上行驶时的目标速度。

### 总结

- **主要功能**: 该函数用于评估一段轨迹中适合的目标速度。它通过比较轨迹中各点的法向加速度，确定最能满足安全行驶要求的速度。
- **使用场景**: 在自动驾驶行为规划中，当生成了一条轨迹后，系统需要确定车辆应该以何种速度沿该轨迹行驶，以确保安全和舒适性。
- **返回结果**: 计算得到的目标速度被返回到 `vel` 指针中，函数本身返回 `kSuccess` 表示计算成功。

这个函数在行为规划过程中起到关键作用，确保车辆在复杂环境中能够以合理的速度行驶。


# 裁员变量

### 补充解释 `BehaviorPlanner` 类中的未注释成员变量

- **`decimal_t user_desired_velocity_{5.0};`**
  - **含义**: 用户期望的目标速度，默认为 5.0。该变量用于控制车辆的行驶速度，在自动驾驶级别允许时，会影响规划器的速度决策。

- **`decimal_t reference_desired_velocity_{5.0};`**
  - **含义**: 参考期望速度，通常由路径规划器或仿真结果生成的速度。这个值会影响行为规划器对速度的判断和选择。

- **`int autonomous_level_{2};`**
  - **含义**: 自动驾驶的级别，默认为 2。不同的自动驾驶级别会影响行为决策的策略，例如是否允许人为控制、是否执行自动变道等。

- **`decimal_t sim_resolution_{0.4};`**
  - **含义**: 仿真分辨率，单位是秒，表示在前向仿真中每一步的时间步长。这个值控制仿真的精细程度，较小的分辨率意味着更精确的仿真，但计算量更大。

- **`decimal_t sim_horizon_{4.0};`**
  - **含义**: 仿真范围，表示前向仿真计算的总时间长度。较大的仿真范围可以模拟更长时间内的车辆行为。

- **`int aggressive_level_{3};`**
  - **含义**: 激进程度的参数，控制车辆在仿真中的驾驶风格。较高的激进级别可能意味着更快速的变道、更激进的加速等行为。

- **`bool use_sim_state_ = true;`**
  - **含义**: 是否使用仿真状态。这个布尔值控制行为规划器在决策时是否使用仿真结果。如果为 `true`，则会使用仿真状态来预测未来的运动状态。

- **`bool lock_to_hmi_ = false;`**
  - **含义**: 锁定人为驾驶控制（HMI，Human-Machine Interface）。当该值为 `true` 时，车辆的横向行为会由人为控制决定，不会自动变道。

- **`LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;`**
  - **含义**: 当前在人为驾驶模式下的横向行为，默认值为 `kLaneKeeping`（保持车道）。用于记录人为驾驶模式下的横向指令。

- **`int ego_lane_id_{kInvalidLaneId};`**
  - **含义**: 自车的当前车道ID，默认值为无效的车道ID（`kInvalidLaneId`）。该变量用于跟踪车辆在地图中的位置。

- **`int ego_id_;`**
  - **含义**: 自车的ID，用于标识自车在多车辆仿真中的唯一身份。

- **`std::vector<int> potential_lcl_lane_ids_;`**
  - **含义**: 潜在的左变道车道ID列表。用于记录车辆可以进行左变道的车道选项。

- **`std::vector<int> potential_lcr_lane_ids_;`**
  - **含义**: 潜在的右变道车道ID列表。用于记录车辆可以进行右变道的车道选项。

- **`std::vector<int> potential_lk_lane_ids_;`**
  - **含义**: 潜在的车道保持ID列表。用于记录车辆可以继续保持当前车道的选项。

### 调试和仿真数据相关成员变量

- **`vec_E<vec_E<common::Vehicle>> forward_trajs_;`**
  - **含义**: 自车的前向仿真轨迹。这个变量存储了自车在仿真中的轨迹序列，每个轨迹包含多个时间点的车辆状态。

- **`std::vector<LateralBehavior> forward_behaviors_;`**
  - **含义**: 仿真中使用的横向行为列表。记录在前向仿真中执行的各种横向行为（例如变道、保持车道等）。

- **`vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;`**
  - **含义**: 环境中其他车辆的前向仿真轨迹。这个变量存储了仿真中的每个周围车辆的轨迹数据，按车辆ID进行映射，每个轨迹包含该车辆在仿真中的状态序列。

### 总结

这些成员变量共同用于控制和管理行为规划器的决策逻辑。它们涵盖了从车辆状态、仿真设置、路径规划到车辆行为等多个方面的内容，确保行为规划器能够在复杂环境中做出正确的决策。


# 函数 SimulateEgoBehavior

### 函数 `SimulateEgoBehavior` 解析

#### 函数功能概述

`BehaviorPlanner::SimulateEgoBehavior` 函数的主要功能是模拟自车（Ego Vehicle）在特定横向行为（如变道或保持车道）下的运动轨迹，并生成周围车辆在同一仿真时间段内的轨迹。该函数尝试先使用多智能体仿真进行前向预测，如果失败，则回退到开环仿真。

#### 参数解析

- **`ego_vehicle`**: 自车的当前状态，包括位置、速度等信息。
- **`ego_behavior`**: 自车的目标横向行为，如变道、保持车道等。
- **`semantic_vehicle_set`**: 当前场景中的语义车辆集合，包含所有已知车辆的状态。
- **`traj`**: 输出参数，用于存储自车在仿真过程中的轨迹。
- **`surround_trajs`**: 输出参数，用于存储仿真过程中其他车辆的轨迹。

#### 函数逻辑步骤

1. **计算前后车道长度**：
   - `forward_lane_len`：前向车道长度，基于自车速度和一个固定的最小值（50.0）。
   - `max_backward_len`：后向车道长度，固定为 10.0。

2. **获取参考车道**：
   - 调用 `map_itf_->GetRefLaneForStateByBehavior` 方法，根据自车的当前状态和目标横向行为获取参考车道。如果获取失败，函数返回 `kWrongStatus` 并输出错误信息。

3. **创建语义车辆对象**：
   - 将自车的车辆状态和参考车道组合成 `SemanticVehicle` 对象，并插入到 `semantic_vehicle_set_tmp` 中。这是为了在多智能体仿真中包含自车的状态。

4. **执行多智能体前向仿真**：
   - 调用 `MultiAgentSimForward` 方法，模拟自车和周围车辆在当前行为下的前向运动。如果仿真成功，输出结果到 `traj` 和 `surround_trajs` 中。
   - 如果多智能体仿真失败，则退回到开环仿真，调用 `OpenloopSimForward` 方法再次尝试。如果再次失败，则函数返回 `kWrongStatus`。

5. **输出仿真结果**：
   - 如果仿真成功，输出自车在该行为下的轨迹数量，并返回 `kSuccess`。

#### 关键点解析

- **参考车道**：函数首先通过当前车辆状态和目标行为获取参考车道。这对于保证仿真的合理性和轨迹生成的正确性至关重要。

- **多智能体仿真与开环仿真**：函数优先尝试更复杂的多智能体仿真（`MultiAgentSimForward`），它可以模拟多个车辆之间的交互行为。如果该方法失败（例如因为计算复杂度高或数据不足），则回退到简单的开环仿真（`OpenloopSimForward`），仅对自车的行为进行预测。

- **错误处理**：函数内置了错误处理机制，确保在参考车道获取失败、多智能体仿真失败等情况下，及时返回错误状态 `kWrongStatus`，并输出调试信息。

#### 结论

`SimulateEgoBehavior` 函数是行为规划器中的核心组件之一。它负责在给定目标行为的情况下，对自车及其周围车辆进行仿真预测，生成用于决策的轨迹数据。函数通过参考车道的获取、多智能体仿真与开环仿真的结合，确保了仿真过程的灵活性和可靠性。


#  OpenloopSimForward

### 函数 `BehaviorPlanner::OpenloopSimForward` 解析

#### 函数功能概述

`BehaviorPlanner::OpenloopSimForward` 函数的主要功能是在开环仿真模式下，对自车和周围车辆进行前向模拟，生成一段时间内的轨迹。与多智能体仿真不同，开环仿真假设每辆车的运动是独立的，不会因为其他车辆的行为而改变自身轨迹。

#### 参数解析

- **`ego_semantic_vehicle`**: 自车的语义车辆信息，包括车辆的状态和参考车道。
- **`agent_vehicles`**: 其他车辆的语义车辆集合，包含场景中所有已知车辆的状态和参考车道。
- **`traj`**: 输出参数，用于存储自车在仿真过程中的轨迹。
- **`surround_trajs`**: 输出参数，用于存储仿真过程中其他车辆的轨迹。

#### 函数逻辑步骤

1. **初始化轨迹存储**：
   - 清空并初始化 `traj`，将自车的当前状态作为仿真轨迹的第一个点。
   - 清空并初始化 `surround_trajs`，为每个周围车辆创建一个轨迹容器，并将该车辆的当前状态作为仿真轨迹的第一个点。

2. **确定仿真步数**：
   - 计算仿真的步数 `num_steps_forward`，该值由仿真时间范围（`sim_horizon_`）除以仿真时间步长（`sim_resolution_`）决定。

3. **进行仿真迭代**：
   - **每一步仿真**：
     - 设定自车的期望速度为参考期望速度 `reference_desired_velocity_`。
     - 使用 `planning::OnLaneForwardSimulation::PropagateOnce` 函数模拟自车的下一状态（`ego_state`）。如果仿真失败，返回错误状态 `kWrongStatus`。
     - 创建一个状态缓存 `state_cache`，用于存储每个周围车辆的下一状态。
     - 对每个周围车辆，使用相同的仿真函数 `PropagateOnce` 计算其下一状态，并存储在 `state_cache` 中。如果仿真失败，返回错误状态 `kWrongStatus`。

   - **碰撞检测**：
     - 在每一步仿真中，检查自车是否与其他车辆发生碰撞。如果检测到碰撞，返回错误状态 `kWrongStatus`。

   - **更新状态和轨迹**：
     - 更新自车的当前状态并将其追加到 `traj` 中。
     - 更新每个周围车辆的状态，并将其追加到 `surround_trajs` 中对应车辆的轨迹中。

4. **返回仿真结果**：
   - 如果所有仿真步骤都成功，函数返回 `kSuccess`，表示仿真过程完成。

#### 关键点解析

- **开环仿真**：与多智能体仿真不同，开环仿真假设每个车辆的行为独立于其他车辆，使用固定的车辆模型和期望速度进行模拟。这种仿真方式通常计算量较小，但无法考虑复杂的交互行为。

- **状态缓存 `state_cache`**：在每一步仿真中，函数会计算所有车辆的下一状态，并暂时存储在 `state_cache` 中，以确保所有车辆的状态在同一时间步内更新。

- **碰撞检测**：函数在每次仿真步后都会检查自车与其他车辆是否发生碰撞。这一步骤确保了仿真过程中不会产生不合理的轨迹。

- **期望速度**：仿真过程中使用的期望速度由自车的参考期望速度和其他车辆的当前速度决定。自车的速度通常与路径规划器设定的速度一致。

#### 结论

`OpenloopSimForward` 函数通过开环仿真生成自车和周围车辆在指定时间范围内的轨迹。这些轨迹不考虑车辆间的交互，因此适合用于一些简单的行为模拟或作为多智能体仿真的备选方案。函数结构清晰，涵盖了状态更新、轨迹生成和碰撞检测等关键步骤，确保仿真结果的合理性和安全性。