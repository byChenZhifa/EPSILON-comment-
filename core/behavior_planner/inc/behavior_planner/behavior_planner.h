#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_

#include <memory>
#include <string>

#include "behavior_planner/map_interface.h"
#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "route_planner/route_planner.h"

#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
namespace planning {

class BehaviorPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  std::string Name() override;

  //初始化行为规划器，加载配置文件。
  ErrorType Init(const std::string config) override;

  //执行一次规划操作，通常在循环中调用。
// - **`RunOnce()`**: 是行为规划器的核心方法，执行一次完整的规划流程：
//   - 获取自车的车道ID和车辆状态。
//   - 如果使用仿真状态，调用 `RunRoutePlanner` 进行路径规划。
//   - 判断当前的横向行为，并根据结果更新 `behavior_`。
//   - 执行MPDM来确定最优的行为和速度，并构建参考车道。
  ErrorType RunOnce() override;

  //设置地图接口，用于获取车辆状态和环境信息。
  void set_map_interface(BehaviorPlannerMapItf* itf);


  /**
   * @brief set desired velocity
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  /**
   * @brief L2-level human-commanded lane changes
   **/
  void set_hmi_behavior(const LateralBehavior& hmi_behavior);

  /**
   * @brief set the level of autonomous driving
   */
  void set_autonomous_level(int level);

  //设置仿真的分辨率和范围，控制仿真计算的精细程度。
  void set_sim_resolution(const decimal_t sim_resolution);

  void set_sim_horizon(const decimal_t sim_horizon);

  //设置是否使用仿真状态。
  void set_use_sim_state(bool use_sim_state);

  void set_aggressive_level(int level);

  //运行路径规划器，根据当前车道ID进行路径规划。
  ErrorType RunRoutePlanner(const int nearest_lane_id);

  //执行MPDM（多策略决策模型）的行为决策。
    //调用 MultiBehaviorJudge 方法评估多个可能的行为，并根据其结果更新 behavior_。
    //如果MPDM成功执行，更新行为规划的相关数据并输出调试信息。
  ErrorType RunMpdm();

  Behavior behavior() const;

  decimal_t user_desired_velocity() const;

  decimal_t reference_desired_velocity() const;

  int autonomous_level() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  std::vector<LateralBehavior> forward_behaviors() const;

 protected:
  // 根据当前的横向行为构建参考车道，考虑了车道保持、变道等情况。
  ErrorType ConstructReferenceLane(const LateralBehavior& lat_behavior,
                                   Lane* lane);

  //评估多个可能的行为并选择最优行为。
  ErrorType ConstructLaneFromSamples(const vec_E<Vecf<2>>& samples, Lane* lane);

//  - **`MultiBehaviorJudge`**: 评估多个可能的行为并选择最优的行为：
//   - 获取相关的车辆信息和预测行为。
//   - 执行前向模拟，评估每个行为的轨迹。
//   - 选择最优的轨迹和行为，并返回相应的速度和行为。
  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LateralBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  
  
//   - **`UpdateEgoLaneId()` 和 `GetPotentialLaneIds()`**: 
// 更新自车的车道ID，并获取可能的车道选项，用于行为决策。
  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  
// - **`JudgeBehaviorByLaneId()`**: 根据车道ID判断当前的行为。
  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);



// 作用是更新自车（Ego Vehicle）的横向行为（Lateral Behavior），
// 确保当前的行为状态与实际的车道位置一致，并处理在变道过程中的行为转换。
// 该函数根据当前行为和由车道ID推断出的行为，
// 决定是否保持当前行为、完成变道或者在某些情况下取消变道。
  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);







//   用于前向模拟：多智能体前向模拟
  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

//- **`OpenloopSimForward()` 和 `MultiAgentSimForward()`**: 
//   用于前向模拟：车辆开环模拟
// 主要功能是在开环仿真模式下，对自车和周围车辆进行前向模拟，生成一段时间内的轨迹。
// 与多智能体仿真不同，开环仿真假设每辆车的运动是独立的，不会因为其他车辆的行为而改变自身轨迹。
  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);


// !- **`SimulateEgoBehavior()`**: 模拟自车的行为，根据当前状态和目标行为生成相应的轨迹。
// 模拟自车（Ego Vehicle）在特定横向行为（如变道或保持车道）下的运动轨迹，并生成周围车辆在同一仿真时间段内的轨迹。
// 该函数尝试先使用多智能体仿真 MultiAgentSimForward 进行前向预测，如果失败，则回退到开环仿真 OpenloopSimForward 。
  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);



// - **`EvaluateMultiPolicyTrajs()` 和 `EvaluateSinglePolicyTraj()`**: 
// 评估多个策略轨迹，计算每个轨迹的安全性、效率等指标，选择最优的轨迹。
  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LateralBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
          valid_surround_trajs,
      LateralBehavior* winner_behavior,
      vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
      decimal_t* desired_vel);

  ErrorType EvaluateSinglePolicyTraj(
      const LateralBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);





// ---------------------- **成员变量**：----------------------
  BehaviorPlannerMapItf* map_itf_{nullptr};//地图接口，用于获取当前的车辆状态和环境信息。
  Behavior behavior_;//: 当前车辆的行为状态。

  planning::RoutePlanner* p_route_planner_{nullptr};// 路径规划器的指针。

// 用户期望的目标速度，默认为 5.0。
// 该变量用于控制车辆的行驶速度，在自动驾驶级别允许时，会影响规划器的速度决策。
  decimal_t user_desired_velocity_{5.0};
  decimal_t reference_desired_velocity_{5.0};//参考期望速度，

  //自动驾驶的级别，默认为 2。不同的自动驾驶级别会影响行为决策的策略，例如是否允许人为控制、是否执行自动变道等。
  int autonomous_level_{2};

  decimal_t sim_resolution_{0.4};//仿真分辨率，单位是秒，表示在前向仿真中每一步的时间步长。
  decimal_t sim_horizon_{4.0};// 仿真范围，表示前向仿真计算的总时间长度。

  // 激进程度的参数，控制车辆在仿真中的驾驶风格。较高的激进级别可能意味着更快速的变道、更激进的加速等行为。
  int aggressive_level_{3};//
  planning::OnLaneForwardSimulation::Param sim_param_;//仿真参数，控制前向模拟的具体设置。



// 是否使用仿真状态。这个布尔值控制行为规划器在决策时是否使用仿真结果。如果为 true，则会使用仿真状态来预测未来的运动状态。
  bool use_sim_state_ = true;
//   锁定人为驾驶控制（HMI，Human-Machine Interface）。当该值为 true 时，车辆的横向行为会由人为控制决定，不会自动变道。
  bool lock_to_hmi_ = false;
//   当前在人为驾驶模式下的横向行为，默认值为 kLaneKeeping（保持车道）。用于记录人为驾驶模式下的横向指令。
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

  // track the ego lane id
//   自车的当前车道ID，默认值为无效的车道ID（kInvalidLaneId）。该变量用于跟踪车辆在地图中的位置。
  int ego_lane_id_{kInvalidLaneId};
  int ego_id_;//自车的ID，用于标识自车在多车辆仿真中的唯一身份。
  std::vector<int> potential_lcl_lane_ids_;//潜在的左变道车道ID列表。用于记录车辆可以进行左变道的车道选项。
  std::vector<int> potential_lcr_lane_ids_;// 潜在的右变道车道ID列表。用于记录车辆可以进行右变道的车道选项。
  std::vector<int> potential_lk_lane_ids_;//潜在的车道保持ID列表。用于记录车辆可以继续保持当前车道的选项。



  // debug  
  // 调试和仿真数据相关成员变量
  vec_E<vec_E<common::Vehicle>> forward_trajs_;//自车的前向仿真轨迹。这个变量存储了自车在仿真中的轨迹序列，每个轨迹包含多个时间点的车辆状态。
  std::vector<LateralBehavior> forward_behaviors_;//仿真中使用的横向行为列表。记录在前向仿真中执行的各种横向行为（例如变道、保持车道等）。
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;//含义: 环境中其他车辆的前向仿真轨迹。
//   这个变量存储了仿真中的每个周围车辆的轨迹数据，按车辆ID进行映射，每个轨迹包含该车辆在仿真中的状态序列。

};

}  // namespace planning

#endif