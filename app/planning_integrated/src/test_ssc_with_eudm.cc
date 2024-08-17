/**
 * @file test_ssc_with_eudm.cc
 * @author HKUST Aerial Robotics Group
 * @brief test ssc planner with eudm behavior planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
// ssc_planner_work_rate 和 bp_work_rate：分别设定SSC规划器和行为规划器的工作频率，单位是Hz。
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

// p_ssc_server_ 和 p_bp_server_：分别指向SSC规划器服务器和EUDM规划器服务器的指针，用于管理和调用规划器的功能。
planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::EudmPlannerServer* p_bp_server_{nullptr};


// BehaviorUpdateCallback：行为规划器更新回调函数，当语义地图更新时，将其推送到SSC规划器中。
int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

// SemanticMapUpdateCallback：语义地图更新回调函数，当语义地图更新时，将其推送到行为规划器中。
int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}





int main(int argc, char** argv) {
  // ! 1节点初始化：
  // 使用ros::init初始化ROS节点，并创建一个私有的NodeHandle（~表示私有命名空间）。
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  



  // ! 2参数读取:
  // 通过nh.getParam从参数服务器获取节点参数，如ego_id、agent_config_path、bp_config_path、ssc_config_path。
  // 这些参数用于配置自车ID、代理配置文件路径、行为规划器配置文件路径和SSC规划器配置文件路径。
  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param agent_config_path %s",
              agent_config_path.c_str());
    assert(false);
  }

  std::string bp_config_path;
  if (!nh.getParam("bp_config_path", bp_config_path)) {
    ROS_ERROR("Failed to get param bp_config_path %s", bp_config_path.c_str());
    assert(false);
  }

  std::string ssc_config_path;
  if (!nh.getParam("ssc_config_path", ssc_config_path)) {
    ROS_ERROR("Failed to get param ssc_config_path %s",
              ssc_config_path.c_str());
    assert(false);
  }

  

  // ! 3语义地图管理器：
  // 创建SemanticMapManager实例，并初始化其ROS适配器RosAdapter。适配器用于接收ROS消息并更新语义地图。
  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  // 绑定SemanticMapUpdateCallback到地图更新回调函数，以确保在地图更新时触发行为规划器更新。
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);



  // ! 4规划器初始化：
  // 创建EUDM规划器和SSC规划器的服务器实例，并为EUDM规划器绑定行为更新回调函数。
  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0);
  // Declare bp
  p_bp_server_ = new planning::EudmPlannerServer(nh, bp_work_rate, ego_id);//创建EUDM规划器的服务器实例，
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);//为EUDM规划器绑定行为更新回调函数。

  p_ssc_server_ =
      new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);//创建SSC规划器的服务器实例，

  
  // 调用Init方法初始化两个规划器，加载各自的配置文件。
  p_bp_server_->Init(bp_config_path);
  p_ssc_server_->Init(ssc_config_path);
  smm_ros_adapter.Init();

  // 启动两个规划器服务器以开始规划任务。
  p_bp_server_->Start();
  p_ssc_server_->Start();

  // TicToc timer;
  // 使用ros::Rate控制循环频率（100Hz），在每次循环中调用ros::spinOnce处理ROS消息，并进行必要的休眠以维持循环频率。
  ros::Rate rate(100);
  
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
