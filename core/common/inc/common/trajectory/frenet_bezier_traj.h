#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__

#include "common/basics/config.h"
#include "common/spline/bezier.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

namespace common {

class FrenetBezierTrajectory : public FrenetTrajectory {
 public:
  using BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>;

   // 构造函数，一个默认构造函数，一个带参数的构造函数
  FrenetBezierTrajectory() {}
  FrenetBezierTrajectory(const BezierTrajectory& bezier_spline,
                         const StateTransformer& stf)
      : bezier_spline_(bezier_spline), stf_(stf), is_valid_(true) {}
      // {}：这是构造函数的函数体，由于所有的初始化工作都在成员初始化列表中完成了，
      // 所以这里不需要任何额外的代码。

  // 重写FrenetTrajectory中的begin和end方法，返回贝塞尔曲线的起始和结束时间
  decimal_t begin() const override { return bezier_spline_.begin(); }
  decimal_t end() const override { return bezier_spline_.end(); }

  // 检查轨迹是否有效
  bool IsValid() const override { return is_valid_; }

  // 根据时间t获取状态，如果t不在轨迹时间范围内返回错误
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    common::FrenetState fs;
    if (GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }
    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    state->velocity = std::max(0.0, state->velocity);
    return kSuccess;
  }

  // 根据时间t获取Frenet状态，包括位置、速度和加速度
  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    Vecf<2> pos, vel, acc;
    bezier_spline_.evaluate(t, 0, &pos);
    bezier_spline_.evaluate(t, 1, &vel);
    bezier_spline_.evaluate(t, 2, &acc);
    // ~ frenet bezier by default works lateral independent mode
    fs->time_stamp = t;
    fs->Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]),
             common::FrenetState::kInitWithDt);
    if (!fs->is_ds_usable) {
      fs->Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0),
               FrenetState::kInitWithDs);
    }
    return kSuccess;
  }

  // 获取轨迹变量，目前尚未实现
  std::vector<decimal_t> variables() const override {
    // TODO
    return std::vector<decimal_t>();
  }

  // 设置轨迹变量，目前尚未实现
  void set_variables(const std::vector<decimal_t>& variables) override {
    // TODO
  }

  // 计算纵向和横向的急动度，目前尚未实现
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    // TODO
  }

 private:
  BezierTrajectory bezier_spline_;// 贝塞尔曲线轨迹
  StateTransformer stf_;// 状态转换器
  bool is_valid_ = false; // 轨迹是否有效
};

}  // namespace common

#endif