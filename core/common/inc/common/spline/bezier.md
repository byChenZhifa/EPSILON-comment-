这段代码实现了一个贝塞尔样条类模板（BezierSpline），用于生成和评估多维度、多段的贝塞尔曲线。贝塞尔曲线在计算机图形学、动画、路径规划等领域有广泛应用。以下是详细解释及增加的注释：

```cpp
#ifndef _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__

#include <assert.h>
#include <vector>

namespace common {
// 前向声明
template <int N_DEG>
class BezierUtils;

/**
 * @brief 贝塞尔样条类模板
 * j段的评估函数为：
 * B_j(t) = s_j * \sum_{i=0}^{N_DEG} c_j^i * b_{N_DEG}^i(t-Tj/s_j)
 * 其中，s_j是第j段的时间缩放因子
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
 public:
  BezierSpline() {}

  /**
   * @brief 设置参数化域
   * @param vec_domain 输入的参数化域向量
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    ctrl_pts_.resize(vec_domain.size() - 1);
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      ctrl_pts_[j].setZero();
    }
  }

  /**
   * @brief 设置控制点的系数
   * @param segment_idx 段索引
   * @param ctrl_pt_index 控制点索引
   * @param coeff 控制点系数
   */
  void set_coeff(const int segment_idx, const int ctrl_pt_index,
                 const Vecf<N_DIM>& coeff) {
    ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff;
  }

  /**
   * @brief 设置控制点
   * @param pts 控制点矩阵向量
   */
  void set_ctrl_pts(const vec_E<Matf<N_DEG + 1, N_DIM>>& pts) {
    ctrl_pts_ = pts;
  }

  /**
   * @brief 获取样条的段数
   * @return 段数
   */
  int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

  /**
   * @brief 获取样条的参数化域
   * @return 参数化域向量
   */
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /**
   * @brief 获取控制点矩阵向量
   * @return 控制点矩阵向量
   */
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts() const { return ctrl_pts_; }

  /**
   * @brief 获取参数化域的起始值
   * @return 起始值
   */
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /**
   * @brief 获取参数化域的结束值
   * @return 结束值
   */
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief 在给定参数化值下评估样条
   * @param s 参数化值，应该在参数化域内
   * @param d 求导次数
   * @param ret 返回值
   * @note 参数化值超出范围将被四舍五入到end()
   */
  ErrorType evaluate(const decimal_t s, const int d, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx =
        std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0),
                 num_pts - 2);
    decimal_t h = s - vec_domain_[idx];
    if (s < vec_domain_[0]) {
      return kIllegalInput;
    } else {
      decimal_t duration = vec_domain_[idx + 1] - vec_domain_[idx];
      decimal_t normalized_s = h / duration;
      Vecf<N_DEG + 1> basis =
          BezierUtils<N_DEG>::GetBezierBasis(d, normalized_s);
      Vecf<N_DIM> result =
          (pow(duration, 1 - d) * basis.transpose() * ctrl_pts_[idx])
              .transpose();
      *ret = result;
    }
    return kSuccess;
  }

  /**
   * @brief 打印控制点信息
   */
  void print() const {
    printf("Bezier control points.\n");
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      printf("segment %d -->.\n", j);
      std::cout << ctrl_pts_[j] << std::endl;
    }
  }

 private:
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts_;  // 控制点矩阵向量
  std::vector<decimal_t> vec_domain_;  // 参数化域向量
};

/**
 * @brief 贝塞尔工具类模板
 */
template <int N_DEG>
class BezierUtils {
 public:
  /**
   * @brief 计算非缩放贝塞尔基函数的Hessian矩阵
   */
  static MatNf<N_DEG + 1> GetBezierHessianMat(int derivative_degree) {
    MatNf<N_DEG + 1> hessian;
    switch (N_DEG) {
      case 5: {
        if (derivative_degree == 3) {
          // 计算jerk的Hessian矩阵
          hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0,
              -3600.0, 0.0, 600.0, 0.0, 1200.0, -3600.0, 3600.0, -1200.0, 0.0,
              0.0, 0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0, 0.0, 600.0, 0.0,
              -3600.0, 4800.0, -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0,
              720.0;
          break;
        } else {
          assert(false);
        }
        break;
      }
      default:
        assert(false);
    }
    return hessian;
  }

  /**
   * @brief 计算非缩放控制点的贝塞尔基函数
   */
  static Vecf<N_DEG + 1> GetBezierBasis(int derivative_degree, decimal_t t) {
    t = std::max(std::min(1.0, t), 0.0);
    Vecf<N_DEG + 1> basis;
    switch (N_DEG) {
      case 5:
        if (derivative_degree == 0) {
          basis << -pow(t - 1, 5), 5 * t * pow(t - 1, 4),
              -10 * pow(t, 2) * pow(t - 1, 3), 10 * pow(t, 3) * pow(t - 1, 2),
              -5 * pow(t, 4) * (t - 1), pow(t, 5);
        } else if (derivative_degree == 1) {
          basis << -5 * pow(t - 1, 4),
              20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
              -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
              10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
              -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4), 5 * pow(t, 4);
        } else if (derivative_degree == 2) {
          basis << -20 * pow(t - 1, 3),
              60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3),
              -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) -
                  30 * t * t * (2 * t - 2),
              60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) +
                  20 *

 t * t * t,
              -60 * t * t * (t - 1) - 40 * t * t * t, 20 * t * t * t;
        } else if (derivative_degree == 3) {
          basis << -60 * pow(t - 1, 2),
              60 * t * (2 * t - 2) + 180 * pow(t - 1, 2),
              -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t,
              180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t,
              -120 * t * (t - 1) - 180 * t * t, 60 * t * t;
        }
        break;
      default:
        printf("N_DEG %d, derivative_degree %d.\n", N_DEG, derivative_degree);
        assert(false);
        break;
    }
    return basis;
  }
};

}  // namespace common

#endif
```

### 主要步骤和原理：

#### BezierSpline 类模板

1. **构造函数和参数化域设置：**
   - `BezierSpline()`：默认构造函数。
   - `set_vec_domain(const std::vector<decimal_t>& vec_domain)`：设置参数化域，并初始化控制点矩阵。

2. **控制点的设置：**
   - `set_coeff(const int segment_idx, const int ctrl_pt_index, const Vecf<N_DIM>& coeff)`：设置指定段和控制点的系数。
   - `set_ctrl_pts(const vec_E<Matf<N_DEG + 1, N_DIM>>& pts)`：设置所有控制点。

3. **获取样条的段数和参数化域：**
   - `num_segments() const`：返回样条的段数。
   - `vec_domain() const`：返回参数化域向量。
   - `ctrl_pts() const`：返回控制点矩阵向量。

4. **获取参数化域的起始和结束值：**
   - `begin() const`：返回参数化域的起始值。
   - `end() const`：返回参数化域的结束值。

5. **样条的评估：**
   - `evaluate(const decimal_t s, const int d, Vecf<N_DIM>* ret) const`：在给定参数化值 `s` 下评估样条，计算第 `d` 阶导数，并将结果存储在 `ret` 中。

6. **打印控制点信息：**
   - `print() const`：打印每段样条的控制点信息。

#### BezierUtils 类模板

1. **计算Hessian矩阵：**
   - `static MatNf<N_DEG + 1> GetBezierHessianMat(int derivative_degree)`：计算非缩放贝塞尔基函数的Hessian矩阵。对于五次贝塞尔曲线，当导数次数为3时，计算其Hessian矩阵。

2. **计算贝塞尔基函数：**
   - `static Vecf<N_DEG + 1> GetBezierBasis(int derivative_degree, decimal_t t)`：计算非缩放控制点的贝塞尔基函数。根据不同的导数次数，计算相应的贝塞尔基函数。

这个代码实现了贝塞尔样条的生成和评估，通过设置控制点和参数化域，可以生成多段的贝塞尔曲线，并在给定参数化值下进行评估。贝塞尔工具类提供了计算基函数和Hessian矩阵的方法，用于曲线的进一步优化和分析。