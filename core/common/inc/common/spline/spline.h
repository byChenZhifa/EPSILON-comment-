#ifndef _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__

#include "common/spline/polynomial.h"

#include <assert.h>
#include <vector>

namespace common {

template <int N_DEG, int N_DIM>
// 定义了一个通用的样条（Spline）类模板，用于在给定的参数化域上创建和评估多维多项式样条
class Spline {
 public:
  typedef PolynomialND<N_DEG, N_DIM> PolynomialType;

  Spline() {}

  /**
   * @brief 设置参数化域
   * @param vec_domain 输入的参数化域向量
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    poly_.resize(vec_domain_.size() - 1);
  }

  /**
   * @brief Get the number of segments of the spline 获取样条的段数
   * @param n number of the segments  段数
   */
  int num_segments() const { return static_cast<int>(poly_.size()); }

  /**
   * @brief Get the vec domain of the spline 获取样条的参数化域向量
   * @param n number of the segments   参数化域向量
   */
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /**
   * @brief Get the begin of the parameterization 获取参数化的起始值
   */
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /**
   * @brief Get the end of the parameterization 获取参数化的结束值
   */
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief Return a reference to a certain 1D polynomial
   * @param n index of the segment
   * @param j index of the dimension
   */
   /**
   * @brief 返回对某个1D多项式的引用
   * @param n 段索引
   * @param j 维度索引
   * @return 对应的多项式
   */
  Polynomial<N_DEG>& operator()(int n, int j) {
    assert(n < this->num_segments());
    assert(j < N_DIM);
    return poly_[n][j];
  }

  /**
   * @brief Return evaluation of spline at given parameterization
   * @param s parameterization, s should be in the vector domain
   * @param d derivate to take
   * @note  the function will automatically do extrapolation if out of domain
   */
  /**
   * @brief 在给定参数化值下评估样条
   * @param s 参数化值，s 应在参数化域内
   * @param d 求导次数
   * @param ret 返回值
   * @return 错误类型
   */
  ErrorType evaluate(const decimal_t s, int d, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];

    if (s < vec_domain_[0]) {
      poly_[0].evaluate(h, d, ret);
      // return kIllegalInput;
    } else if (s > vec_domain_[num_pts - 1]) {
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, d, ret);
      // return kIllegalInput;
    } else {
      poly_[idx].evaluate(h, d, ret);
    }
    return kSuccess;
  }

  /**
   * @brief 在给定参数化值下评估样条
   * @param s 参数化值，s 应在参数化域内
   * @param ret 返回值
   * @return 错误类型
   */
  ErrorType evaluate(const decimal_t s, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];

    if (s < vec_domain_[0]) {
      poly_[0].evaluate(h, ret);
      // return kIllegalInput;
    } else if (s > vec_domain_[num_pts - 1]) {
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, ret);
      // return kIllegalInput;
    } else {
      poly_[idx].evaluate(h, ret);
    }
    return kSuccess;
  }

  /**
   * @brief 打印样条信息
   */
  void print() const {
    int num_polys = static_cast<int>(poly_.size());
    for (int i = 0; i < num_polys; i++) {
      printf("vec domain (%lf, %lf).\n", vec_domain_[i], vec_domain_[i + 1]);
      poly_[i].print();
      Vecf<2> v;
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 0, &v);
      printf("end pos: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 1, &v);
      printf("end vel: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 2, &v);
      printf("end acc: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 3, &v);
      printf("end jerk: (%lf, %lf).\n", v[0], v[1]);
    }
  }

 private:
  vec_E<PolynomialType> poly_;// 多维多项式的向量
  std::vector<decimal_t> vec_domain_;// 参数化域向量
};

}  // namespace common

#endif