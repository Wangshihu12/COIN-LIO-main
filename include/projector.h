// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_PROJECTOR_H_
#define COIN_LIO_PROJECTOR_H_

#include <common_lib.h>
#include "ros/ros.h"

/**
 * @brief Projector类，提供点云投影、图像生成及相关功能的接口。
 *
 * 该类用于将点云数据投影到图像平面，生成图像及其他相关操作，
 * 例如计算投影雅可比矩阵、检查点是否在视野范围内等。
 */
class Projector
{
public:
  /**
   * @brief 构造函数，初始化投影器并加载相关参数。
   * @param nh ROS节点句柄，用于加载参数。
   */
  Projector(ros::NodeHandle nh);

  /**
   * @brief 创建投影图像，包括强度图、距离图等。
   * @param frame Lidar帧数据，用于生成投影图像。
   */
  void createImages(LidarFrame &frame) const;

  /**
   * @brief 计算投影的雅可比矩阵。
   * @param p 输入的3D点坐标。
   * @param du_dp 输出的雅可比矩阵。
   */
  void projectionJacobian(const V3D &p, Eigen::MatrixXd &du_dp) const;

  /**
   * @brief 将点从畸变矫正坐标投影到图像平面。
   * @param frame 当前Lidar帧数据。
   * @param p_L_k 输入的矫正点坐标。
   * @param p_L_i 输出的畸变点坐标。
   * @param uv 输出的图像平面坐标。
   * @param distortion_idx 输出的畸变点索引。
   * @param round 是否对坐标进行四舍五入。
   * @return 如果投影成功，返回true；否则返回false。
   */
  bool projectUndistortedPoint(const LidarFrame &frame, const V3D &p_L_k, V3D &p_L_i, V2D &uv, int &distortion_idx,
                               bool round = false) const;

  /**
   * @brief 根据图像像素坐标获取索引。
   * @param px 图像平面坐标。
   * @return 像素对应的线性索引。
   */
  size_t indexFromPixel(const V2D &px) const;

  /**
   * @brief 将3D点投影到图像平面。
   * @param point 输入的3D点。
   * @param uv 输出的图像平面坐标。
   * @return 如果投影成功，返回true；否则返回false。
   */
  bool projectPoint(const V3D &point, V2D &uv) const;

  /**
   * @brief 检查点是否位于视野范围内。
   * @param uv 输入的图像平面坐标。
   * @return 如果点在视野范围内，返回true；否则返回false。
   */
  bool isFOV(const V2D &uv) const;

  /**
   * @brief 获取图像行数。
   * @return 图像的行数。
   */
  int rows() const { return rows_; }

  /**
   * @brief 获取图像列数。
   * @return 图像的列数。
   */
  int cols() const { return cols_; }

private:
  /**
   * @brief 加载参数。
   * @param nh ROS节点句柄，用于加载参数。
   */
  void loadParameters(ros::NodeHandle nh);

  /**
   * @brief 根据行列索引计算线性索引。
   * @param row 行索引。
   * @param col 列索引。
   * @return 对应的线性索引。
   */
  size_t vectorIndexFromRowCol(const size_t row, const size_t col) const;

  double beam_offset_m_;                 // 激光束偏移量（米）。
  int u_shift_;                          // 列方向的偏移量。
  size_t rows_;                          // 图像的行数。
  size_t cols_;                          // 图像的列数。
  std::vector<int> offset_lut_;          // 偏移查找表。
  std::vector<double> elevation_angles_; // 垂直角度列表。
  std::vector<int> idx_to_v_;            // 索引到行的映射。
  std::vector<int> idx_to_u_;            // 索引到列的映射。
  M3D K_;                                // 内参矩阵。
  bool destagger_;                       // 是否取消交错。
};

#endif // COIN_LIO_PROJECTOR_H_