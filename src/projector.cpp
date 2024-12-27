// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include "projector.h"
#include <stdexcept>

#define DUPLICATE_POINTS 10

/**
 * @brief Projector构造函数，用于初始化投影器
 *
 * @param nh ROS节点句柄，用于加载参数
 *
 * 该构造函数完成了参数加载、相机内参矩阵计算、角度单位转换以及索引到像素坐标的查找表初始化。
 */
Projector::Projector(ros::NodeHandle nh)
{
    try
    {
        // 加载参数，若加载失败抛出异常并退出
        loadParameters(nh);
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR_STREAM(e.what()); // 打印错误信息
        exit(1);                    // 程序异常退出
    }

    // 将仰角数组中的角度值从度转换为弧度
    for (auto &angle : elevation_angles_)
    {
        angle *= M_PI / 180.0;
    }

    // 根据图像行数和列数以及仰角范围计算相机内参参数fy、fx、cy、cx
    double fy = -static_cast<double>(rows_) / fabs(elevation_angles_[0] -
                                                   elevation_angles_[elevation_angles_.size() - 1]); // 垂直方向焦距
    double fx = -static_cast<double>(cols_) / (2 * M_PI);                                            // 水平方向焦距
    double cy = rows_ / 2;                                                                           // 垂直方向光心位置
    double cx = cols_ / 2;                                                                           // 水平方向光心位置

    // 初始化相机内参矩阵K_
    K_ << fx, 0, cx,
        0, fy, cy,
        0, 0, 1;

    // 将激光束偏移值从毫米转换为米
    beam_offset_m_ *= 1e-3;

    // 初始化索引到图像行列的查找表
    idx_to_v_ = std::vector<int>(rows_ * cols_, 0); // 行索引表
    idx_to_u_ = std::vector<int>(rows_ * cols_, 0); // 列索引表
    for (size_t i = 0; i < rows_; ++i)
    {
        for (size_t j = 0; j < cols_; ++j)
        {
            // 根据像素位置计算索引
            auto idx = indexFromPixel(V2D(i, j));
            idx_to_v_[idx] = i;            // 存储行索引
            idx_to_u_[idx] = j - u_shift_; // 存储列索引并应用偏移量

            // 如果列索引小于0，调整到有效范围内
            if (idx_to_u_[idx] < 0)
            {
                idx_to_u_[idx] += cols_;
            }
            // 如果列索引超过最大值，调整到有效范围内
            if (idx_to_u_[idx] >= cols_)
            {
                idx_to_u_[idx] -= cols_;
            }
        }
    }
}

void Projector::loadParameters(ros::NodeHandle nh)
{
    float rows;
    if (!(nh.getParam("/data_format/pixels_per_column", rows) ||
          nh.getParam("/lidar_data_format/pixels_per_column", rows)))
    {
        throw std::runtime_error("Missing rows parameter");
        return;
    }
    rows_ = static_cast<size_t>(rows);

    float cols;
    if (!(nh.getParam("/data_format/columns_per_frame", cols) ||
          nh.getParam("/lidar_data_format/columns_per_frame", cols)))
    {
        throw std::runtime_error("Missing cols parameter");
        return;
    }
    cols_ = static_cast<size_t>(cols);

    if (!(nh.getParam("/lidar_origin_to_beam_origin_mm", beam_offset_m_) ||
          nh.getParam("/beam_intrinsics/lidar_origin_to_beam_origin_mm", beam_offset_m_)))
    {
        throw std::runtime_error("Missing beam_offset_m parameter");
        return;
    }

    if (!(nh.getParam("/data_format/pixel_shift_by_row", offset_lut_) ||
          nh.getParam("/lidar_data_format/pixel_shift_by_row", offset_lut_)))
    {
        throw std::runtime_error("Missing offset parameter");
        return;
    }

    if (!(nh.getParam("/beam_altitude_angles", elevation_angles_) ||
          nh.getParam("/beam_intrinsics/beam_altitude_angles", elevation_angles_)))
    {
        throw std::runtime_error("Missing alt parameter");
        return;
    }

    float u_shift;
    if (!nh.getParam("image/u_shift", u_shift))
    {
        throw std::runtime_error("Missing column shift parameter");
        return;
    }
    u_shift_ = static_cast<int>(u_shift);

    // true: destagger the pointcloud data
    // https://static.ouster.dev/sdk-docs/reference/lidar-scan.html#staggering-and-destaggering
    nh.param<bool>("image/destagger", destagger_, true);
}

/**
 * @brief 根据行列索引计算点云在向量中的一维索引
 *
 * @param row 点云在图像中的行索引
 * @param col 点云在图像中的列索引
 * @return size_t 点云在一维向量中的索引
 *
 * 该函数通过给定的行索引和列索引，结合列数和DUPLICATE_POINTS参数，
 * 计算点云在内部存储的一维向量中的索引。
 */
size_t Projector::vectorIndexFromRowCol(const size_t row, const size_t col) const
{
    // 使用行列索引、列数和DUPLICATE_POINTS参数计算向量索引
    return (row * cols_ + col) * DUPLICATE_POINTS;
}

/**
 * @brief 创建激光雷达帧的投影图像
 *
 * @param frame 激光雷达帧，包含校正后的点云和投影相关的数据结构
 *
 * 该函数将校正后的点云投影到图像上，生成深度图像、强度图像以及索引图像。
 * 同时建立点云与图像之间的映射关系，用于后续处理。
 */
void Projector::createImages(LidarFrame &frame) const
{
    // 初始化强度图、深度图和索引图
    frame.img_intensity = cv::Mat::zeros(rows_, cols_, CV_32FC1);           // 初始化强度图为零矩阵
    frame.img_range = cv::Mat::zeros(rows_, cols_, CV_32FC1);               // 初始化深度图为零矩阵
    frame.img_idx = cv::Mat::ones(rows_, cols_, CV_32SC1) * (-1);           // 初始化索引图为-1
    frame.proj_idx = std::vector<int>(rows_ * cols_ * DUPLICATE_POINTS, 0); // 初始化投影索引向量

    // 用于记录点云索引与图像像素位置的映射
    std::vector<int> idx_to_vk(frame.points_corrected->points.size(), -1);
    std::vector<int> idx_to_uk(frame.points_corrected->points.size(), -1);

    // 将校正后的点云投影到图像平面上
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM); // 设置并行线程数
#pragma omp parallel for
#endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j)
    {
        const V3D p_Lk = frame.points_corrected->points[j].getVector3fMap().cast<double>(); // 获取点的3D坐标
        V2D px_k;
        if (!projectPoint(p_Lk, px_k)) // 投影点到图像平面
            continue;
        idx_to_uk[j] = std::round(px_k(0)); // 记录投影到的图像列
        idx_to_vk[j] = std::round(px_k(1)); // 记录投影到的图像行
    }

    // 使用点云的索引构建图像表示
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j)
    {
        // 从全局索引映射到图像像素位置
        int v_i = idx_to_v_[frame.points_corrected->points[j].normal_x];
        int u_i = idx_to_u_[frame.points_corrected->points[j].normal_x];
        const float p_norm = frame.points_corrected->points[j].getVector3fMap().norm();         // 计算点的距离
        frame.img_range.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].normal_y;      // 填充深度值
        frame.img_intensity.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].intensity; // 填充强度值

        // 记录点云索引到图像
        frame.img_idx.ptr<int>(v_i)[u_i] = j;
    }

    // 记录点云与图像的投影索引映射
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j)
    {
        int u_k = idx_to_uk[j];
        int v_k = idx_to_vk[j];
        if (u_k < 0 || v_k < 0) // 跳过无效投影点
            continue;
        size_t start_idx = vectorIndexFromRowCol(v_k, u_k); // 获取投影点的起始索引
        size_t offset = frame.proj_idx[start_idx] + 1;      // 更新当前像素的点数计数
        if (offset >= DUPLICATE_POINTS)                     // 如果超出最大点数，跳过该点
            continue;
        size_t idx = start_idx + offset;    // 计算点在投影索引向量中的位置
        frame.proj_idx[idx] = j;            // 记录点索引
        frame.proj_idx[start_idx] = offset; // 更新点数计数
    }
}

/**
 * @brief 根据像素坐标计算其在一维索引中的位置
 *
 * @param px 输入的二维像素坐标
 * @return size_t 返回该像素在一维数组中的索引
 *
 * 该函数将二维图像坐标映射到一维索引，支持列偏移和去交错功能。
 */
size_t Projector::indexFromPixel(const V2D &px) const
{
    // 计算列索引，考虑列偏移 (offset_lut_) 和图像宽度循环 (cols_)
    const int vv = (int(px(1)) + cols_ - offset_lut_[int(px(0))]) % cols_;

    // 计算一维索引，区分是否需要去交错 (destagger_)
    const size_t index = px(0) * cols_ + (destagger_ ? vv : int(px(1)));

    // 返回计算的一维索引
    return index;
};

/**
 * @brief 将三维点投影到二维图像平面上
 *
 * @param point 输入的三维点 (x, y, z)
 * @param uv 输出的二维像素坐标 (u, v)
 * @return bool 返回投影是否成功（点是否在视场范围内）
 *
 * 该函数基于球面投影模型进行投影，考虑到激光雷达的光束偏移（beam offset），
 * 并使用标定矩阵 K_ 和标定角度表 (elevation_angles_) 进行行列坐标的计算。
 */
bool Projector::projectPoint(const V3D &point, V2D &uv) const
{
    // 计算水平距离 L，减去光束偏移
    const double L = sqrt(point.x() * point.x() + point.y() * point.y()) - beam_offset_m_;

    // 计算点到原点的距离 R
    const double R = sqrt(point.z() * point.z() + L * L);

    // 计算水平角 phi 和垂直角 theta
    const double phi = atan2(point.y(), point.x());
    const double theta = asin(point.z() / R);

    // 使用标定矩阵 K_ 将 phi 转换为图像列坐标 u
    uv.x() = K_(0, 0) * phi + K_(0, 2);

    // 检查垂直角 theta 是否超出有效范围
    if (theta > elevation_angles_[0]) // 超出上边界
    {
        uv.y() = 0;   // 设置 v 为 0（最上方行）
        return false; // 返回失败
    }
    else if (theta < elevation_angles_[rows_ - 1]) // 超出下边界
    {
        uv.y() = rows_ - 1; // 设置 v 为最大行索引
        return false;       // 返回失败
    }

    // 使用标定角度表计算 v
    // 找到大于 theta 的最小角度（上边界）
    auto greater = (std::upper_bound(elevation_angles_.rbegin(), elevation_angles_.rend(), theta) + 1).base();
    // 找到小于 theta 的最大角度（下边界）
    auto smaller = greater + 1;

    if (greater == elevation_angles_.end()) // 如果 theta 超出范围
    {
        uv.y() = rows_ - 1; // 设置 v 为最大行索引
    }
    else
    {
        // 插值计算 v 的亚像素精度
        uv.y() = std::distance(elevation_angles_.begin(), greater);
        uv.y() += (*greater - theta) / (*greater - *smaller);
    }

    // 检查投影点是否在视场范围内
    return isFOV(uv);
}

bool Projector::isFOV(const V2D &uv) const
{
    return (uv.x() >= 0 && uv.x() <= cols_ - 1 && uv.y() >= 0 && uv.y() <= rows_ - 1);
}

/**
 * @brief 计算投影雅可比矩阵
 *
 * @param p 输入的三维点坐标 (x, y, z)
 * @param du_dp 输出的雅可比矩阵 (2x3)，表示二维像素坐标对三维点坐标的偏导数
 *
 * 该函数计算投影雅可比矩阵 du_dp，即二维投影坐标 (u, v) 相对于三维点位置 (x, y, z) 的偏导数。
 * 基于论文公式 8，考虑光束偏移 (beam offset) 和标定矩阵 K_ 的影响。
 */
void Projector::projectionJacobian(const V3D &p, Eigen::MatrixXd &du_dp) const
{
    // 计算水平距离 rxy（xy 平面上的模）
    double rxy = p.head<2>().norm();

    // 计算水平距离减去光束偏移后的结果 L
    double L = rxy - beam_offset_m_;

    // 计算到原点的距离平方 R^2
    double R2 = L * L + p.z() * p.z();

    // 计算 rxy 的倒数和倒数平方，用于简化后续公式
    double irxy = 1. / rxy;     // rxy 的倒数
    double irxy2 = irxy * irxy; // rxy 的倒数平方

    // 计算标定矩阵中水平投影因子的缩放值
    double fx_irxy2 = K_(0, 0) * irxy2;

    // 初始化雅可比矩阵为零矩阵
    du_dp = Eigen::MatrixXd::Zero(2, 3);

    // 根据公式填充雅可比矩阵
    // 第一行表示 u 对 (x, y, z) 的偏导数
    du_dp << -fx_irxy2 * p.y(), // du/dx
        fx_irxy2 * p.x(),       // du/dy
        0,                      // du/dz

        // 第二行表示 v 对 (x, y, z) 的偏导数
        -K_(1, 1) * p.x() * p.z() / (L * R2), // dv/dx
        -K_(1, 1) * p.y() * p.z() / (L * R2), // dv/dy
        K_(1, 1) * L / R2;                    // dv/dz
}

/**
 * @brief 将点从当前激光雷达坐标系投影到图像坐标，同时考虑失真矫正
 *
 * @param frame 包含激光雷达帧信息的结构体
 * @param p_L_k 当前激光雷达坐标系下的点坐标
 * @param p_L_i 矫正后的激光雷达坐标系下点的坐标
 * @param uv 矫正后的图像坐标
 * @param distortion_idx 输出：对应点的失真索引
 * @param round 是否对图像坐标进行取整
 * @return 是否成功投影
 *
 * 该函数根据给定的点坐标，先将其投影到图像坐标。随后通过帧信息查找对应的矫正变换，
 * 将点从当前坐标系矫正到实际采集点的坐标系，并最终完成矫正点的投影。
 */
bool Projector::projectUndistortedPoint(const LidarFrame &frame, const V3D &p_L_k, V3D &p_L_i, V2D &uv,
                                        int &distortion_idx, bool round) const
{
    // 将点投影到图像平面
    V2D uv_k;
    if (!projectPoint(p_L_k, uv_k)) // 如果点超出图像范围，投影失败
    {
        return false;
    }

    // 是否对图像坐标取整
    if (round)
    {
        uv_k(0) = std::round(uv_k(0));
        uv_k(1) = std::round(uv_k(1));
    }

    distortion_idx = -1; // 初始化失真索引
    int row = uv_k(1);
    int col = uv_k(0);

    // 计算像素位置对应的索引
    int idx = vectorIndexFromRowCol(row, col);

    // 在矫正映射中查找与该像素对应的点
    if (frame.proj_idx[idx] == 0) // 如果当前像素未匹配点
    {
        row = 0;
        while (row < frame.img_intensity.rows) // 向下查找匹配点
        {
            idx = vectorIndexFromRowCol(row, col);
            if (frame.proj_idx[idx] > 0) // 如果找到匹配点，退出循环
            {
                break;
            }
            else
            {
                ++row;
            }
        }
    }

    // 如果超出图像范围，投影失败
    if (row >= frame.img_intensity.rows)
    {
        return false;
    }

    // 如果像素对应多个点，选择与当前点最近的点
    if (frame.proj_idx[idx] > 1)
    {
        float min_dist = std::numeric_limits<float>::max();
        for (int i = 1; i <= frame.proj_idx[idx]; i++)
        {
            const int j = frame.proj_idx[idx + i];
            const V3D p_cand = frame.points_corrected->points[j].getVector3fMap().cast<double>();
            const V3D diff = p_L_k - p_cand;
            const float dist = diff.norm(); // 计算候选点与当前点的距离
            if (dist < min_dist)
            {
                min_dist = dist;
                distortion_idx = j; // 更新最近点索引
            }
        }
    }
    else
    {
        distortion_idx = frame.proj_idx[idx + 1]; // 如果只有一个点，直接获取索引
    }

    // 如果仍未找到有效索引，返回失败
    if (distortion_idx < 0)
    {
        return false;
    }

    // 查找与该像素点对应的矫正变换矩阵 T_Li_Lk
    const M4D &T_Li_Lk = frame.T_Li_Lk_vec[frame.vec_idx[distortion_idx]];

    // 将点从当前激光雷达帧的坐标系变换到目标帧坐标系
    p_L_i = T_Li_Lk.block<3, 3>(0, 0) * p_L_k + T_Li_Lk.block<3, 1>(0, 3);

    // 再次将矫正后的点投影到图像
    if (!projectPoint(p_L_i, uv))
    {
        return false;
    }

    return true;
}