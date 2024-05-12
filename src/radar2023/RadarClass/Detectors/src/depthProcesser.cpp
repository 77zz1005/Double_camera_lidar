#include "../include/depthProcesser.h"

DepthQueue::DepthQueue()
{
}

DepthQueue::DepthQueue(vector<Matrix<float, 3, 3>> &K_0, vector<Matrix<float, 1, 5>> &C_0, vector<Matrix<float, 4, 4>> &E_0)
{
    this->K_0[0] = K_0[0];
    this->C_0[0] = C_0[0];
    this->E_0[0] = E_0[0];
    this->K_0[1] = K_0[1];
    this->C_0[1] = C_0[1];
    this->E_0[1] = E_0[1];
    // 相当于Mat 先访问行，再访问列
    vector<float> tmp(ImageW, 0.);
    this->depth[0].resize(ImageH, tmp); // left
    this->depth[1].resize(ImageH, tmp); // right
}

DepthQueue::~DepthQueue()
{
}

// TODO:制作2张深度图
vector<vector<float>> DepthQueue::pushback(int camera_index, pcl::PointCloud<pcl::PointXYZ> &pc)
{
    if (this->processQueue.empty())
    {
        this->_initflag = true;
    }
    // share
    Matrix4Xf pc_Matrix = pc.getMatrixXfMap();
    int cols = pc_Matrix.cols(); // 点云数据的数量
    // lidar to camera
    Matrix3Xf transformed_points = (this->E_0[camera_index] * pc_Matrix).topRows(3);
    Matrix<float, 3, MaxPointsNum> pointsBox;
    Matrix<float, 1, MaxPointsNum> dptBox;
    Matrix<int, 2, MaxPointsNum> ipBox;
    pointsBox.leftCols(cols) << transformed_points;                                                                              // 完整数据
    dptBox.leftCols(cols) << transformed_points.row(2);                                                                          // 深度z
    ipBox << ((this->K_0[camera_index] * pointsBox).array().rowwise() * (pointsBox.row(2).array().inverse())).topRows(2).matrix().cast<int>(); // 像素坐标
    // CHECK:越界则将像素坐标x,y置0
    /*
    注：点云转为像素坐标 越界的会变为负数/大于相机分辨率 因此深度图与相机图像的原点是等价的
    */
    auto inside_x = (ipBox.row(0).array() >= 0 && ipBox.row(0).array() < ImageW);
    auto inside_y = (ipBox.row(1).array() >= 0 && ipBox.row(1).array() < ImageH);
    ipBox.row(0) << (inside_x).select(ipBox.row(0), MatrixXf::Constant(1, MaxPointsNum, 0));
    ipBox.row(1) << (inside_y).select(ipBox.row(1), MatrixXf::Constant(1, MaxPointsNum, 0));

    this->processQueue[camera_index].push(ipBox); // 加入优先队列

    // 更新数据
    if (this->processQueue[camera_index].size() > maxQueueSize)
    {
        Matrix<int, 2, MaxPointsNum> outpoints = this->processQueue[camera_index].front(); // 获取队首元素【old】
        // 重置为0
        for (int i = 0; i < MaxPointsNum; ++i)
        {
            if (this->depth[camera_index][outpoints(1, i)][outpoints(0, i)] == 0.)
                continue;
            this->depth[camera_index][outpoints(1, i)][outpoints(0, i)] = 0.;
        }
        // 删除
        this->processQueue[camera_index].pop();
    }
    // 更新深度信息，确保有效、“最近”
    for (int i = 0; i < MaxPointsNum; ++i)
    {
        if (dptBox(0, i) > 0) // CHECK
        {
            if ((fabs(this->depth[camera_index][ipBox(1, i)][ipBox(0, i)]) > Epsilon && dptBox(0, i) < this->depth[camera_index][ipBox(1, i)][ipBox(0, i)]) || fabs(this->depth[camera_index][ipBox(1, i)][ipBox(0, i)]) < Epsilon)
                this->depth[camera_index][ipBox(1, i)][ipBox(0, i)] = dptBox(0, i); // 深度
        }
        else
            break;
    }
    return this->depth[camera_index];
}