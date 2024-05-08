#ifndef __DEPTHPROCESSER_H
#define __DEPTHPROCESSER_H

#include "../../Common/include/public.h"

/**
 * @brief 深度信息处理类
 * 处理雷达点云信息为相机对应的深度图
 */
class DepthQueue
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<DepthQueue> Ptr;

private:
    bool _initflag = false;
    vector<queue<Matrix<int, 2, MaxPointsNum>>> processQueue(2);

    vector<Matrix<float, 3, 3>> K_0(2);
    vector<Matrix<float, 1, 5>> C_0(2);
    vector<Matrix<float, 4, 4>> E_0(2);
    vector<vector<vector<float>>> depth(2);

public:
    DepthQueue();
    DepthQueue(vector<Matrix<float, 3, 3>> &K_0, vector<Matrix<float, 1, 5>> &C_0, vector<Matrix<float, 4, 4>> &E_0);
    ~DepthQueue();

    //好像没用
    // int getProcessQueueSize() { return processQueue[camera_index].size(); }

    vector<vector<float>> pushback(int camera_index, pcl::PointCloud<pcl::PointXYZ> &pc);
};

#endif