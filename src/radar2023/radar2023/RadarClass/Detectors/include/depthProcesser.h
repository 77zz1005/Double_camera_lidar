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
    vector<queue<Matrix<int, 2, MaxPointsNum>>> processQueue = vector<queue<Matrix<int, 2, MaxPointsNum>>>(2);
    // C++
    /*
    在类定义中，不能直接使用括号 () 初始化成员变量。应该使用等号 = 或花括号 {} 进行
    参考：https://cplusplus.com/forum/beginner/240393/
    参考：https://stackoverflow.com/questions/11490988/c-compile-time-error-expected-identifier-before-numeric-constant
    */
    vector<Matrix<float, 3, 3>> K_0 = vector<Matrix<float, 3, 3>>(2);
    vector<Matrix<float, 1, 5>> C_0 = vector<Matrix<float, 1, 5>>(2);
    vector<Matrix<float, 4, 4>> E_0 = vector<Matrix<float, 4, 4>>(2);
    vector<vector<vector<float>>> depth = vector<vector<vector<float>>>(2);

public:
    DepthQueue();
    DepthQueue(vector<Matrix<float, 3, 3>> &K_0, vector<Matrix<float, 1, 5>> &C_0, vector<Matrix<float, 4, 4>> &E_0);
    ~DepthQueue();

    //好像没用
    // int getProcessQueueSize() { return processQueue[camera_index].size(); }

    vector<vector<float>> pushback(int camera_index, pcl::PointCloud<pcl::PointXYZ> &pc);
};

#endif