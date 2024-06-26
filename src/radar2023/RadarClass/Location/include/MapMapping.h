#ifndef __MAPMAPPING_H
#define __MAPMAPPING_H

#include "../../Common/include/algorithm.h"
#include "../../Detectors/include/depthProcesser.h"

/**
 * @brief 映射类
 * 使用四点标定所得旋转平移向量进行坐标系转换
 */
class MapMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapMapping> Ptr;

    //<第二层网络分类cls,重新映射的id>
    //[B1 B2 B3 B4 B5 B7 R1 R2 R3 R4 R5 R7]
    map<int, int> _ids = {{0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}, {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}};

private:
    vector<MapLocation3D> _location3D;
    vector<MapLocation3D> cached_location3D;

    vector<Matrix<float, 4, 4>> _T = vector<Matrix<float, 4, 4>>(2);            // world2camera
    vector<Matrix<float, 4, 4>> T = vector<Matrix<float, 4, 4>>(2);             // camera2world
    vector<Matrix<float, 3, 1>> cameraPostion = vector<Matrix<float, 3, 1>>(2); // 相机位置
    // changed
    vector<Mat> rvec = vector<Mat>(2);
    vector<Mat> tvec = vector<Mat>(2);
    bool _pass_flag = false;

    int _location_pred_time[12] = {0};
    vector<vector<MapLocation3D>> _location_cache;
    vector<bboxAndRect> _IoU_pred_cache;
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    map<int, int> _DeepSort_pred_cache; // classid, trackid
#endif
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    void adjust_z_one(int camera_index, MapLocation3D &locs);
    void _location_prediction();

public:
    MapMapping();
    ~MapMapping();

    bool _is_pass();
    void push_T(int camrea_index, Mat &rvec, Mat &tvec); // add index
    void _plot_region_rect(int camera_index,vector<vector<Point3f>> &points, Mat &frame, Mat &K_0, Mat &C_0);
    vector<ArmorBoundingBox> _IoU_prediction(vector<bboxAndRect> pred, vector<DetectBox> sepboxs);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    void _DeepSort_prediction(vector<bboxAndRect> &pred, vector<DetectBox> sepboxs);
#endif
    vector<MapLocation3D> getloc();
    // void mergeUpdata(vector<bboxAndRect> &pred, vector<ArmorBoundingBox> &Ioubbox, Mat &K_0, Mat &C_0);
    void mergeUpdata(vector<bboxAndRect> &left_pred, vector<bboxAndRect> &right_pred, vector<ArmorBoundingBox> &left_Ioubbox, vector<ArmorBoundingBox> &right_Ioubbox, vector<Mat> &K_0, vector<Mat> &C_0);
};

#endif