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

    //<idx,第二层网络分类cls>
    // ？？为什么要这么分
    map<int, int> _ids = {{0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}, {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}};

private:
    vector<MapLocation3D> _location3D;
    vector<MapLocation3D> cached_location3D;

    vector<Matrix<float, 4, 4>> _T(2);            // world2camera
    vector<Matrix<float, 4, 4>> T(2);             // camera2world
    vector<Matrix<float, 3, 1>> cameraPostion(2); // 相机位置
    // changed
    vector<Mat> rvec(2);
    vector<Mat> tvec(2);
    bool _pass_flag = false;

    int _location_pred_time[12] = {0};
    vector<vector<MapLocation3D>> _location_cache;
    vector<bboxAndRect> _IoU_pred_cache;
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    map<int, int> _DeepSort_pred_cache; // classid, trackid
#endif
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    void adjust_z_one(MapLocation3D &locs);
    void _location_prediction();

public:
    MapMapping();
    ~MapMapping();

    bool _is_pass();
    void push_T(int camrea_index, Mat &rvec, Mat &tvec); // add index
    void _plot_region_rect(vector<vector<Point3f>> &points, Mat &frame, Mat &K_0, Mat &C_0);
    vector<ArmorBoundingBox> _IoU_prediction(vector<bboxAndRect> pred, vector<DetectBox> sepboxs);
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    void _DeepSort_prediction(vector<bboxAndRect> &pred, vector<DetectBox> sepboxs);
#endif
    vector<MapLocation3D> getloc();
    // void mergeUpdata(vector<bboxAndRect> &pred, vector<ArmorBoundingBox> &Ioubbox, Mat &K_0, Mat &C_0);
    void mergeUpdata(vector<bboxAndRect> &left_pred, vector<bboxAndRect> &right_pred, vector<ArmorBoundingBox> &left_Ioubbox, vector<ArmorBoundingBox> &right_Ioubbox, vector<Mat> &K_0, vector<Mat> &C_0);
};

#endif