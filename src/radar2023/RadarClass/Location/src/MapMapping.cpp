#include "../include/MapMapping.h"

MapMapping::MapMapping()
{
    MapLocation3D temp;
    vector<MapLocation3D> temp_1(this->_ids.size(), temp);
    this->_location3D.swap(temp_1);
    vector<vector<MapLocation3D>> temp_2(2, vector<MapLocation3D>(this->_ids.size(), temp));
    this->_location_cache.swap(temp_2);
}

MapMapping::~MapMapping()
{
}

bool MapMapping::_is_pass()
{
    return this->_pass_flag;
}

// 位置预测
void MapMapping::_location_prediction()
{
    // TODO：this->_location_pred_time[i]？
    for (size_t i = 0; i < this->_location3D.size(); ++i)
    {
        bool do_pre = (fabs(this->_location3D[i].x) > Epsilon &&
                       fabs(this->_location3D[i].y) > Epsilon &&
                       fabs(this->_location_cache[0][i].x) > Epsilon &&
                       fabs(this->_location_cache[0][i].y) > Epsilon &&
                       fabs(this->_location_cache[1][i].x) > Epsilon &&
                       fabs(this->_location_cache[1][i].y) > Epsilon &&
                       this->_location_pred_time[i] != 1);
        if (do_pre)
        {
            // x 方向和 y 方向上的增量：当前帧和上一帧之间的位置差异乘以预测速度比例（Pre_ratio）
            // 得到了运动的趋势（向量）
            float m_v[2] = {Pre_ratio * (this->_location_cache[1][i].x - this->_location_cache[0][i].x),
                            Pre_ratio * (this->_location_cache[1][i].y - this->_location_cache[0][i].y)};

            // 将预测位置更新为_location3D（最终发送的）
            this->_location3D[i].x = m_v[0] + this->_location_cache[1][i].x;
            this->_location3D[i].y = m_v[1] + this->_location_cache[1][i].y;
        }
        if (fabs(this->_location3D[i].x) > Epsilon && fabs(this->_location3D[i].y) > Epsilon && this->_location_pred_time[i] == 1) // CHECK：是否大于一个极小的阈值 Epsilon，并且 _location_pred_time[i] 是否等于 1
            this->_location_pred_time[i] = 0;                                                                                      // 如果满足这些条件，说明对象的位置已经被成功预测，并且当前是第一次预测到该位置。在这种情况下，将 _location_pred_time[i] 设置为 0，以表示对象位置已经成功预测

        if (do_pre && this->_location_pred_time[i] == 0) // 表示预测了一次
            this->_location_pred_time[i] = Pre_Time + 1;
        if (do_pre) // 需要进行预测，但是对象的预测时间计数器不是 0。在这种情况下，将对象的预测时间计数器减少 1，以表示经过了一次时间周期
            --this->_location_pred_time[i];
    }
    this->_location_cache[0] = this->_location_cache[1];
    this->_location_cache[1] = this->_location3D;
}

void MapMapping::_plot_region_rect(vector<vector<Point3f>> &points, Mat &frame, Mat &K_0, Mat &C_0)
{
    for (auto &it : points)
    {
        vector<Point2f> ips_pre;
        vector<Point2i> ips_dst;
        cv::projectPoints(it, this->rvec, this->tvec, K_0, C_0, ips_pre);
        circle(frame, ips_pre[0], 5, Scalar(0, 255, 0), 2);
        circle(frame, ips_pre[1], 5, Scalar(0, 255, 255), 2);
        circle(frame, ips_pre[2], 5, Scalar(255, 255, 0), 2);
        circle(frame, ips_pre[3], 5, Scalar(255, 0, 0), 2);
        line(frame, ips_pre[0], ips_pre[1], Scalar(0, 255, 0), 3);
        line(frame, ips_pre[1], ips_pre[2], Scalar(0, 255, 0), 3);
        line(frame, ips_pre[2], ips_pre[3], Scalar(0, 255, 0), 3);
        line(frame, ips_pre[3], ips_pre[0], Scalar(0, 255, 0), 3);
    }
}

vector<ArmorBoundingBox> MapMapping::_IoU_prediction(vector<bboxAndRect> pred, vector<DetectBox> sepboxs)
{
    vector<ArmorBoundingBox> pred_bbox = {};
    map<int, int>::iterator iter; // 类别 ID 的迭代器
    iter = this->_ids.begin();    // 初始化迭代器
    if (this->_IoU_pred_cache.size() > 0)
    {
        bboxAndRect cached_pred;
        while (iter != this->_ids.end()) // 遍历id:0-11
        {
            bool cache_check = false;
            bool pred_check = false;
            for (const auto &it : this->_IoU_pred_cache) // 遍历缓存的pred_armor
            {
                cache_check = it.armor.cls == iter->first ? true : false; // 是否有缓存的识别框
                if (cache_check)
                {
                    cached_pred.armor = it.armor; // 有 记录
                    break;                        // 跳出当前for
                }
            }

            for (const auto &it : pred) // 遍历当前pred_armor
            {
                pred_check = it.armor.cls != iter->first ? true : false; // 是否有当前的识别框
                if (pred_check)
                    break; // 有 跳出
            }

            if (cache_check && pred_check) // 如果2种pred_armor都有
            {
                vector<float> iou; // 计算iou (%)
                for (const auto &it : sepboxs)
                {
                    float x1 = f_max(cached_pred.armor.x0, it.x1);
                    float x2 = f_min(cached_pred.armor.x0 + cached_pred.armor.w, it.x2);
                    float y1 = f_max(cached_pred.armor.y0, it.y1);
                    float y2 = f_min(cached_pred.armor.y0 + cached_pred.armor.h, it.y2);
                    float overlap = f_max(0, x2 - x1) * f_max(0, y2 - y1);
                    float area = cached_pred.armor.w * cached_pred.armor.h;
                    iou.emplace_back(overlap / area);
                }
                if (iou.size() > 0) // 如果计算出了 IoU 值
                {
                    int max_index = std::distance(iou.begin(), max_element(iou.begin(), iou.end())); // 最大iou的id

                    // 如果最大 IoU 值大于阈值
                    if (iou[max_index] > IoU_THRE)
                    {
                        // iou的点位
                        Rect current_rect = Rect(sepboxs[max_index].x1, sepboxs[max_index].y1, sepboxs[max_index].x2 - sepboxs[max_index].x1, sepboxs[max_index].y2 - sepboxs[max_index].y1);

                        // 假设iou的1/3-width 2/5-3/5-hight是armor的位置
                        current_rect.width = floor(current_rect.width / 3.);
                        current_rect.height = floor(current_rect.height / 5.);
                        current_rect.x += current_rect.width;
                        current_rect.y += current_rect.height * 3;
                        pred_bbox.emplace_back(ArmorBoundingBox{true,
                                                                (float)current_rect.x,
                                                                (float)current_rect.y,
                                                                (float)current_rect.width,
                                                                (float)current_rect.height,
                                                                (float)iter->first});
                    }
                }
            }
            iter++;
        }
    }

    // 如果预测不为空，则更新 IoU 缓存；否则清空 IoU 缓存
    if (pred.size() > 0)
        this->_IoU_pred_cache.swap(pred);
    else
        vector<bboxAndRect>().swap(this->_IoU_pred_cache);
    return pred_bbox;
}

#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
void MapMapping::_DeepSort_prediction(vector<bboxAndRect> &pred, vector<DetectBox> sepboxs)
{
    map<int, int> _DeepSort_pred_temp;
    if (this->_DeepSort_pred_cache.size() == 0)
    {
        for (const auto &it : pred)
        {
            _DeepSort_pred_temp[it.armor.cls] = it.rect.trackID;
        }
    }
    else
    {
        for (const auto &it : pred)
        {
            _DeepSort_pred_temp[it.armor.cls] = it.rect.trackID;
        }
        map<int, int>::iterator iter;
        iter = this->_ids.begin();
        while (iter != this->_ids.end())
        {
            if (_DeepSort_pred_temp.find(iter->first) == _DeepSort_pred_temp.end())
            {
                if (this->_DeepSort_pred_cache.find(iter->first) != this->_DeepSort_pred_cache.end())
                {
                    for (auto it : sepboxs)
                    {
                        if (it.trackID == this->_DeepSort_pred_cache[iter->first])
                        {
                            _DeepSort_pred_temp[iter->first] = it.trackID;
                            pred.emplace_back(bboxAndRect{
                                ArmorBoundingBox{true,
                                                 (float)(it.x1 + (it.x2 - it.x1) / 3.),
                                                 (float)((it.y1 + ((it.y1 + it.y2) / 5.) * 3.)),
                                                 (float)((it.x2 - it.x1) / 3.),
                                                 (float)((it.y2 - it.y1) / 5.),
                                                 (float)iter->first},
                                it});
                            break;
                        }
                    }
                }
            }
        }
    }
    this->_DeepSort_pred_cache.swap(_DeepSort_pred_temp);
}
#endif

void MapMapping::push_T(int camera_index, Mat &rvec_input, Mat &tvec_input)
{
    // 互转顺序
    /*
    solvePnP【“world2camera”】
    旋转向量 rvec_Mat 表示了相机坐标系在世界坐标系中的方向。通常它还要通过罗德里格斯变换转换为旋转矩阵，描述了相机的旋转姿态。
    平移向量 tvec_Mat 表示了相机坐标系原点在世界坐标系中的位置。它描述了相机的位置。

    因此要将相机观测到的点转为世界坐标，需要得到转换矩阵(齐次)并转置/求逆
    */
    rvec_input.copyTo(this->rvec[camera_index]);
    tvec_input.copyTo(this->tvec[camera_index]);

    stringstream ss_rvec,
        ss_tvec;
    ss_rvec << "rvec= " << endl
            << " " << rvec_input << endl
            << endl;
    ss_tvec << "tvec= " << endl
            << " " << tvec_input << endl
            << endl;
    this->logger->info(ss_rvec.str());
    this->logger->info(ss_tvec.str());

    // R
    Mat rvec_Matrix;
    Rodrigues(this->rvec[camera_index], rvec_Matrix);
    Mat T_Matrix = Mat::zeros(Size(4, 4), CV_32F);
    rvec_Matrix.copyTo(T_Matrix(Rect(0, 0, 3, 3)));
    // t
    this->tvec[camera_index].copyTo(T_Matrix(Rect(3, 0, 1, 3))); // 从（3,0）开始的1列3行
    T_Matrix.at<_Float32>(Point2i(3, 3)) = 1.f;                  // 齐次
    cv2eigen(T_Matrix, this->T[camera_index]);
    this->_T[camera_index] << this->T[camera_index].inverse(); // 转换矩阵的逆
    Matrix<float, 4, 1> m1;                                    // 设以相机为原点
    m1 << 0.f, 0.f, 0.f, 1.f;
    this->cameraPostion[camera_index] << (this->_T[camera_index] * m1).topRows(3); // 通过_T转换为世界坐标的camera位置
    if (camera_index == 1)
    {
        this->_pass_flag = true;
    }
}

vector<MapLocation3D> MapMapping::getloc()
{
    return this->_location3D;
}

void MapMapping::mergeUpdata(vector<bboxAndRect> &left_pred, vector<bboxAndRect> &right_pred, vector<ArmorBoundingBox> &left_Ioubbox, vector<ArmorBoundingBox> &right_Ioubbox, vector<Mat> &K_0, vector<Mat> &C_0)
{
    if (!this->_pass_flag)
    {
        this->logger->error("Can't get _T !");
        return;
    }
    vector<MapLocation3D> temp(this->_ids.size());
    this->_location3D.swap(temp); // 3d and id

    vector<ArmorBoundingBox> left_locations;  // 左相机装甲板深度
    vector<ArmorBoundingBox> right_locations; // 右相机装甲板深度

    /*************left pred*****************/
    if (left_pred.size() > 0)
    {
        for (size_t i = 0; i < left_pred.size(); ++i)
        {
            if (left_pred[i].armor.depth != 0 && !isnan(left_pred[i].armor.depth))
            {
                pred[i].armor.flag = true;
                left_locations.emplace_back(left_pred[i].armor);
            }
            else
                left_pred[i].armor.flag = false;
        }
    }
    if (left_Ioubbox.size() > 0) // 缓存框深度
    {
        for (size_t i = 0; i < left_Ioubbox.size(); ++i)
        {
            int item_cls = int(left_Ioubbox[i].cls);

            // 查找 locations 容器中是否存在一个 ArmorBoundingBox 对象 二者cls相同
            // 若check为真 则没有相同cls
            bool check = std::find_if(left_locations.begin(), left_locations.end(), [item_cls](ArmorBoundingBox item)
                                      { return int(item.cls) == item_cls; }) == left_locations.end();
            // 如果有相同的cls 跳过 因为更信任当前识别框
            if (!check)
                continue;
            // 如果没有相同的cls 将缓存的ious当作装甲板框
            if (fabs(left_Ioubbox[i].depth) > Epsilon && !isnan(left_Ioubbox[i].depth))
            {
                left_Ioubbox[i].flag = true;
                left_locations.emplace_back(left_Ioubbox[i]);
            }
            else
                left_Ioubbox[i].flag = false;
        }
    }
    /********************end:得到left_locations**********************/
    /*************right pred*****************/
    if (right_pred.size() > 0)
    {
        for (size_t i = 0; i < right_pred.size(); ++i)
        {
            if (right_pred[i].armor.depth != 0 && !isnan(right_pred[i].armor.depth))
            {
                right_pred[i].armor.flag = true;
                right_locations.emplace_back(right_pred[i].armor);
            }
            else
                right_pred[i].armor.flag = false;
        }
    }
    if (right_Ioubbox.size() > 0) // 缓存框深度
    {
        for (size_t i = 0; i < right_Ioubbox.size(); ++i)
        {
            int item_cls = int(right_Ioubbox[i].cls);

            // 查找 locations 容器中是否存在一个 ArmorBoundingBox 对象 二者cls相同
            // 若check为真 则没有相同cls
            bool check = std::find_if(right_locations.begin(), right_locations.end(), [item_cls](ArmorBoundingBox item)
                                      { return int(item.cls) == item_cls; }) == right_locations.end();
            // 如果有相同的cls 跳过 因为更信任当前识别框
            if (!check)
                continue;
            // 如果没有相同的cls 将缓存的ious当作装甲板框
            if (fabs(right_Ioubbox[i].depth) > Epsilon && !isnan(right_Ioubbox[i].depth))
            {
                right_Ioubbox[i].flag = true;
                right_locations.emplace_back(right_Ioubbox[i]);
            }
            else
                right_Ioubbox[i].flag = false;
        }
    }
    /********************end:得到right_locations**********************/
    // Main Purpose:融合左右相机的数据
    // 参照上交 以右相机为基准做逻辑合并，而非取overlap的均值
    // TODO

    if (locations.size() > 0)
    {
        vector<MapLocation3D> pred_loc;
        vector<MapLocation3D> cache_pred;
        map<int, int>::iterator iter;
        iter = this->_ids.begin();

        while (iter != this->_ids.end()) // 遍历所有id:0~11
        {
            int key = iter->first;
            MapLocation3D al;
            bool find_id_right = false;
            for (const auto &it : right_locations) // 遍历右相机框
            {
                if ((int)it.cls == key) // 如果找到了当前id的框
                {
                    // 更新flag
                    find_id_right = true;
                    //  camera (x y z 1)
                    Matrix<float, 4, 1> xyzu;
                    vector<Point2f> center = {Point2f((it.x0 + it.w / 2.f), (it.y0 + it.h / 2.f))}; // pixel center
                    cv::undistortPoints(center, center, K_0[1], C_0[1]);                            // 去畸变,区别左右
                    xyzu << center[0].x * it.depth, center[0].y * it.depth, it.depth, 1.f;

                    // camera to world
                    Matrix<float, 4, 1> dst_xyzu;
                    dst_xyzu << this->_T[1] * xyzu; // 区别左右
                    al.id = it.cls;
                    al.x = dst_xyzu(0, 0);
                    al.y = dst_xyzu(1, 0);
                    al.z = dst_xyzu(2, 0);
                    al.flag = true;

                    if (Z_A)
                        this->adjust_z_one(al); // 处理z突变
                    break;
                }
            }
            if (!find_id_right) // 如果右相机没有找到，再找左相机
            {
                for (const auto &it : left_locations) // 遍历左相机框
                {
                    if ((int)it.cls == key) // 如果找到了当前id的框
                    {
                        //  camera (x y z 1)
                        Matrix<float, 4, 1> xyzu;
                        vector<Point2f> center = {Point2f((it.x0 + it.w / 2.f), (it.y0 + it.h / 2.f))}; // pixel center
                        cv::undistortPoints(center, center, K_0[0], C_0[0]);                            // 去畸变,区别左右
                        xyzu << center[0].x * it.depth, center[0].y * it.depth, it.depth, 1.f;

                        // camera to world
                        Matrix<float, 4, 1> dst_xyzu;
                        dst_xyzu << this->_T[0] * xyzu; // 区别左右
                        al.id = it.cls;
                        al.x = dst_xyzu(0, 0);
                        al.y = dst_xyzu(1, 0);
                        al.z = dst_xyzu(2, 0);
                        al.flag = true;

                        if (Z_A)
                            this->adjust_z_one(al); // 处理z突变
                        break;
                    }
                }
            }
            // 注：这里有这么多Z_A和类似的代码，可能是因为当初为了测试Z_A功能
            // 默认Z_A为true 参考的是上交的方法
            // 更新容器中的数据 （为了后续数据处理）
            if (Z_A)
            {
                if (al.flag)
                    cache_pred.emplace_back(al);
            }
            if (al.flag)
                pred_loc.emplace_back(al);
            ++iter;
            /*
            由    bool flag = false;
                    int id = -1;
                    float x = 0., y = 0., z = 0.;
            flag最大的作用是确定里面会不会有被处理好的3d数据
            */
        }

        // 更新:将此次id x y z作为下一次的缓存数据
        if (Z_A)
        {
            if (pred_loc.size() > 0)
            {
                this->cached_location3D.clear();
                for (auto it : pred_loc)
                {
                    this->cached_location3D.emplace_back(it);
                }
            }
        }
        for (size_t i = 0; i < pred_loc.size(); ++i)
        {
            pred_loc[i].y += Real_Size_W; // for 选手端小地图 将y变为正值
            // 这里的映射起到在最终的_location3D按顺序赋值的作用
            this->_location3D[this->_ids[(int)pred_loc[i].id]] = pred_loc[i];
            this->logger->info("LOC: [CLS] " + to_string(pred_loc[i].id) +
                               " [x] " + to_string(pred_loc[i].x) +
                               " [y] " + to_string(pred_loc[i].y) +
                               " [z] " + to_string(pred_loc[i].z));
        }
    }
    if (L_P)
        // TODO: FIX HERE【不知道这里是写完了还是没写完，可能是想根据运动模型预测3d坐标（2d……）。】
        this->_location_prediction();
}
// TODO: 待验证-深度突变
void MapMapping::adjust_z_one(MapLocation3D &loc)
{
    MapLocation3D pre_loc;
    for (const auto &it : this->cached_location3D)
    {
        if (it.id == loc.id)
            pre_loc = it;
    }
    if (!pre_loc.flag)
        return;
    if (loc.z - pre_loc.z > Z_THRE)
    {
        Matrix<float, 3, 1> line;
        line << loc.x - this->cameraPostion(0, 0), loc.y - this->cameraPostion(1, 0), loc.z - this->cameraPostion(2, 0);
        float ratio = (pre_loc.z - this->cameraPostion(2, 0)) / line(2, 0);
        loc.x = ratio * line(0, 0) + this->cameraPostion(0, 0);
        loc.y = ratio * line(1, 0) + this->cameraPostion(1, 0);
        loc.z = ratio * line(2, 0) + this->cameraPostion(2, 0);
    }
}