#include "../include/Radar.h"
void Radar::armor_filter(vector<bboxAndRect> &pred)
{
    vector<bboxAndRect> results;
    for (int i = 0; i < int(this->mapMapping->_ids.size()); ++i)
    {
        int max_id = 0;
        float max_conf = 0.f;
        for (size_t j = 0; j < pred.size(); ++j)
        {
            // 对于每个目标，如果其类别与当前ID匹配，并且其置信度高于之前找到的最高置信度，就更新最高置信度和对应的目标索引
            if ((int)pred[j].armor.cls == this->ids[i] && pred[j].armor.conf - max_conf > Epsilon)
            {
                max_id = j;                    // 记录拥有max 置信度的bboxAndRect在pred中的结果
                max_conf = pred[j].armor.conf; // 记录当前最大置信度
            }
        }
        // CHECK:max_conf 是否大于一个非常小的阈值 Epsilon,可以过滤掉置信度较低的预测框，只保留置信度较高的目标
        if (fabs(max_conf) > Epsilon)
            results.emplace_back(pred[max_id]);
    }
    pred.swap(results); // 最终结果
}
// 装甲板深度
void Radar::detectDepth(int camera_index, vector<bboxAndRect> &pred)
{
    if (pred.size() == 0)
        return;
    for (size_t i = 0; i < pred.size(); ++i)
    {
        // CHANGE:由于在reBuild中已经根据边界调整了
        if (pred[i].armor.x0 > ImageW || pred[i].armor.y0 > ImageH || pred[i].armor.w * pred[i].armor.h <= 0)
            continue;
        vector<float> tempBox; // 储存所有深度
        float center[2] = {pred[i].armor.x0 + pred[i].armor.w / 2.f, pred[i].armor.y0 + pred[i].armor.h / 2.f};
        for (int j = int(max<float>(center[1] - pred[i].armor.h / 2.f, 0.)); j < int(min<float>(center[1] + pred[i].armor.h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - pred[i].armor.w / 2.f, 0.)); k < int(min<float>(center[0] + pred[i].armor.w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[camera_index][j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[camera_index][j][k]);
            }
        }
        float tempDepth = 0;
        for (const auto &jt : tempBox)
        {
            tempDepth += jt;
        }
        pred[i].armor.depth = tempBox.size() != 0 ? tempDepth / tempBox.size() : 0.;
        this->logger->info("Depth: [CLS] " + to_string(pred[i].armor.cls) + " [Depth] " + to_string(pred[i].armor.depth));
    }
}
// Iou深度
void Radar::detectDepth(int camera_index, vector<ArmorBoundingBox> &armors)
{
    if (armors.size() == 0)
        return;
    for (size_t i = 0; i < armors.size(); ++i)
    {
        // CHANGE:由于在reBuild中已经根据边界调整了
        if (armors[i].x0 > ImageW || armors[i].y0 > ImageH || armors[i].w * armors[i].h <= 0)
            continue;
        int count = 0;
        vector<float> tempBox;
        float center[2] = {armors[i].x0 + armors[i].w / 2.f, armors[i].y0 + armors[i].h / 2.f};
        for (int j = int(max<float>(center[1] - armors[i].h / 2.f, 0.)); j < int(min<float>(center[1] + armors[i].h / 2.f, ImageH)); ++j)
        {
            for (int k = int(max<float>(center[0] - armors[i].w / 2.f, 0.)); k < int(min<float>(center[0] + armors[i].w / 2.f, ImageW)); ++k)
            {
                if (this->publicDepth[camera_index][j][k] == 0)
                    continue;
                tempBox.emplace_back(this->publicDepth[camera_index][j][k]);
                ++count;
            }
        }
        int tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        armors[i].depth = count != 0 ? (float)tempNum / (float)count : 0.;
        this->logger->info("Depth: [CLS] " + to_string(armors[i].cls) + " [Depth] " + to_string(armors[i].depth));
    }
}

void Radar::send_judge(judge_message &message)
{
    vector<vector<float>> loc;
    switch (message.task)
    {
    case 1:
        for (int i = 0; i < int(message.loc.size() / 2); ++i)
        {
            vector<float> temp_location;
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].x);
            temp_location.emplace_back(message.loc[i + this->ENEMY * 6].y);
            loc.emplace_back(temp_location);
        }
        this->myUART->myUARTPasser.push_loc(loc);
        break;

    default:
        break;
    }
}

void Radar::drawBbox(vector<DetectBox> &bboxs, Mat &img)
{
    for (DetectBox &it : bboxs)
    {
        cv::rectangle(img, Rect(it.x1, it.y1, it.x2 - it.x1, it.y2 - it.y1), Scalar(0, 255, 0), 2);
    }
}

void Radar::drawArmorsForDebug(vector<ArmorBoundingBox> &armors, Mat &img)
{
    for (auto &it : armors)
    {
        Rect temp = Rect(it.x0, it.y0, it.w, it.h);
        cv::rectangle(img, temp, Scalar(255, 255, 0), 2);
        stringstream ss;
        ss << it.cls << "[Depth]" << it.depth << "[Conf]" << it.conf;
        cv::putText(img, ss.str(), Point2i(int(it.x0 + it.w / 2), int(it.y0 + it.h / 2)), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255));
    }
}

void Radar::drawArmorsForDebug(vector<bboxAndRect> &armors, Mat &img)
{
    for (auto &it : armors)
    {
        Rect temp = Rect(it.armor.x0, it.armor.y0, it.armor.w, it.armor.h);
        cv::rectangle(img, temp, Scalar(0, 255, 0), 2);
        cv::putText(img, to_string(int(it.armor.cls)), cv::Point2i(temp.x, temp.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
    }
}

Radar::Radar()
{
    // add
    this->myFrames[0].setDepth(FRAME_DEPTH);
    this->myFrames[1].setDepth(FRAME_DEPTH);
}

Radar::~Radar()
{
    if (this->is_alive)
        this->stop();
    this->logger->flush();
}

void Radar::init()
{
    assert(this->nh);
    assert(!share_path.empty());
    if (this->_init_flag)
        return;
    this->logger->info("Initing ...Process");
    if (ENEMY)
        this->logger->critical("YOU ARE RED");
    else
        this->logger->critical("YOU ARE BLUE");

    // changed:双相机话题
    this->GUI_left_image_pub_ = this->image_transport->advertise("/radar2023/left_result_view", 1, true);
    this->GUI_right_image_pub_ = this->image_transport->advertise("/radar2023/right_result_view", 1, true);
    this->pub_locations = this->nh->advertise<radar2023::Locations>("/radar2023/locations", 1, true); // share
    std::string param_name;
    if (this->nh->searchParam("/radar2023/EnemyType", param_name))
    {
        this->nh->getParam(param_name, this->ENEMY);
    }
    else
    {
        ROS_WARN("Parameter EnemyType not defined");
    }

    //// 双相机外参
    if (this->nh->searchParam("/radar2023/CameraParam0", param_name))
    {
        this->nh->getParam(param_name, this->CAMERA_PARAM_0);
    }
    else
    {
        ROS_WARN("Parameter CameraParam_0 not defined");
    }
    if (this->nh->searchParam("/radar2023/CameraParam1", param_name))
    {
        this->nh->getParam(param_name, this->CAMERA_PARAM_1);
    }
    else
    {
        ROS_WARN("Parameter CameraParam_1 not defined");
    }
    ////
    if (this->nh->searchParam("/radar2023/AuthPassword", param_name))
    {
        this->nh->getParam(param_name, this->PASSWORD);
    }
    else
    {
        ROS_WARN("Parameter AuthPassword not defined");
    }
    if (this->nh->searchParam("/radar2023/EngineForArmor", param_name))
    {
        this->nh->getParam(param_name, this->EngineForArmor);
    }
    else
    {
        ROS_WARN("Parameter EngineForArmor not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForArmor", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForArmor);
    }
    else
    {
        ROS_WARN("Parameter OnnxForArmor not defined");
    }
    //// 双相机图像参数
    if (this->nh->searchParam("/radar2023/CameraConfig0", param_name))
    {
        this->nh->getParam(param_name, this->CameraConfig_0);
    }
    else
    {
        ROS_WARN("Parameter CameraConfig_0 not defined");
    }
    if (this->nh->searchParam("/radar2023/CameraConfig1", param_name))
    {
        this->nh->getParam(param_name, this->CameraConfig_1);
    }
    else
    {
        ROS_WARN("Parameter CameraConfig_1 not defined");
    }
    ////

    if (this->nh->searchParam("/radar2023/SerialPortName", param_name))
    {
        this->nh->getParam(param_name, this->SerialPortName);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }

// remain:测试视频路径仅1个
#ifdef UsingVideo
    if (this->nh->searchParam("/radar2023/TestVideo", param_name))
    {
        this->nh->getParam(param_name, this->TestVideo);
    }
    else
    {
        ROS_WARN("Parameter TestVideo not defined");
    }
#endif
#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)

#endif
#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (this->nh->searchParam("/radar2023/EngineForCar", param_name))
    {
        this->nh->getParam(param_name, this->EngineForCar);
    }
    else
    {
        ROS_WARN("Parameter EngineForCar not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForCar", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForCar);
    }
    else
    {
        ROS_WARN("Parameter OnnxForCar not defined");
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    if (this->nh->searchParam("/radar2023/EngineForSort", param_name))
    {
        this->nh->getParam(param_name, this->EngineForSort);
    }
    else
    {
        ROS_WARN("Parameter EngineForSort not defined");
    }
    if (this->nh->searchParam("/radar2023/OnnxForSort", param_name))
    {
        this->nh->getParam(param_name, this->OnnxForSort);
    }
    else
    {
        ROS_WARN("Parameter OnnxForSort not defined");
    }
#endif
    vector<Matrix<float, 3, 3>> K_0(2);
    vector<Matrix<float, 1, 5>> C_0(2);
    vector<Matrix<float, 4, 4>> E_0(2);
    if (!read_param(this->K_0_Mat[0], this->C_0_Mat[0], this->E_0_Mat[0], this->share_path + "/params/" + this->CAMERA_PARAM_0))
    {
        this->logger->error("Can't read CAMERA_PARAM_0: {}!", this->share_path + "/params/" + this->CAMERA_PARAM_0);
        return;
    }
    if (!read_param(this->K_0_Mat[1], this->C_0_Mat[1], this->E_0_Mat[1], this->share_path + "/params/" + this->CAMERA_PARAM_1))
    {
        this->logger->error("Can't read CAMERA_PARAM_1: {}!", this->share_path + "/params/" + this->CAMERA_PARAM_1);
        return;
    }
    // cv::mat to eigen::Matrix
    cv2eigen(this->K_0_Mat[0], K_0[0]);
    cv2eigen(this->C_0_Mat[0], C_0[0]);
    cv2eigen(this->E_0_Mat[0], E_0[0]);
    cv2eigen(this->K_0_Mat[1], K_0[1]);
    cv2eigen(this->C_0_Mat[1], C_0[1]);
    cv2eigen(this->E_0_Mat[1], E_0[1]);

    this->depthQueue = std::make_shared<DepthQueue>(K_0, C_0, E_0); // changed: depth[0] depth[1]
    this->myUART = std::make_shared<UART>(this->ENEMY);
    this->myLocation = std::make_shared<Location>();         // empty
    this->videoRecorder = std::make_shared<VideoRecorder>(); // empty
    this->mySerial = std::make_shared<MySerial>();           // empty // TODO:自主决策
    this->mapMapping = std::make_shared<MapMapping>();       // TODO:在map上2d预测位置
    this->myLocation->decodeMapPoints(this->share_path + "/params/MapMappingPoints.json");

// 相机线程
#ifdef UsingVideo
    // NOTICE:this->CameraConfig_0
    this->cameraThread = std::make_shared<CameraThread>((this->share_path + "/params/" + this->CameraConfig_0), (this->share_path + "/resources/" + this->TestVideo));
#else
    this->cameraThread = std::make_shared<CameraThread>((this->share_path + "/params/" + this->CameraConfig_0), (this->share_path + "/params/" + this->CameraConfig_1)); // change inside
#endif

    // TODO:GUI
    this->nh->setParam("/radar2023/ExitProgram", false);
    // this->nh->setParam("/radar2023/Recorder", true);
    this->nh->setParam("/radar2023/Recorder", false);

    // Lidar
    this->LidarListenerBegin();

    // 加载模型
    this->armorDetector = std::make_shared<ArmorDetector>(this->share_path + "/models/" + this->EngineForArmor,
                                                          this->share_path + "/models/" + this->OnnxForArmor);
    this->armorDetector->accessModelTest();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector = std::make_shared<CarDetector>(this->share_path + "/models/" + this->EngineForCar,
                                                      this->share_path + "/models/" + this->OnnxForCar);
    this->carDetector->accessModelTest();
#endif

    if (!this->armorDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    if (!this->carDetector->initModel())
    {
        this->stop();
        this->logger->flush();
        return;
    }
#endif
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
    this->dsTracker = std::make_shared<DsTracker>(this->share_path + "/models/" + this->OnnxForSort,
                                                  this->share_path + "/models/" + this->EngineForSort);
#endif
#ifdef ExperimentalOutput
    this->myExpLog = std::make_shared<ExpLog>();
    this->myExpLog->init(this->share_path + "/ExpResultDir/");
#endif

    // changed: 初始化视频录制器/Record/left  /Record/right
    this->_recorder_block = (!this->videoRecorder->init(0, (this->share_path + "/Record/" + "left/").c_str(), VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH))) && (!this->videoRecorder->init(1, (this->share_path + "/Record/" + "right/").c_str(), VideoWriter::fourcc('m', 'p', '4', 'v'), Size(ImageW, ImageH)));

    this->cameraThread->start(); // changed: Double camera
    this->_init_flag = true;
    this->logger->info("Init Done");
    this->is_alive = true;
}

void Radar::setRosNodeHandle(ros::NodeHandle &nh)
{
    this->nh = std::make_shared<ros::NodeHandle>(nh);
}

void Radar::setRosImageTransport(image_transport::ImageTransport &image_transport)
{
    this->image_transport = std::make_shared<image_transport::ImageTransport>(image_transport);
}

void Radar::setRosPackageSharedPath(String &path)
{
    this->share_path = path;
}

void Radar::LidarListenerBegin()
{
    assert(this->nh);
    if (this->_is_LidarInited)
        return;
    this->sub_lidar = this->nh->subscribe(lidarTopicName, LidarQueueSize, &Radar::LidarCallBack, this);
    this->_is_LidarInited = true;
    this->logger->info("Lidar inited");
}

void Radar::RosSpinLoop()
{
    while (this->__RosSpinLoop_working)
    {
        if (ros::ok())
            ros::spinOnce();
    }
    ros::shutdown();
    this->logger->critical("RosSpinLoop Exit");
}

//  changed:publicDepth[0] publicDepth[1]
void Radar::LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    std::vector<std::vector<float>> left_tempDepth = this->depthQueue->pushback(0, *pc); // change:pushback
    std::vector<std::vector<float>> right_tempDepth = this->depthQueue->pushback(1, *pc);
    unique_lock<shared_timed_mutex> ulk(this->myMutex_publicDepth);
    this->publicDepth[0].swap(left_tempDepth);
    this->publicDepth[1].swap(right_tempDepth);
    ulk.unlock();
}

void Radar::SerReadLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->read(this->mySerial);
    }
    this->logger->critical("SerReadLoop Exit");
}

void Radar::SerWriteLoop()
{
    while (this->_Ser_working)
    {
        this->myUART->write(this->mySerial);
    }
    this->logger->critical("SerWriteLoop Exit");
}

void Radar::MainProcessLoop()
{
    // TODO:可以在哪里把>_cap->Index_Camera_直接传给线程类，然后修改所有访问这个索引的地方
    int left_idx = this->cameraThread->_cap->Index_Camera_[0];
    int right_idx = this->cameraThread->_cap->Index_Camera_[1];

    while (this->__MainProcessLoop_working)
    {
        auto start_t = std::chrono::system_clock::now().time_since_epoch(); // start time
        // CHECK：若发生错误关闭，尝试再次通过相机线程开启抓图
        if (!this->cameraThread->is_open())
        {
            this->cameraThread->open(); // share 2
            continue;
        }

        // changed
        FrameBag frameBag_0 = this->cameraThread->read(left_idx);
        FrameBag frameBag_1 = this->cameraThread->read(right_idx);
        if (frameBag_0.flag && frameBag_1.flag)
        {
            if (this->_if_record) // 为了不影响录制结果
            {
                // TODO：修改所有录制相关。这里先改成2次调用
                Mat left_record_frame = frameBag_0.frame.clone();
                Mat right_record_frame = frameBag_1.frame.clone();
                this->myFrames[left_idx].push(left_record_frame);
                this->myFrames[right_idx].push(right_record_frame);
            }

// changed: 先只考虑修改双神经网络的代码
#ifndef UseOneLayerInfer
#ifdef UsePointCloudSepTarget
            shared_lock<shared_timed_mutex> slk_md(this->myMutex_publicDepth);
            vector<Rect> sepTargets = this->movementDetector->applyMovementDetector(this->publicDepth);
            slk_md.unlock();
            vector<bboxAndRect> pred = this->movementDetector->_ifHistoryBuild() ? this->armorDetector.infer(frameBag.frame, sepTargets) : {};
#else
            vector<DetectBox> left_sepTargets = this->carDetector->infer(frameBag_0.frame);  // 兵种yolov5
            vector<DetectBox> right_sepTargets = this->carDetector->infer(frameBag_1.frame); // 兵种yolov5
                                                                                             // 注：classId不是真实id
#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
            this->dsTracker->sort(frameBag.frame, sepTargets);
#endif
            vector<bboxAndRect> left_pred = this->armorDetector->infer(frameBag_0.frame, left_sepTargets);   // 装甲板yolov5
            vector<bboxAndRect> right_pred = this->armorDetector->infer(frameBag_1.frame, right_sepTargets); // 装甲板yolov5

#endif
#else
            vector<bboxAndRect> pred = this->armorDetector->infer(frameBag.frame);
#endif
// 正式比赛时可关闭：from config.h
// TODO:显示2幅frame
#if defined Test && defined TestWithVis
#ifndef UseOneLayerInfer
            this->drawBbox(left_sepTargets, frameBag_0.frame);
            this->drawBbox(right_sepTargets, frameBag_1.frame);

#endif
            this->drawArmorsForDebug(left_pred, frameBag_0.frame);
            this->drawArmorsForDebug(right_pred, frameBag_1.frame);
#endif
#ifdef ExperimentalOutput
            int pred_size = pred.size();
            float pred_conf_average = sumConfAverage(pred);
#endif
#ifdef Test
#ifdef ShowDepth
            if (this->_if_coverDepth)
            {
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth);
                for (int i = 0; i < ImageH; ++i)
                {
                    uchar *data = frameBag.frame.ptr<uchar>(i);
                    int k = 0;
                    for (int j = 0; j < ImageW; ++j)
                    {
                        if (fabs(this->publicDepth[i][j]) > Epsilon)
                        {
                            int b, g, r;
                            hsv_to_bgr((this->publicDepth[i][j] / 100.) * 255, 255, 255, b, g, r);
                            data[k] = b;
                            data[k + 1] = g;
                            data[k + 2] = r;
                        }
                        k += 3;
                    }
                }
                slk_pd.unlock();
            }
#endif
#endif
            if (left_pred.size() != 0 || right_pred.size() != 0)
            {
                // 保留最大置信度最大的装甲板
                if (left_pred.size() != 0)
                {
                    this->armor_filter(left_pred);
                }
                if (right_pred.size() != 0)
                {
                    this->armor_filter(right_pred);
                }
                shared_lock<shared_timed_mutex> slk_pd(this->myMutex_publicDepth); // 上锁
                // 获取深度
                if ((this->publicDepth[left_idx].size() == 0) && (this->publicDepth[right_idx].size() == 0))
                {
                    slk_pd.unlock();
                    this->logger->info("No Lidar Msg , Return");
                }
                else
                {
                    this->detectDepth(left_idx, left_pred);
                    this->detectDepth(right_idx, right_pred);

#if defined UseDeepSort && !(defined UsePointCloudSepTarget)
                    this->mapMapping._DeepSort_prediction(pred, sepTargets);
#endif
#ifndef UseOneLayerInfer
                    vector<ArmorBoundingBox> left_IouArmors = this->mapMapping->_IoU_prediction(left_pred, left_sepTargets); // 预测 防遮挡
                    vector<ArmorBoundingBox> right_IouArmors = this->mapMapping->_IoU_prediction(right_pred, right_sepTargets);
#else
                    vector<ArmorBoundingBox>
                        IouArmors = {};
#endif
                    // 获取缓存框深度
                    this->detectDepth(left_idx, left_IouArmors);
                    this->detectDepth(right_idx, right_IouArmors);
                    slk_pd.unlock();

                    // changed
                    // this->mapMapping->mergeUpdata(pred, IouArmors, this->K_0_Mat, this->C_0_Mat);
                    this->mapMapping->mergeUpdata(left_pred, right_pred, left_IouArmors, right_IouArmors, this->K_0_Mat, this->C_0_Mat);
                    /*由this->_location3D[this->_ids[(int)pred_loc[i].id]] = pred_loc[i];
                    可以得到按某种顺序排列的3d坐标（正y）
                    */

                    judge_message myJudge_message;
                    myJudge_message.task = 1; // TODO:不同任务
                    myJudge_message.loc = this->mapMapping->getloc();
                    this->send_judge(myJudge_message);
                    if (myJudge_message.loc.size() > 0)
                    {
                        radar2023::Locations locations_msg;
                        for (int i = 0, N = myJudge_message.loc.size(); i < N; ++i)
                        {
                            radar2023::Location location_msg;
                            location_msg.id = myJudge_message.loc[i].id;
                            location_msg.x = myJudge_message.loc[i].x;
                            location_msg.y = myJudge_message.loc[i].y;
                            locations_msg.locations.emplace_back(location_msg);
                        }
                        locations_msg.header = std_msgs::Header();
                        locations_msg.header.frame_id = "radar2023";
                        locations_msg.header.stamp = ros::Time::now();
                        locations_msg.header.seq = 1;
                        this->pub_locations.publish(locations_msg);
                    }
                }
            }
            auto end_t = std::chrono::system_clock::now().time_since_epoch();

#ifdef Test
            char ch[255];
            sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
            std::string fps_str = ch; // to 实时显示程序运行时间
            // add
            cv::putText(frameBag_0.frame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 0}, 3);
            cv::putText(frameBag_1.frame, fps_str, {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 2, {0, 255, 0}, 3);

            // TODO:GUI-这是什么？
            this->mapMapping->_plot_region_rect(this->show_region, frameBag_0.frame, this->K_0_Mat[0], this->C_0_Mat[0]); // left
            this->mapMapping->_plot_region_rect(this->show_region, frameBag_1.frame, this->K_0_Mat[1], this->C_0_Mat[1]); // right

#endif
            // changed: 2 Publisher
            sensor_msgs::ImagePtr left_image_msg;
            try
            {
                left_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBag_0.frame).toImageMsg();
                left_image_msg->header.frame_id = "radar2023";
                left_image_msg->header.stamp = ros::Time::now();
                left_image_msg->header.seq = 1;
                this->GUI_left_image_pub_.publish(left_image_msg);
            }
            catch (cv_bridge::Exception &e)
            {
                this->logger->error("cv_bridge exception: %s", e.what());
                this->logger->flush();
                continue;
            }
            this->logger->flush();
            // add
            sensor_msgs::ImagePtr right_image_msg;
            try
            {
                right_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBag_1.frame).toImageMsg();
                right_image_msg->header.frame_id = "radar2023";
                right_image_msg->header.stamp = ros::Time::now();
                right_image_msg->header.seq = 1;
                this->GUI_right_image_pub_.publish(right_image_msg);
            }
            catch (cv_bridge::Exception &e)
            {
                this->logger->error("cv_bridge exception: %s", e.what());
                this->logger->flush();
                continue;
            }
            this->logger->flush();

#ifdef ExperimentalOutput
            std::vector<string> msg;
            msg.emplace_back(to_string(pred.size()));
            msg.emplace_back(to_string(pred_size));
            msg.emplace_back(to_string(sumConfAverage(pred)));
            msg.emplace_back(to_string(pred_conf_average));
            msg.emplace_back(to_string((end_t - start_t).count()));
            this->myExpLog->input(msg);
#endif
        }
        else
            continue;
    }
    this->logger->critical("MainProcessLoop Exit");
}

void Radar::VideoRecorderLoop()
{
    while (this->__VideoRecorderLoop_working)
    {
        if (this->_if_record && this->myFrames[0].size() > 0 && this->myFrames[1].size() > 0 && !this->_recorder_block)
        {
            this->videoRecorder->write(0, this->myFrames[0].front());
            this->videoRecorder->write(1, this->myFrames[1].front());

            this->myFrames[0].pop();
            this->myFrames[1].pop();
        }
        else if (!this->_if_record)
        {
            sleep(1);
        }
    }
    this->videoRecorder->close();
    this->_if_record = false;
    this->nh->setParam("/radar2023/Recorder", false);
    this->logger->critical("VideoRecorderLoop Exit");
}

void Radar::spin()
{
    // TODO: 增加更多标志位来扩展流程控制
    this->init(); // main change: Camera

    // CHECK: init result
    assert(depthQueue && armorDetector);
#if !(defined UseOneLayerInfer)
#ifdef UsePointCloudSepTarget
    assert(movementDetector);
#else
    assert(carDetector);
#endif
#endif

    assert(cameraThread && myLocation && mapMapping && myUART && mySerial && videoRecorder);

#ifdef ExperimentalOutput
    assert(myExpLog);
#endif

    if (!this->_init_flag)
        return;
    std::string param_name;

    // TODO:GUI:获取GUI的滑动条参数？
    if (this->nh->searchParam("/radar2023/ExitProgram", param_name))
    {
        this->nh->getParam(param_name, this->ExitProgramSiginal);
    }
    else
    {
        ROS_WARN("Parameter ExitProgram not defined");
    }
    if (this->nh->searchParam("/radar2023/Recorder", param_name))
    {
        this->nh->getParam(param_name, this->VideoRecorderSiginal);
    }
    else
    {
        ROS_WARN("Parameter Recorder not defined");
    }

#ifdef ShowDepth
    if (this->nh->searchParam("/gui/CoverDepth", param_name))
    {
        this->nh->getParam(param_name, this->_if_coverDepth);
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
#endif

    if (this->ExitProgramSiginal == true)
    {
        this->stop();
        return;
    }
    this->_if_record = VideoRecorderSiginal;

    int left_idx = this->cameraThread->_cap->Index_Camera_[0];
    int right_idx = this->cameraThread->_cap->Index_Camera_[1];

    // TODO:pnp标定
    if (!this->mapMapping->_is_pass())
    {
        this->logger->info("Locate pick start ...Process");

        vector<Mat> rvec(2);
        vector<Mat> tvec(2);

        unique_lock<shared_timed_mutex> ulk(myMutex_cameraThread);
        try
        {
            // 互斥锁
            // unique_lock 对象 ulk 锁住了 myMutex_cameraThread，从而确保在 try 块中的代码执行期间，只有一个线程可以访问由该互斥量保护的资源

            // Mat:不用显示&来修改
            bool pick_left = this->myLocation->locate_pick(this->cameraThread, left_idx, this->ENEMY, rvec[left_idx], tvec[left_idx], this->K_0_Mat[left_idx], this->C_0_Mat[left_idx], this->E_0_Mat[left_idx]);
            bool pick_right = this->myLocation->locate_pick(this->cameraThread, right_idx, this->ENEMY, rvec[right_idx], tvec[right_idx], this->K_0_Mat[right_idx], this->C_0_Mat[right_idx], this->E_0_Mat[right_idx]);
            if (!pick_left && !pick_left)
            {
                std::cout << "PICK PNP POINTS FAILURE!" << endl; // debug
                ulk.unlock();
                return;
            }
            std::cout << "PICK PNP POINTS SUCCESSFULLY" << endl;
            ulk.unlock();
        }
        catch (const std::exception &e)
        {
            this->logger->error(e.what());
            return;
        }
        // camera_position and Transform matrix
        this->mapMapping->push_T(left_idx, rvec[left_idx], tvec[left_idx]);
        this->mapMapping->push_T(right_idx, rvec[right_idx], tvec[right_idx]);
        this->logger->info("Locate pick Done");
    }

    // 启动线程
    if (!this->_thread_working && this->is_alive)
    {
        this->logger->info("Thread starting ...Process");
        this->_thread_working = true;
        if (!this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = true;
            this->rosSpinLoop = thread(std::bind(&Radar::RosSpinLoop, this)); // 控制ros消息处理 同步
        }
        if (!this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = true;
            this->processLoop = thread(std::bind(&Radar::MainProcessLoop, this)); // TODO
        }
        if (!this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = true;
            this->videoRecoderLoop = thread(std::bind(&Radar::VideoRecorderLoop, this)); // TODO：录制线程
        }
        this->logger->info("Thread starting ...Done");
    }
    if (!this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("Serial initing ...Process");
        this->mySerial->initSerial(this->SerialPortName, this->PASSWORD); // 串口初始化
        this->logger->info("Serial initing ...Done");
    }
    if (!this->_Ser_working && this->mySerial->_is_open() && this->is_alive)
    {
        this->logger->info("SerThread initing ...Process");
        this->_Ser_working = true;
        this->serRead = thread(std::bind(&Radar::SerReadLoop, this)); // 串口通信(Y坐标最终为正)
        this->serWrite = thread(std::bind(&Radar::SerWriteLoop, this));
        this->logger->info("SerThread initing ...Done");
    }
    if ((myFrames[0].size() > 0) || (myFrames[1].size() > 0) && this->is_alive && !this->_if_record)
    {
        myFrames[0].pop(); // TODO
        myFrames[1].pop();
        // C++
        // 队列（例如myFrames）是先进先出（FIFO）的数据结构，因此从队列中弹出的元素是最早进入队列的元素，也就是队列的前面元素
    }
}

void Radar::stop()
{
    this->is_alive = false;
    this->logger->warn("Start Shutdown Process...");
    this->logger->flush();
    if (this->cameraThread->is_open())
        this->cameraThread->stop();
    if (this->_thread_working)
    {
        this->_thread_working = false;
        if (this->__RosSpinLoop_working)
        {
            this->__RosSpinLoop_working = false;
            this->rosSpinLoop.join();
        }
        if (this->__MainProcessLoop_working)
        {
            this->__MainProcessLoop_working = false;
            this->processLoop.join();
        }
        if (this->__VideoRecorderLoop_working)
        {
            this->__VideoRecorderLoop_working = false;
            this->videoRecoderLoop.join();
        }
        if (this->_Ser_working)
        {
            this->_Ser_working = false;
            this->serRead.join();
            this->serWrite.join();
        }
    }
    this->armorDetector->unInit();

#if !(defined UsePointCloudSepTarget || defined UseOneLayerInfer)
    this->carDetector->unInit();
#endif

#ifdef ExperimentalOutput
    this->myExpLog->uninit();
#endif

    ros::shutdown();
    this->logger->warn("Program Shutdown");
}

bool Radar::alive()
{
    return this->is_alive;
}