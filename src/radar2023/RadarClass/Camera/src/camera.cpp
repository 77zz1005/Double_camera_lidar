#include "../include/camera.h"

#ifndef UsingVideo
// unsigned char camera_match_index = (char)0;                  //相机索引号

MV_Camera::MV_Camera()
{
}

MV_Camera::MV_Camera(bool Is_init, string config_path_0, string config_path_1)
{
    this->CameraConfigPath[0] = config_path_0;
    this->CameraConfigPath[1] = config_path_1;

    CameraSdkInit(1); // 设置初始化时的日志级别

    // 枚举
    this->iStatus = CameraEnumerateDevice(this->tCameraEnumList, &this->iCameraCounts);
    if (this->iCameraCounts == 0)
    {
        this->logger->error("No camera found!");
        sleep(1);
        return;
    }
    if (this->iCameraCounts == 1)
    {
        this->logger->error("Only 1 camera found!");
        sleep(1);
        return;
    }

    // 设备序列号 区分相机
    this->SerialNumber[0] = string(this->tCameraEnumList[0].acSn);
    std::cout << "Camera Serial Number: " + this->SerialNumber[0] << std::endl; // 为了调试
    this->Index_Camera_[0] = 0;
    this->SerialNumber[1] = string(this->tCameraEnumList[1].acSn);
    std::cout << "Camera Serial Number: " + this->SerialNumber[1] << std::endl; // 为了调试
    this->Index_Camera_[1] = 1;
    // TODO
    // 通过cameraSeq.SerialNumber[i]来判断cameraSeq.Index_Camera_[i]是0还是1
    /*
    int getIndexBySerialNumber(const string& serialNumber) {
        for (int i = 0; i < 2; ++i) {
            if (cameraSeq.SerialNumber[i] == serialNumber) {
                return cameraSeq.Index_Camera_[i];
            }
        }
        return -1; // 如果未找到匹配的序列号，返回 -1 表示未知
    }

    */

    // set up camera
    //  First camera
    try
    {
        this->iStatus = CameraInit(&this->tCameraEnumList[0], -1, -1, &this->hCamera[0]);
        this->logger->info("One CameraIniting ...Process");
    }
    catch (const std::exception &e)
    {
        this->logger->error("First CameraInitERROR!{}", e.what());
        sleep(1);
        // TODO:CameraUnInit(this->hCamera[0]);
        return;
    }
    // Second camera
    try
    {
        this->iStatus = CameraInit(&this->tCameraEnumList[1], -1, -1, &this->hCamera[1]);
        this->logger->info("Two CameraIniting ...Process");
    }
    catch (const std::exception &e)
    {
        this->logger->error("Second CameraInitERROR!{}", e.what());
        sleep(1);
        // TODO:CameraUnInit(this->hCamera[1]);
        return;
    }

    if (this->iStatus != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("Double CameraInit Failed!{}", this->iStatus);
        sleep(1);
        return;
    }
    else
    {
        this->logger->info("Double CameraIniting ...Done");
    }

    //// 获取相机设备的能力描述信息，包括支持的分辨率范围、帧率范围、像素格式等
    // 第一个相机的初始化和参数设置
    CameraGetCapability(this->hCamera[0], &this->tCapability[1]);
    if (!tCapability[0].sIspCapacity.bMonoSensor)
        CameraSetIspOutFormat(this->hCamera[0], CAMERA_MEDIA_TYPE_BGR8);
    else
    {
        this->logger->error("None suitable camera_0!");
    }
    // 设置
    CameraSetTriggerMode(this->hCamera[0], 0);
    if (Is_init || access(CameraConfigPath[0].c_str(), F_OK) == 0)
        CameraReadParameterFromFile(this->hCamera[0], const_cast<char *>(CameraConfigPath[0].c_str()));
    CameraSetAeState(this->hCamera[0], 0);
    CameraPlay(this->hCamera[0]);
    int frameBufferSize_0 = this->tCapability[0].sResolutionRange.iWidthMax * this->tCapability[0].sResolutionRange.iHeightMax * 3;
    this->pFrameBuffer[0] = CameraAlignMalloc(frameBufferSize_0, 16);

    // 第二个相机的初始化和参数设置
    if (this->iCameraCounts == 2)
    {
        CameraGetCapability(this->hCamera[1], &this->tCapability[1]);
        if (!tCapability[1].sIspCapacity.bMonoSensor)
            CameraSetIspOutFormat(this->hCamera[1], CAMERA_MEDIA_TYPE_BGR8);
        else
        {
            this->logger->error("None suitable camera_1!");
        }
        CameraSetTriggerMode(this->hCamera[1], 0);
        if (Is_init || access(CameraConfigPath[1].c_str(), F_OK) == 0)
            CameraReadParameterFromFile(this->hCamera[1], const_cast<char *>(CameraConfigPath[1].c_str()));
        CameraSetAeState(this->hCamera[1], 0);
        CameraPlay(this->hCamera[1]);
        int frameBufferSize_1 = this->tCapability[1].sResolutionRange.iWidthMax * this->tCapability[1].sResolutionRange.iHeightMax * 3;
        this->pFrameBuffer[1] = CameraAlignMalloc(frameBufferSize_1, 16);
    }
    this->logger->info("Camera setup complete");
}

MV_Camera::~MV_Camera()
{
}

// change: 根据index获取图像buffer
FrameBag MV_Camera::read(int cameraIndex)
{
    FrameBag framebag;
    if (cameraIndex < 0 || cameraIndex >= 2)
    {
        this->logger->error("Invalid camera index!");
        return framebag;
    }
    if (this->hCamera[cameraIndex] == -1)
    {
        this->logger->error("No handled camera found for index {}!", cameraIndex);
        sleep(1);
        return framebag;
    }
    // 原图
    CameraGetImageBuffer(this->hCamera[cameraIndex], &this->sFrameInfo[cameraIndex], &this->pRawDataBuffer[cameraIndex], 200); // 200(ms 超时时间)
    // 处理图
    if (CameraImageProcess(this->hCamera[cameraIndex], this->pRawDataBuffer[cameraIndex], this->pFrameBuffer[cameraIndex], &this->sFrameInfo[cameraIndex]) != CAMERA_STATUS_SUCCESS)
    {
        this->logger->error("Can not process Image for camera index {}!", cameraIndex);
        sleep(1);
        return framebag;
    }
    // 赋值
    framebag.device_seq = cameraIndex;
    framebag.frame = cv::Mat(
        Size(this->sFrameInfo[cameraIndex].iWidth, this->sFrameInfo[cameraIndex].iHeight),
        CV_8UC3,
        this->pFrameBuffer[cameraIndex]);
    // 释放临时缓冲区
    CameraReleaseImageBuffer(this->hCamera[cameraIndex], this->pRawDataBuffer[cameraIndex]);
    framebag.flag = true;
    return framebag;
}

void MV_Camera::uninit()
{
    CameraUnInit(this->hCamera[0]);
    CameraUnInit(this->hCamera[1]);
}

// changed
void MV_Camera::setExposureTime(int camera_index, int ex)
{
    if (this->hCamera[camera_index] == -1)
        return;
    CameraSetExposureTime(this->hCamera[camera_index], ex);
}
// changed
void MV_Camera::setGain(int gain, int camera_index)
{
    if (this->hCamera[camera_index] == -1)
        return;
    CameraSetAnalogGain(this->hCamera[camera_index], gain);
}
// //
void MV_Camera::saveParam(int camera_index, const char tCameraConfigPath)
{
    if (access(tCameraConfigPath, F_OK) == 0)
        return;
    if (this->hCamera[camera_index] == -1)
        return;
    CameraSaveParameterToFile(this->hCamera[camera_index], const_cast<char *>(tCameraConfigPath));
}

// changed
void MV_Camera::disableAutoEx(int camera_index)
{
    if (this->hCamera[camera_index] == -1)
        return;
    CameraSetAeState(this->hCamera[camera_index], 0);
}
// changed
int MV_Camera::getExposureTime(int camera_index)
{
    if (this->hCamera[camera_index] == -1)
        return -1;
    double ex;
    CameraGetExposureTime(this->hCamera[camera_index], &ex);
    return int(ex);
}

// changed
int MV_Camera::getAnalogGain(int camera_index)
{
    if (this->hCamera[camera_index] == -1)
        return -1;
    int gain;
    CameraGetAnalogGain(this->hCamera[camera_index], &gain);
    return gain;
}

#endif

#ifdef UsingVideo
CameraThread::CameraThread(string config_path_0, string config_path_0, string video_path_0, string video_path_1)
{
    // TODO:统一格式
    this->CameraConfigPath_0 = config_path_0;
    this->CameraConfigPath_1 = config_path_1;
    this->TestVideoPath[0] = video_path_0;
    this->TestVideoPath[1] = video_path_1;
}
#else
CameraThread::CameraThread(string config_path_0, string config_path_1)
{
    //// add two
    this->CameraConfigPath_0 = config_path_0;
    this->CameraConfigPath_1 = config_path_1;
}
#endif
CameraThread::~CameraThread()
{
}

void CameraThread::start()
{
    while (!this->_open)
    {
        this->open();
    }
}

void CameraThread::stop()
{
    this->_is_init = false;
    this->_open = false;
    this->_alive = false;
    this->release(); // changed
}

#ifndef UsingVideo
void CameraThread::openCamera(bool is_init)
{
    bool initFlag = false;
    // TODO
    int left_index = this->_cap->Index_Camera_[0];  // 使用索引为0的相机
    int right_index = this->_cap->Index_Camera_[1]; // 使用索引为1的相机
    try
    {
        this->logger->info("Camera opening ...Process");
        this->_cap = std::make_shared<MV_Camera>(is_init, this->CameraConfigPath_0, this->CameraConfigPath_0); // change inside and param[in]

        FrameBag framebag_0 = this->_cap->read(left_index);
        FrameBag framebag_1 = this->_cap->read(right_index);

        // CHECK
        if (!framebag_0.flag || !framebag_1.flag)
        {
            this->logger->warn("Double Camera not inited");
            return;
        }
        // TODO:尝试重新初始化

        // 手动调节相机参数
        if (!is_init && framebag_0.flag && framebag_1.flag)
        {
            // changed：t/T y/Y
            // TODO:先假设0为左相机吧……
            namedWindow("LEFT_CAMERA_PREVIEW", WINDOW_NORMAL); // 预览
            resizeWindow("LEFT_CAMERA_PREVIEW", Size(840, 640));
            setWindowProperty("LEFT_CAMERA_PREVIEW", WND_PROP_TOPMOST, 1); // 将窗口置顶显示
            moveWindow("LEFT_CAMERA_PREVIEW", 100, 100);                   // 移动窗口到指定位置
            imshow("LEFT_CAMERA_PREVIEW", framebag_0.frame);               // 在窗口中显示左相机图像
            int key = waitKey(0);                                          // 等待键盘输入
            destroyWindow("LEFT_CAMERA_PREVIEW");                          // 销毁窗口

            this->_cap->disableAutoEx(left_index);                               // 禁用左相机的自动曝光功能
            if (key == 84 || key == 116)                                         // 键盘t/T
                this->adjustExposure(left_index);                                // 调整左相机曝光参数
            this->_cap->saveParam(left_index, this->CameraConfigPath_0.c_str()); // 保存左相机参数路径

            // TODO:先假设1为右相机
            namedWindow("RIGHT_CAMERA_PREVIEW", WINDOW_NORMAL);
            resizeWindow("RIGHT_CAMERA_PREVIEW", Size(840, 640));
            setWindowProperty("RIGHT_CAMERA_PREVIEW", WND_PROP_TOPMOST, 1); // 置顶
            moveWindow("RIGHT_CAMERA_PREVIEW", 1000, 100);                  // 移动窗口到指定位置，水平位置不同
            imshow("RIGHT_CAMERA_PREVIEW", framebag_1.frame);
            int r_key = waitKey(0);
            destroyWindow("RIGHT_CAMERA_PREVIEW");
            this->_cap->disableAutoEx(right_index);                               // 关闭右相机自动曝光
            if (r_key == 121 || r_key == 89)                                      // 如果按下 y/Y 键
                this->adjustExposure(right_index);                                // 调整右相机曝光
            this->_cap->saveParam(right_index, this->CameraConfigPath_1.c_str()); // 保存右相机参数路径
        }

        initFlag = true;
        this->logger->info("Camera opening ...Done");
    }
    catch (const std::exception &e)
    {
        this->logger->error("CameraThread::openCamera(){}", e.what());
        sleep(1);
        return;
    }
    this->_cap->_openflag = initFlag;
}

// changed
void CameraThread::adjustExposure(int camera_index)
{
    string window_name = "EXPOSURE_" + to_string(camera_index) + " Press Q to Exit";

    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 1280, 960);
    moveWindow(window_name, 200 + camera_index * 200, 200); // Adjust window position based on camera index
    setWindowProperty(window_name, WND_PROP_TOPMOST, 1);
    createTrackbar("ex", window_name, 0, 30000);
    setTrackbarPos("ex", window_name, this->_cap->getExposureTime(camera_index) != -1 ? this->_cap->getExposureTime(camera_index) : 0); // param[in]:滑动条名称，窗口名称，滑动条初始位置
    createTrackbar("gain", window_name, 0, 256);
    setTrackbarPos("gain", window_name, this->_cap->getAnalogGain(camera_index) != -1 ? this->_cap->getAnalogGain(camera_index) : 0);
    createTrackbar("Quit", window_name, 0, 1);
    setTrackbarPos("Quit", window_name, 0);

    FrameBag framebag = this->_cap->read(camera_index); // processed but "raw"

    // 循环处理直至退出【注：使用时有2种退出方式】：
    // <1>：waitKey(1) != 81 和 waitKey(1) != 113：分别检查按键的 ASCII 值是否不等于 81 和 113，即按键不是 "Q" 或 "q"（当按下 "Q" 或 "q" 键时，循环结束）
    //<2>:getTrackbarPos("Quit", "EXPOSURE Press Q to Exit") == 0：检查名为 "Quit" 的滑动条在指定窗口中的位置是否为 0(退出)
    while (framebag.flag && waitKey(1) != 81 && waitKey(1) != 113 && getTrackbarPos("Quit", window_name) == 0)
    {
        // 获取滑动条value 并设置相机的曝光时间和增益
        this->_cap->setExposureTime(camera_index, getTrackbarPos("ex", window_name));
        this->_cap->setGain(camera_index, getTrackbarPos("gain", window_name));

        // 显示图像
        imshow(window_name, framebag.frame);
        // 读取下一帧图像
        framebag = this->_cap->read(camera_index);
        // 等待按键事件
        waitKey(1);
    }

    // 获取最终设置的曝光时间和增益
    int ex = this->_cap->getExposureTime(camera_index);
    int gain = this->_cap->getAnalogGain(camera_index);
    this->logger->info("Setting Expoure Time {}us", ex);
    this->logger->info("Setting analog gain {}", gain);
    // 关闭窗口
    destroyWindow(window_name);
}

#endif

void CameraThread::open()
{
#ifdef UsingVideo
    if (!this->_open)
    {
        if (access(TestVideoPath.c_str(), F_OK) != 0)
        {
            this->logger->error("No video file : {}", TestVideoPath);
            sleep(1);
            return;
        }
        this->_cap = VideoCapture(TestVideoPath);
        this->_open = true;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#else
    if (!this->_open && this->_alive)
    {
        this->openCamera(this->_is_init); // 手动设置参数
        this->_open = this->_cap->_openflag;
        if (!this->_is_init && this->_open)
            this->_is_init = true;
    }
#endif
}

bool CameraThread::is_open()
{
    return this->_open;
}

FrameBag CameraThread::read(int camera_index)
{
    FrameBag framebag;
    if (this->_open && this->_alive)
    {
// TODO
#ifdef UsingVideo
        this->frame_counter[camera_index] += 1;

        if (this->frame_counter[camera_index] == int(_cap[camera_index].get(cv::CAP_PROP_FRAME_COUNT)))
        {
            this->frame_counter[camera_index] = 0;

            _cap[camera_index].set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        this->_cap[camera_index].read(framebag.frame);
        framebag.flag = !framebag.frame.empty();
#else
        framebag = this->_cap->read(camera_index);
#endif
    }
    else if (this->_alive)
    {
        this->logger->warn("Camera closed !");
    }
    if (!framebag.flag && this->_alive)
    {
        this->logger->error("Failed to get frame!");
    }
    return framebag;
}

void CameraThread::release()
{
#ifndef UsingVideo
    this->_cap->uninit();
#endif
    this->_open = false;
}