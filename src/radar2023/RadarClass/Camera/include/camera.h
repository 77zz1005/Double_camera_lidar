#ifndef __CAMERA_H
#define __CAMERA_H

#include "../../Common/include/public.h"
#include "CameraDefine.h"
#include "CameraStatus.h"
#include "CameraApi.h"

#define  DISPLAY_MAX     2

#ifndef UsingVideo
/**
 * @brief 相机驱动类
 * 迈德威视驱动程序，提供格式化的OpenCV Mat图像
 */

/*
Log-记录一下主要修改
1.将多个信息修改成数组 注：不改成容器是因为容器动态，数组大小固定（静态），时间关系先不假设只有一个相机正常运行的情况
2.TODO：通过SerialNumber区分不同相机，并且简短地令index为0 1
*/
class MV_Camera
{
public:
    typedef std::shared_ptr<MV_Camera> Ptr;

private:
    int iCameraCounts = DISPLAY_MAX;
    int iStatus = -1;                     // share 2
    tSdkCameraDevInfo tCameraEnumList[DISPLAY_MAX]; // 枚举
    int hCamera[DISPLAY_MAX] = {-1, -1};            // 存储两个相机的句柄
    tSdkCameraCapbility tCapability[DISPLAY_MAX];
    tSdkFrameHead sFrameInfo[DISPLAY_MAX];
    unsigned char *pFrameBuffer[DISPLAY_MAX];
    unsigned char *pRawDataBuffer[DISPLAY_MAX];
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    string CameraConfigPath[2]; // 路径
    string SerialNumber[2];     // TODO:用于存储相机序列号,通过硬件确定产品唯一序列号



public:
    bool _openflag = false;

public:
    MV_Camera();
    MV_Camera(bool Is_init, string config_path_0, string config_path_1); // share 2
    ~MV_Camera();

    // vector<FrameBag> read(); // share two
    FrameBag read(int cameraIndex); // add index:读取指定相机的图像

    void uninit(); // share 2

    void setExposureTime(int camera_index, int ex = 30); // add index
    // C++
    /*
    Err: void setExposureTime(int ex = 30, int camera_index)
    默认参数位置问题：在声明函数setExposureTime时，默认参数应该在非默认参数之后。例如，应该这样声明函数：
    void setExposureTime(int camera_index, int ex = 30); // add index
    */
    void setGain(int camera_index, int gain); // add index

    void saveParam(int camera_index, const char tCameraConfigPath[23]); // add index

    void disableAutoEx(int camera_index);  // add index
    int getExposureTime(int camera_index); // add index
    int getAnalogGain(int camera_index);   // add index
};
#endif

/**
 * @brief 相机线程内部类
 * 单独驱动的相机线程内部类，提供接口以获取图像
 */
class CameraThread
{
public:
    typedef std::shared_ptr<CameraThread> Ptr;


private:
    bool _open = false;
    bool _alive = true;

#ifdef UsingVideo
    VideoCapture _cap;
    int frame_counter = 0;
    string TestVideoPath;
#else
    MV_Camera::Ptr _cap;
#endif

    bool _is_init = false;
    string CameraConfigPath_0;
    string CameraConfigPath_1;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
#ifndef UsingVideo
    void openCamera(bool is_init);         // share 2
    void adjustExposure(int camera_index); // add index

#endif

#ifdef UsingVideo
    CameraThread(string config_path, string video_path);
#else
    CameraThread(string config_path_0, string config_path_1);
#endif

    ~CameraThread();
    void open(); // share 2
    bool is_open();
    FrameBag read(int camera_index); // add index
    void release();
    void start();
    void stop();
};

#endif