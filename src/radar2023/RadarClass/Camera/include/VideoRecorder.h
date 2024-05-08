#ifndef _VIDEORECORDER_H
#define _VIDEORECORDER_H

#include "../../Common/include/public.h"

/**
 * @brief 视频录制类
 * 提供对控制窗口图像的录制功能
 */
class VideoRecorder
{
public:
    typedef std::shared_ptr<VideoRecorder> Ptr;

private:
    VideoWriter vw[2];
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    VideoRecorder();
    ~VideoRecorder();

    bool init(int camera_index, const char *videoPath, int coder, Size size); // add index
    void write(Mat src);
    void close();
};

#endif