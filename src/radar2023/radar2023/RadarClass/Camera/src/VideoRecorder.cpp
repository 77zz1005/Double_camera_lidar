#include "../include/VideoRecorder.h"

VideoRecorder::VideoRecorder()
{
}

VideoRecorder::~VideoRecorder()
{
    this->close();
}

bool VideoRecorder::init(int camera_index, const char *videoPath, int coder, Size size)
{
    if (access(videoPath, F_OK) == -1)
    {
        this->logger->error("[ERR] VideoPath, non-existent");
        this->logger->flush();
    }
    if (!this->vw[camera_index].isOpened())
    {
        char filename[1024];
        time_t currentTime = time(NULL);
        char chCurrentTime[256];
        strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
        strcat(chCurrentTime, ".mp4");
        strcpy(filename, videoPath);
        int length = strlen(videoPath);
        if (videoPath[length - 1] != '/')
        {
            strcat(filename, "/");
        }
        strcat(filename, chCurrentTime);
        this->vw[camera_index] = VideoWriter();
        if (!this->vw[camera_index].open(filename, coder, 60.0, size, true))
        {
            this->logger->warn("Block Video Recoder");
            return false;
        }
    }
    return true;
}

void VideoRecorder::write(int camera_index, Mat src)
{
    if (!this->vw[camera_index].isOpened())
        return;
    this->vw[camera_index].write(src);
}

void VideoRecorder::close()
{
    this->vw[0].release();
    this->vw[1].release();
}