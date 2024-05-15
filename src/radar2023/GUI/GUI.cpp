#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <radar2023/Location.h>
#include <radar2023/Locations.h>
#include <unistd.h>

using namespace std;
using namespace cv;

static cv_bridge::CvImageConstPtr left_result_image;
static cv_bridge::CvImageConstPtr right_result_image;

static vector<radar2023::Location> locs;

void locations_msgCallback(const radar2023::Locations::ConstPtr &msg)
{
    locs.clear();
    for (auto &it : msg->locations)
    {
        locs.emplace_back(it);
    }
}

// add
void left_image_msgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        left_result_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
void right_image_msgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        right_result_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

#ifdef DoubleGui
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gui_node");
    ros::NodeHandle nh;
    std::string share_path = ros::package::getPath("radar2023");
    std::cout << "*******************GUI节点初始化*******************" << endl;
    ros::Subscriber msg_sub = nh.subscribe("/radar2023/locations", 1, locations_msgCallback);
    image_transport::ImageTransport it_(nh);

    // changed
    image_transport::Subscriber left_image_sub_;
    left_image_sub_ = it_.subscribe("/radar2023/left_result_view", 1, left_image_msgCallback);
    image_transport::Subscriber right_image_sub_;
    right_image_sub_ = it_.subscribe("/radar2023/right_result_view", 1, right_image_msgCallback); // TODO
    std::cout << "订阅消息成功" << endl;
    // TODO:怎样在一个界面中设置双相机的视图?
    /***程序标完pnp,开始识别时的界面GUI***/
    // 设置滑动条
    string left_win = "Left Camera GUI";
    string right_win = "Right Camera GUI";
    namedWindow(left_win, WINDOW_GUI_NORMAL);
    namedWindow(right_win, WINDOW_GUI_NORMAL);

    createTrackbar("Exit Program", left_win, 0, 1, nullptr);
    createTrackbar("Exit Program", right_win, 0, 1, nullptr);
    setTrackbarPos("Exit Program", left_win, 0);
    setTrackbarPos("Exit Program", right_win, 0);

    createTrackbar("Recorder", left_win, 0, 1, nullptr);
    createTrackbar("Recorder", right_win, 0, 1, nullptr);
    setTrackbarPos("Recorder", left_win, 1);
    setTrackbarPos("Recorder", right_win, 1);

    createTrackbar("CoverDepth", left_win, 0, 1, nullptr);
    createTrackbar("CoverDepth", right_win, 0, 1, nullptr);
    setTrackbarPos("CoverDepth", left_win, 1);
    setTrackbarPos("CoverDepth", right_win, 1);

    /** 加载配置参数**/
    bool if_exit_program = false;
    nh.setParam("/gui/CoverDepth", if_exit_program);
    bool _if_record = true;
    bool _if_coverDepth = true;
    string param_name, map_name;
    int ENEMY = 0;
    if (nh.searchParam("/gui/CoverDepth", param_name))
    {
        nh.getParam(param_name, _if_coverDepth);
        _if_coverDepth ? setTrackbarPos("CoverDepth", left_win, 1) : setTrackbarPos("CoverDepth", left_win, 0);
        _if_coverDepth ? setTrackbarPos("CoverDepth", right_win, 1) : setTrackbarPos("CoverDepth", right_win, 0);
    }
    else
    {
        ROS_WARN("Parameter CoverDepth not defined");
    }
    if (nh.searchParam("/gui/MapRMUC", param_name))
    {
        nh.getParam(param_name, map_name);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }
    if (nh.searchParam("/radar2023/EnemyType", param_name))
    {
        nh.getParam(param_name, ENEMY);
    }
    else
    {
        ROS_WARN("Parameter SerialPortName not defined");
    }
    // 小地图
    Mat map;
    if (!map_name.empty() && access((share_path + "/resources/" + map_name).c_str(), F_OK) == 0)
    {
        map = cv::imread(share_path + "/resources/" + map_name);
        if (ENEMY == 1)
            cv::flip(map, map, -1);
    }
    else
    {
        ROS_WARN("NON MAP !");
    }
    /** 加载完成**/

    while (ros::ok() && !if_exit_program)
    {
        ros::spinOnce();
        // “左相机”窗口
        if (!left_result_image || left_result_image->image.empty())
            continue;
        Mat left_display = Mat::zeros(Size(left_result_image->image.cols + left_result_image->image.rows * 0.54, left_result_image->image.rows), CV_8UC3);
        left_result_image->image.copyTo(left_display(Rect(0, 0, left_result_image->image.cols, left_result_image->image.rows)));
        if (!map.empty())
        {
            cv::resize(map, left_display(Rect(left_result_image->image.cols, 0, left_result_image->image.rows * 0.54, left_result_image->image.rows)), Size(left_result_image->image.rows * 0.54, left_result_image->image.rows));
        }
        // “右相机”窗口
        if (!right_result_image || right_result_image->image.empty())
            continue;
        Mat right_display = Mat::zeros(Size(right_result_image->image.cols + right_result_image->image.rows * 0.54, right_result_image->image.rows), CV_8UC3);
        right_result_image->image.copyTo(right_display(Rect(0, 0, right_result_image->image.cols, right_result_image->image.rows)));
        if (!map.empty())
        {
            cv::resize(map, right_display(Rect(right_result_image->image.cols, 0, right_result_image->image.rows * 0.54, right_result_image->image.rows)), Size(right_result_image->image.rows * 0.54, right_result_image->image.rows));
        }

        // 小地图坐标显示
        for (auto it : locs) // share 2
        {
            if (it.x == 0 && it.y == 0)
                continue;
            if (ENEMY == 1)
            {
                it.x = 28.0 - it.x;
                it.y = 15.0 - it.y;
            }
            Scalar color;
            string text;
            if (it.id < 6)
            {
                color = Scalar(255, 0, 0);
                text += "B";
            }
            else
            {
                color = Scalar(0, 0, 255);
                text += "R";
            }
            // add
            Point2f l_center = Point2f((it.y / 15.0) * left_result_image->image.rows * 0.54 + left_result_image->image.cols, (it.x / 28.0) * left_result_image->image.rows);
            Point2f r_center = Point2f((it.y / 15.0) * right_result_image->image.rows * 0.54 + right_result_image->image.cols, (it.x / 28.0) * right_result_image->image.rows);
            // add
            circle(left_display, l_center, 40.0, color, 3);
            circle(right_display, r_center, 40.0, color, 3);

            text += to_string(it.id);
            int baseline;
            Size text_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
            // add
            l_center.x = l_center.x - text_size.width / 2;
            l_center.y = l_center.y + (text_size.height) / 2;
            r_center.x = r_center.x - text_size.width / 2;
            r_center.y = r_center.y + (text_size.height) / 2;
            // add
            putText(left_display, text, l_center, FONT_HERSHEY_SIMPLEX, 1, color, 2);
            putText(right_display, text, r_center, FONT_HERSHEY_SIMPLEX, 1, color, 2);
        }
        // add
        cv::imshow(left_win, left_display);
        cv::imshow(right_win, right_display);

        cv::resizeWindow(left_win, Size(1536, 864));
        cv::resizeWindow(right_win, Size(1536, 864));

        // bool if_exit_program_current = getTrackbarPos("Exit Program", left_win) == 1 ? true : false;
        // bool _if_record_current = getTrackbarPos("Recorder", left_win) == 1 ? true : false;
        // bool _if_coverDepth_current = getTrackbarPos("CoverDepth", left_win) == 1 ? true : false;

        // TODO：暂时以右相机的滑动条为准
        bool if_exit_program_current = getTrackbarPos("Exit Program", right_win) == 1 ? true : false;
        bool _if_record_current = getTrackbarPos("Recorder", right_win) == 1 ? true : false;
        bool _if_coverDepth_current = getTrackbarPos("CoverDepth", right_win) == 1 ? true : false;

        if (if_exit_program_current != if_exit_program)
        {
            if_exit_program = if_exit_program_current;
            nh.setParam("/radar2023/ExitProgram", if_exit_program);
        }
        if (_if_record_current != _if_record)
        {
            _if_record = _if_record_current;
            nh.setParam("/radar2023/Recorder", _if_record);
        }
        if (_if_coverDepth_current != _if_coverDepth)
        {
            _if_coverDepth = _if_coverDepth_current;
            nh.setParam("/gui/CoverDepth", _if_coverDepth);
        }
        cv::waitKey(1);
    }
    cv::destroyAllWindows();
    ros::shutdown();
    return 0;
}
#endif