#include "../include/location.h"

void __callback__click(int event, int x, int y, int flage, void *param)
{
    Location *location = reinterpret_cast<Location *>(param);
    Mat img_cut = Mat::zeros(Size(200, 200), CV_8UC3);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);
    Rect rect;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    switch (event)
    {
    case MouseEventTypes::EVENT_MOUSEMOVE:
        rect = cv::getWindowImageRect("PickPoints"); // origin:"PickPoints" 要改吗？
        location->frame.frame(Rect(Point(max(x - 100, 0), max(y - 100, 0)), Point(min(x + 100, location->frame.frame.cols - 1), min(y + 100, location->frame.frame.rows - 1))))
            .copyTo(img_cut(Rect(0, 0, min(x + 100, location->frame.frame.cols - 1) - max(x - 100, 0), min(y + 100, location->frame.frame.rows - 1) - max(y - 100, 0))));
        circle(img_cut, Point(100, 100), 1, Scalar(0, 255, 0), 1);
        imshow("ZOOM_WINDOW", img_cut);
        cv::moveWindow("ZOOM_WINDOW", rect.width - 400, rect.height + 200);
        cv::resizeWindow("ZOOM_WINDOW", 400, 400);
        break;
    case MouseEventTypes::EVENT_LBUTTONDOWN:
        if (!location->flag)
        {
            location->flag = true;
            logger->info("Pick:{}|{}", x, y);
            vector<Point2f> temp_corner;
            temp_corner.emplace_back(Point2f(x, y));
            Mat grey;
            cvtColor(location->frame.frame, grey, COLOR_BGR2GRAY);
            cornerSubPix(grey, temp_corner, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            location->pick_points.emplace_back(temp_corner[0]);
            circle(location->frame.frame, Point(x, y), 2, Scalar(0, 255, 0), 1);
        }
        break;
    }
}

Location::Location()
{
}

Location::~Location()
{
}

bool Location::locate_pick(CameraThread::Ptr cap, int camera_index, int enemy, Mat &rvec_Mat, Mat &tvec_Mat,
                           Mat &K_0, Mat &C_0, Mat &E_0)
{
    // CHECK:解码json文件获得真实3d坐标
    if (this->location_targets.empty())
    {
        this->logger->error("Empty MapPoints !");
        return false;
    }
    // C++
    /*
    没有显式构造函数的结构体对象:
    <1>FrameBag frame; // 默认初始化
    <2>this->frame = FrameBag();
        对于结构体，使用赋值运算符或初始化列表是更常见的做法，而不是使用构造函数。因此，this->frame = FrameBag(); 的目的是将 frame 成员变量初始化为 FrameBag 结构体的默认值。
    在 C++ 中，对于结构体，如果没有显式提供构造函数，编译器会自动生成默认构造函数。因此，FrameBag() 会调用该默认构造函数，返回一个具有默认初始化值的 FrameBag 结构体对象，然后将其赋值给 frame 成员变量。
    */
    // 初始化
    this->frame = FrameBag();
    this->flag = false;
    vector<Point2f>().swap(this->pick_points);
    // 将3d点归类
    map<int, vector<string>> tips;
    tips[0] = this->targets_selected_enemy_red; // C++：自动创建一个与键 0 关联的空向量，并返回对应的引用
    tips[1] = this->targets_selected_enemy_blue;
    vector<Point3f> ops;
    if (enemy == 0)
    {
        for (auto &it : this->targets_selected_enemy_red)
        {
            ops.emplace_back(location_targets.find(it)->second);
        }
    }
    else
    {
        for (auto &it : this->targets_selected_enemy_blue)
        {
            ops.emplace_back(location_targets.find(it)->second);
        }
    }

    // change CameraThread::read(),which must could collabrate with MV_Camera::read(int camera_index)!
    frame = cap->read(camera_index);

    if (!cap->is_open() || !frame.flag)
        return false;
    int tip_w = floor(frame.frame.cols / 2.);
    int tip_h = frame.frame.rows - 200;
    string window_name;
    if (camera_index == 0)
    {
        window_name = "Left_PickPoints";
    }
    else
    {
        window_name = "Right_PickPoints";
    }
    cv::namedWindow(window_name, WindowFlags::WINDOW_GUI_NORMAL);
    cv::resizeWindow(window_name, Size(1280, 780));
    cv::setWindowProperty(window_name, WindowPropertyFlags::WND_PROP_TOPMOST, 1);
    cv::moveWindow(window_name, 420, 150);
    cv::namedWindow("ZOOM_WINDOW", WindowFlags::WINDOW_GUI_NORMAL);
    cv::resizeWindow("ZOOM_WINDOW", 400, 400);
    cv::setWindowProperty("ZOOM_WINDOW", WindowPropertyFlags::WND_PROP_TOPMOST, 1);
    cv::setMouseCallback(window_name, __callback__click, reinterpret_cast<void *>(this));
    while (true)
    {
        putText(frame.frame, tips[(int)(enemy)][pick_points.size()], Point(tip_w, tip_h), HersheyFonts::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 255, 0), 2);
        for (const auto &it : pick_points)
            circle(frame.frame, it, 1, Scalar(0, 255, 0), 2);
        for (size_t i = 1; i < pick_points.size(); ++i)
            line(frame.frame, pick_points[i - 1], pick_points[i], cv::Scalar(0, 255, 0), 2);
        imshow(window_name, frame.frame);
        if (flag)
        {
            if (pick_points.size() == 4)
            {
                line(frame.frame, pick_points[3], pick_points[0], cv::Scalar(0, 255, 0), 2);
                imshow(window_name, frame.frame);
            }
            int key = waitKey(0);
            if (key == 90 || key == 122) // z/Z撤销选点
            {
                if (pick_points.size() == 4)
                    line(frame.frame, pick_points[3], pick_points[0], cv::Scalar(0, 0, 255), 2);
                else if (pick_points.size() > 1)
                    line(frame.frame, pick_points[pick_points.size() - 1], pick_points[pick_points.size() - 2], cv::Scalar(0, 0, 255), 2);
                circle(frame.frame, pick_points[pick_points.size() - 1], 1, cv::Scalar(0, 0, 255), 2);
                pick_points.pop_back();
                imshow(window_name, frame.frame);
            }
            else if (key == 81 || key == 113) // q/Q退出
            {
                cv::destroyWindow(window_name);
                cv::destroyWindow("ZOOM_WINDOW");
                return false;
            }
            flag = false;
        }
        else
        {
            waitKey(1);
        }
        if (pick_points.size() == 4)
            break;
        frame = cap->read(camera_index);
        if (!cap->is_open() || !frame.flag)
        {
            cv::destroyWindow(window_name);
            cv::destroyWindow("ZOOM_WINDOW");
            return false;
        }
    }
    cv::destroyWindow(window_name);
    cv::destroyWindow("ZOOM_WINDOW");
    if (!solvePnP(ops, pick_points, K_0, C_0, rvec_Mat, tvec_Mat, false, SolvePnPMethod::SOLVEPNP_P3P))
    {
        std::cout << window_name + "Solve PnP failed" << end;
        this->logger->error("Solve PnP failed");
        return false;
    }
    return true;
}

bool Location::decodeMapPoints(string path)
{
    this->location_targets.clear();
    this->targets_selected_enemy_blue.clear();
    this->targets_selected_enemy_red.clear();
    Json::Reader jsonReader;
    Json::Value jsonValue;
    std::ifstream jsonFile(path);
    if (!jsonReader.parse(jsonFile, jsonValue, true))
    {
        this->logger->error("Json file read error !");
        jsonFile.close();
        return false;
    }
    Json::Value arrayValue = jsonValue["Points"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
        Point3f point;
        point.x = arrayValue[i]["x"].asFloat();
        point.y = arrayValue[i]["y"].asFloat();
        point.z = arrayValue[i]["z"].asFloat();
        this->location_targets[arrayValue[i]["name"].asCString()] = point;
    }
    arrayValue = jsonValue["when_enemy_red"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
        this->targets_selected_enemy_red.emplace_back(arrayValue[i].asCString());
    }
    arrayValue = jsonValue["when_enemy_blue"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
        this->targets_selected_enemy_blue.emplace_back(arrayValue[i].asCString());
    }
    return true;
}