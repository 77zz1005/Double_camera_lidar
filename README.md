# RM_Radar2023

## 开发日志
#### 自主决策 [易伤自动发送]

TO TEST:设计决策，完善串口通信 
* 开局一分钟左右发送工程定点坐标
   TODO:在Radar.h中修改：

         #define BLUE_ENGINEER_X 14.0
         #define BLUE_ENGINEER_Y 7.5
         #define RED_ENGINEER_X 14.0
         #define RED_ENGINEER_Y 7.5
* 自动决定易伤
   注：没有定位模块，暂时无法测试

#### 双相机方案
1.单相机-->双相机
   * （1）如何区分左右相机 
   
   解决方案：通过程序输出不同相机的产品序号，确定唯一相机，最后贴标签纸 [MV_Camera是一起初始化的，构造函数会输出相关内容 ]
   
   进度：乱码

   TODO:解决方案：<1>电脑分配的硬件地址<2>通过完善GUI界面决定那个是左相机，哪个是右相机 
   
   * （2）双相机pnp标定界面 

   解决方案：按顺序标，先左后右 

   * （3）推理 

   进度：单相机程序运行速度40FPS(含坐标结算)；为保证同步，双相机驱动、推理在同一线程中，运行速度21-23FPS

   * （4）融合识别信息 

  在3d->2d信息时融合：优先采用右相机的识别框，与左相机做逻辑合并

  验证方案：TODO:（通过完善GUI的功能进行验证）结算左右相机在不同情况下输出的位置信息，以检查融合识别框是否有逻辑错误 

      * 两个视野中有同一辆车的识别效果，以及结算精度【用尺子量】  且应该以右相机的识别框为准
      * 两个视野中只有一个相机看到了某一辆车：（1）左相机：应有输出 （2）右相机：应有输出

  TODO:合并信息时没有利用置信度/深度等任何信息 后续可以通过测试来决定

   * 右相机效果可能更好，如何融合左右相机识别框的世界坐标信息？重复id/无重复id 
   #### 其他变化

   （1）视频录制GUI ：this->nh->setParam("/radar2023/Recorder", false); 
      
       双相机视频录制，有2个界面

       TODO:后面测试成功了可以改回一个界面 控制2个相机

       * 关于视频读取：保留读取一个视频

   TODO:修改利用视频测试相关 


#### 位置预测（用运动模型，不是iou框）【由于可能在小地图上跳变，区域赛暂不考虑上场】

TODO: 测试deepsort跟踪效果

TODO: 如果后面在小地图上的位置不会因为误识别而跳变，则完善并启用

2.测试现有程序精度 
* 识别效果
[Program]

<第二层网络分类cls,重新映射的id>

[B1 B2 B3 B4 B5 B7 R1 R2 R3 R4 R5 R7]--->[R1 R2 R3 R4 R5 R7 B1 B2 B3 B4 B5 B7 ]

map<int, int> _ids = {{0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}, {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}};

装甲板分类[B1 B2 B3 B4 B5 B7 R1 R2 R3 R4 R5 R7]

[识别]

测试对象：红/英雄、步兵、哨兵

偶有误识别，但是基本正确

TODO:即使过滤了装甲板，但是在Box中可能会出现识别出了2个装甲板的情况。或许可以取深度均值？

TODO:
* 定位精度

待测量

4.测试单网络+k聚类+deepsort的效果（参考华南理工）

## 稳定功能
在沈阳航空航天大学T-UP战队2023赛季雷达程序基础上的拓展

方案：单相机与Livox mid-70 + pnp结算

请认真阅读本文档，配置程序时需要注意部分细节，务必完成Demo运行后再进行参数修改

程序点云接收基于ros-noetic框架，使用ROS版Livox雷达驱动【注：代码数据处理部分不基于ROS】

[雷达站赛场识别效果](https://www.bilibili.com/video/BV11u4y1x7qh/?vd_source=5db413777174c970e9df0ab76facd06c "B站链接")

[TRT-YOLOV5推理模块](https://github.com/tup-robomaster/TRTInferenceForYolo "GitHub链接")

[通用onnx->engine转换器](https://github.com/INIF-FISH/onnx2tensorrt_cpp "GitHub链接")

## 0.前言

程序整体的设计目的在于尽可能的压榨算力的条件下保留充足的可扩展性，各模块充分解耦、即插即用，做到程序二次开发简便、扩展性高。 

特别感谢上海交通大学[SJTU]提供的开源程序及测试资源对本项目的帮助。

## 1.简介

本程序适用采用激光雷达+单工业相机方案的雷达岗。
硬件：我们采用Livox 觅道Mid-70激光雷达和迈德威视MV-SUA630C-T工业相机，串口通讯采用了USB转TTL的方式。
识别方案:速度略低但精度较高的双层神经网络方案（GPU重负载CPU轻负载）。
融合方案：_____

功能：提供场上车辆的高精度坐标

部署前需要自行准备的并更改的有：

* Livox雷达驱动配置[包含于ThirdParty]
* 双相机内参与畸变参数
   外参配置：Config.h/default.yaml：CameraParam0 | CameraParam1 的文件名
   文件存放：params/
   图像参数：Config.h/default.yaml：CameraConfig0 | CameraConfig1 的文件名
   文件存放：params/
* 相机到激光雷达的外参 推荐使用 [Livox相机-激光雷达 联合标定](https://github.com/Livox-SDK/livox_camera_lidar_calibration "Github链接")[包含于ThirdParty]
* 网络对应的TensorRT相关参数[废弃]
* 车辆及装甲板识别模型
* 测试视频[可选]

对每个模块更加具体的说明可以参考文件夹下的README.md文件:

> [Camera](src/radar2023/RadarClass/Camera/README.md)
> [Detectors](src/radar2023/RadarClass/Detectors/README.md)
> [Location](src/radar2023/RadarClass/Location/README.md)
> [Radar](src/radar2023/RadarClass/Radar/README.md)

## 2.环境配置

在使用minimum配置或显存为6G时，应注意程序占用，雷达程序显存占用约为5G，显存吃紧时不应运行其他程序。

运算平台 (tested && minimum)：
  
[minimum]

* Intel Core i5-1250P CPU
* DDR4 16G 3200 RAM
* GeForce RTX 3060 Laptop GPU [6GB]

2[best]

* Ubuntu 20.04 LTS
* GCC 9.4.0
* CUDA 11.8
* cudnn 8.9.0
* Tensorrt 8.5.2.2
* ros-noetic
* OpenCV4.6.0
* PCL 1.10.0(ROS安装附带)
* spdlog
* fmt[选择性安装]
* Eigen3
* MKL[可选]
* matplotlibcpp[可选]
* Livox雷达驱动
* 迈德威视相机驱动

运算平台 (best)：

* Intel Core i9 13900KF CPU
* DDR5 32G 7400 RAM
* GeForce RTX 4090 GPU

## 3.文件结构-SRC [OLD]

* radar2023
  * demo_resource 存放常用资源和工具
  * logs 日志存放文件夹
  * RadarClass 雷达模块库
    * Camera 相机及录制相关
    * Common 通用文件
    * Detectors 检测器
    * Location 坐标系处理相关
    * Logger spdlog日志记录器
    * Radar 程序主线程及线程管理
    * UART 官方裁判系统通讯
  * tools 调试工具
  * Recorder 录制文件存放
  * CMakeLists.txt CMake文件
  * config.h 程序参数配置文件
  * main.cpp 程序主文件
  * package.xml ROS

### 使用前准备

* 环境配置完成后，需根据运算平台及环境修改src下CMakeLists.txt

  * 特别注意事项：
  * 应根据设备显卡型号及其算力修改第20行 CUDA_GEN_CODE 例：RTX 4060 ： -gencode=arch=compute_89, code=sm_89 ;

* 若不使用MKL，注释掉src/radar2023/CMakeLists.txt ->34 include_directories(/opt/intel/oneapi/mkl/latest/include)、35 link_directories(/opt/intel/oneapi/mkl/latest/lib/intel64) 、108 libmkl_rt.so 和 src/radar2023/RadarClass/Common/include/public.h
  -> 3 #define EIGEN_USE_MKL_ALL、4 #define EIGEN_VECTORIZE_SSE4_2 [可选]
* 修改Config目录下default.yaml以适配参数
* ExpResultDir[实验数据输出目录]、logs[日志存放目录]、models[模型存放目录『onnx、engine』]、params[参数存放目录『Config、yaml、Json』]、Record[录制视频存放目录]、resources[资源存放目录『pcds.txt、*.mp4、map.jpg』]
* 修改config.h中相关配置

  * 特别注意事项：
  * 本项目不存在测试用视频，需自行准备并放置于resources目录下
* 准备装甲板识别及车辆识别模型，现版本可用模型为yolov5 v6.0，注意导出动态Onnx
* 车辆分类[CAR]
* 装甲板分类[B1 B2 B3 B4 B5 B7 R1 R2 R3 R4 R5 R7]
* 将yolov5导出的动态尺寸onnx放置于models目录
* 将标定所得参数放置在params目录中，格式如资源文件camera0.SJTU.yaml所示
* 确保ROS环境激活后在RM_RADAR2023文件夹下使用（oneapi命令为mkl所需环境）：

  ```
  source /opt/ros/noetic/setup.sh
  source ~/intel/oneapi/setvars.sh
  catkin_make
  ```
* 为串口及雷达驱动添加权限[在串口权限自动设置失败时使用]

### 启动 [XXX详见livox_ros_driver->README.md]

```
source devel/setup.bash
source ~/intel/oneapi/setvars.sh
roslaunch livox_ros_driver livox_lidar.launch bd_list:="XXX" 
roslaunch radar2023 radar2023.launch
```

### 使用

1.若第一次运行，程序会针对运行设备进行Onnx生成Engine的过程，根据设备算力，时间在1～10分钟不等，此操作会一共进行两次（车辆模型和装甲板模型）。

2.初始化完成后，程序会尝试启动相机并展示一张预览图，若对相机曝光、增益不满意，在预览图上按“t”进入调节界面，在调节界面中有相应的退出滑条。

3.相机预览结束后，进入四点标注界面，根据提示完成标注点选取，每确定一点，在标定窗口按“z”撤回，按其他任意键确定。Tips: 在params/MapMappingPoints配置标定点

4.标定结束后，进入控制窗口，可控制程序退出和视频录制，在图像中显示空间点反投影效果和识别车辆，对标定结果不满意可退出程序重新标定。

5.【不稳定功能未在此列出】

### 其他Launch文件说明-使用离线点云

使用下列launch前需修改其中pcd_path参数

预置pcds.txt由上交开源程序提供，需搭配上交开源视频[非上交视频仅用于正常运行]

* OfflinePointCloudPub_and_Rviz.launch //启动离线点云发布及Rviz
  ```
  roslaunch radar2023 OfflinePointCloudPub_and_Rviz.launch
  ```
* OfflinePointCloudPub.launch //启动离线点云发布
  ```
  roslaunch radar2023 OfflinePointCloudPub.launch
  ```
* radar2023_with_OfflinePointCloudPub_and_Rviz.launch //启动离线点云发布、Rviz及雷达主程序
  ```
  roslaunch radar2023 radar2023_with_OfflinePointCloudPub_and_Rviz.launch
  ```
* radar2023_with_OfflinePointCloudPub.launch //启动离线点云发布及雷达主程序
  ```
  roslaunch radar2023 radar2023_with_OfflinePointCloudPub.launch
  ```
#### 使用

修改Config.h中注释掉的UsePointCloudSepTarget项目及配套配置:实验性功能-深度图背景分割

重新编译即可使用

#### 注意

此为实验性功能，不保证有效效果

### 实验性功能-DeepSort目标跟踪

#### 简介

本程序提供实验性的DeepSort目标跟踪

利用DeepSort稳定目标类别跟踪，可有效应对装甲板长时间丢失的情况

#### 使用

修改Config.h中注释掉的UseDeepSort项目及配套配置

重新编译即可使用

#### 注意

此为实验性功能且性能消耗极大，不保证有效效果

### Q&A

1. Q: 程序内存占用持续升高，过不了多久就会OOM怎么办？
   * A: 使用视频或高帧率相机时，图像队列可能会因为主线程无法及时将图像pop出而持续累积，应适当减少config.h中 FRAME_DEPTH 的值。
2. Q: 程序可否运行在6GB显存的显卡上？
   * A: 可以，正常运行时，程序的显存占用为5GB左右。
3. Q: 在程序初次运行时，会长时间没有输出，并抛出TRT警告。
   * A: 此为正常现象，初次运行时程序会根据设备进行onnx到engine的转化，因设备性能差异持续1～10分钟不等。
4. Q: 我是3060显卡，运行时抛出cuda failure: 2怎么办？
   * A: 检查default.yaml中路径是否合法，报错中应当存在"Engine bad file"。
   * A: 运行雷达程序时，尽可能减少其他程序对显存的占用。
5. Q: engine生成了，但是运行程序时没有检出怎么办？
   * A: 程序中所提供的onnx模型已在多台设备上通过测试，请确认CMakeLists.txt中CUDA_GEN_CODE项设置正确。
6. Q: 可否更换到自用Yolov5模型？
   * A: 可以，但需要导出动态onnx并确认单输入单输出，程序中相应修改：MapMapping.h -> _ids的映射值、Radar.h -> ids过滤id。
7. Q: 报错 error while loading shared libraries: libnvinfer.so.8: cannot open shared object file: No such file or directory…
   * A: 没有配置好Tensorrt环境，请确保环境变量配置正常
8. Q: 程序无报错退出，终止前输出Strating to read params from yaml file
   * A: 多为opencv FileStorage 错误，请尝试更换版本或重新编译opencv
9. Q: 编译时报错：找不到 -lCUDA:cublas -lCUDA:cudart...但是CUDA等已正常安装
   * A: 安装了冲突的CUDA和ceres-solver版本，临时解决办法：编译安装ceres-solver后转移或删除系统cmake目录下FindCUDAToolkit.cmake，重新编译项目
10. Q:加载模型时报错CUDA failure:100
   * A: 首次使用主机，需要在bios下禁用Security Boot[disable]
   
### Issues

1. 在控制窗口再次进入四点标定模式时会抛出窗口丢失的错误，导致窗口缩放异常无法完成标定
   1. 暂时通过重启程序来避开此问题
   2. 进入标定的控制滑条已移除
2. 在使用相机时绘制结果会发生闪烁
   1. 暂时不影响程序运行
   2. 疑似相机驱动类存在逻辑BUG
3. 异常退出的程序并不会释放显存
   1. 通过关闭终端来释放显存