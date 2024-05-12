# UART: 裁判系统模块 | 串口模块

## 文件结构

├── include
│   ├── GameData.h			//游戏数据定义头文件
│   ├── offical_Judge_Handler.h	//串口数据处理头文件
│   ├── serial.h				//串口头文件
│   ├── UART.h				//裁判系统头文件
│   └── UARTPasser.h		//裁判系统数据传递头文件
└── src
     ├── offical_judge_Handler.cpp	//串口数据处理cpp文件
     ├── serial.cpp			//串口cpp文件
     ├── UART.cpp			//裁判系统cpp文件
     └── UARTPasser.cpp		//裁判系统数据传递cpp文件

## 简介

基于官方裁判系统Demo修改，提供C++版本，根据需求可进行适当定制

## 修改日志 2024 V1.6.1
5.1：
（1）在UART::read()和处理串口数据的函数中添加了一些输出信息
（2）在UART的头文件、源文件中添加测试发信息的函数Test_Referee_Transmit_Map、 Test_Robot_Data_Transmit_Map 
注：发送串口数据需要测试2个功能：
     ·雷达站将enemy信息发给选手端小地图（basic）
     ·雷达站通过车间通信将所有识别到的地面车辆的信息发给哨兵*(additional)
### 接收数据：

#### modify:
1.飞镖结构体相关√
#### add:
基本流程：定义类--UART添加属性--UART::read()修改--添加读buffer的函数  
额外函数：是否添加额外功能（UART::func or UARTPasser::func）  
1.雷达易伤状态信息同步 √<br>
2.雷达标记进度 √<br>
#### TODO:
1. 0x301的子数据id好像包含雷达的自主决策
2. 在UART::read()中添加了一些输出信息，为了验证读信息是否成功
