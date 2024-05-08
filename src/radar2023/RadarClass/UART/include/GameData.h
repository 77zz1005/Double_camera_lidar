#ifndef __GAMEDATA_H
#define __GAMEDATA_H

#include "../../Common/include/public.h"

/**
 * @brief 比赛数据类
 * 官方裁判系统Demo的C++版本
 */
/****************************read & add *****************************/
// add: 雷达站对敌方车辆的标记进度
class radar_mark_data
{
    unsigned char mark_hero_progress = 0U;
    unsigned char mark_engineer_progress = 0U;
    unsigned char mark_standard_3_progress = 0U;
    unsigned char mark_standard_4_progress = 0U;
    unsigned char mark_standard_5_progress = 0U;
    unsigned char mark_sentry_progress = 0U;
};

// add:判断雷达是否有易伤机会
// 0-1 bit: num: 开局为 0,至多为 2
// 2 bit: state: 0未触发 1触发
// 3-7 bit: remain
class radar_info
{
    unsigned char radar_info = 0U;
};

// add：雷达是否确认触发双倍易伤
// TODO:自主决策
class radar_cmd
{
    unsigned char radar_cmd = 0U;
};
/******************************read*******************************/

 // changed
class game_state
{
public:
    unsigned char game_type = 4;
    unsigned char game_progress = 4;
    unsigned char stage_remain_time[2] = {0U, 0U};
    unsigned char SyncTimeStamp[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}; // add
};

class game_result
{
public:
    unsigned char winner = 0U;
};

class game_robot_HP
{
public:
    unsigned char red_1_robot_HP[2] = {0U, 0U};
    unsigned char red_2_robot_HP[2] = {0U, 0U};
    unsigned char red_3_robot_HP[2] = {0U, 0U};
    unsigned char red_4_robot_HP[2] = {0U, 0U};
    unsigned char red_5_robot_HP[2] = {0U, 0U};
    unsigned char red_7_robot_HP[2] = {0U, 0U};
    unsigned char red_outpost_HP[2] = {0U, 0U};
    unsigned char red_base_HP[2] = {0U, 0U};
    unsigned char blue_1_robot_HP[2] = {0U, 0U};
    unsigned char blue_2_robot_HP[2] = {0U, 0U};
    unsigned char blue_3_robot_HP[2] = {0U, 0U};
    unsigned char blue_4_robot_HP[2] = {0U, 0U};
    unsigned char blue_5_robot_HP[2] = {0U, 0U};
    unsigned char blue_7_robot_HP[2] = {0U, 0U};
    unsigned char blue_outpost_HP[2] = {0U, 0U};
    unsigned char blue_base_HP[2] = {0U, 0U};
};

// 飞镖 changed
class dart_status
{
public:
    unsigned char dart_remaining_time = 0U; // changed
    unsigned char dart_info[2] = {0U,0U};     // changed
};

class event_data
{
public:
    unsigned char event_type[4] = {0U, 0U, 0U, 0U};
};

class supply_projectile_action
{
public:
    unsigned char supply_projectile_id = 0U;
    unsigned char supply_robot_id = 0U;
    unsigned char supply_projectile_step = 0U;
    unsigned char supply_projectile_num = 0U;
};

// changed
class refree_warning
{
public:
    unsigned char level = 0U;
    unsigned char foul_robot_id = 0U;
    unsigned char count = 0U;  //add
};

// class dart_remaining_time


/******************************write*******************************/
class robot_location
{
public:
    vector<vector<float>> loc;

public:
    // x y
    //{{0,0,0,0,0},{0,0,0,0,0}}
    robot_location()
    {
        vector<vector<float>> temp(2, vector<float>(5, 0.f));
        this->loc.swap(temp);
    };

    void push(vector<vector<float>> loc)
    {
        this->loc.swap(loc);
    };
};

//TODO
/******************************疑似无需定义*******************************/
/*
注：机器人交互数据包的总长不超过 128 个字节；数据段最大为113 个字节
*/
// add:多机通信
// TODO：雷达站的辅助功能
// 0x0200~0x02FF x≤113 机器人之间通信
// 0x0121 1 雷达自主决策指令 maximal cmd_id frequency = 10Hz
// class robot_interaction_data
// {
//     unsigned char data_cmd_id[2] = {0U, 0U};
//     unsigned char sender_id[2] = {0U, 0U};
//     unsigned char receiver_id[2] = {0U, 0U};
//     unsigned char user_data[50]; // self-defined, but maximum = 113
// };

// TODO: 但是原来就有这个功能，这里是否无需定义？
// add: 雷达-服务器-选手端：0x0305
class map_robot_data
{
    unsigned char robot_id[2] = {0U, 0U};
    unsigned char target_position_x[4] = {0U, 0U, 0U, 0U};
    unsigned char target_position_y[4] = {0U, 0U, 0U, 0U};
};

/******************************no use*******************************/
//to discard
class graphic_data_struct
{
public:
    vector<unsigned char> data;
    int datalength = 0U;
    unsigned char graphic_name[4] = {0U, 0U, 0U, 0U};
    unsigned char operate_tpye[4] = {0U, 0U, 0U, 0U};
    unsigned char graphic_tpye[4] = {0U, 0U, 0U, 0U};
    unsigned char layer[4] = {0U, 0U, 0U, 0U};
    unsigned char color[4] = {0U, 0U, 0U, 0U};
    unsigned char start_angle[4] = {0U, 0U, 0U, 0U};
    unsigned char end_angle[4] = {0U, 0U, 0U, 0U};
    unsigned char width[4] = {0U, 0U, 0U, 0U};
    unsigned char start_x[4] = {0U, 0U, 0U, 0U};
    unsigned char start_y[4] = {0U, 0U, 0U, 0U};
    unsigned char radius[4] = {0U, 0U, 0U, 0U};
    unsigned char end_x[4] = {0U, 0U, 0U, 0U};
    unsigned char end_y[4] = {0U, 0U, 0U, 0U};

public:
    void Add()
    {
        vector<unsigned char> temp;
        this->data.swap(temp);
        this->datalength = 15;
    };
};

class custom_data
{
public:
    unsigned char data1[4] = {0U, 0U, 0U, 0U};
    unsigned char data2[4] = {0U, 0U, 0U, 0U};
    unsigned char data3[4] = {0U, 0U, 0U, 0U};
    unsigned char masks = 0U;
};

#endif