#ifndef UARTPASSER_H
#define UARTPASSER_H

#include "../../Common/include/public.h"

/**
 * @brief UART传递
 * 官方裁判系统Demo的C++版本
 * 负责处理裁判系统数据
 */
class UARTPasser
{
private:
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    int _hp_up[9] = {100, 150, 200, 250, 300, 350, 400, 450, 500};
    int _init_hp[12] = {500};
    int _last_hp[12] = {500};
    int _HP[16] = {500};
    int _max_hp[12] = {500};
    bool _set_max_flag = false;
    vector<vector<float>> _robot_location = vector<vector<float>>(6, vector<float>(2, 0.f));
    int _BO = 0;
    vector<string> _stage = {"NOT START", "PREPARING", "CHECKING", "5S", "PLAYING", "END"};
    int _Now_stage = 0;
    bool _Game_Start_Flag = false;
    bool _Game_End_Flag = false;
    int Remain_time = 1;
    int _HP_thres = 10;
    float _prevent_time[6] = {2.f, 2.f, 2.f, 2.f, 2.f, 2.f};
    float _event_prevent[6] = {0.f};
    int loop_send = 0;

#ifdef Referee_sys_Test
    vector<vector<float>> _test_robot_location = vector<vector<float>>{{0.0f, 0.0f}, {25.0 / 5.0, -18.0 / 5.0}, {25.0 / 5.0 * 2.0, -18.0 / 5.0 * 2.0}, {25.0 / 5.0 * 3.0, -18.0 / 5.0 * 3.0}, {25.0 / 5.0 * 4.0, -18.0 / 5.0 * 4.0}, {25.0, -18.0}};
#endif
    // add: radar
    // 双倍易伤
    int _Injury_Num = 0;              // 雷达是否有易伤的机会 开局为0,至多为2
    bool _Enemy_Injury_State = false; // 0未触发 1触发
    // 标记进度:英雄 工程 步兵3/4/5 哨兵
    int _Mark_Enemy_Process[6] = {0};

public:
    UARTPasser();
    ~UARTPasser();

    int bytes2Int(unsigned char a, unsigned char b);
    float bytesToFloat(unsigned char bytes[]);

    void push_loc(vector<vector<float>> &location);
    vector<vector<float>> get_position();
    void get_message();
    void Refree_MapLocationSelf_Message();
    void Referee_Update_GameData(unsigned char *buffer);
    void Referee_Robot_HP(unsigned char *buffer);
    BOData One_compete_end();
    bool One_compete_start();
    void Receive_Robot_Data(unsigned char *buffer);
    void Radar_Injury_State(unsigned char *buffer); // 全场双倍易伤状态
    void Mark_Enemy_Process(unsigned char *buffer); // 标记进度
    // TO TEST
    vector<vector<float>> test_get_position();
};

#endif