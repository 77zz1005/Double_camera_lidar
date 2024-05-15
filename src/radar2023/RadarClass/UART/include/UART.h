#ifndef UART_H
#define UART_H

#include "../../Common/include/public.h"
#include "../include/offical_Judge_Handler.h"
#include "../include/GameData.h"
#include "../include/serial.h"
#include "../include/UARTPasser.h"

/**
 * @brief UART
 * 官方裁判系统Demo的C++版本
 */
class UART
{
public:
    typedef std::shared_ptr<UART> Ptr;

    int ENEMY;

    int ind = 0;
    int Id_red = 1;
    int Id_blue = 101;

    int buffercnt = 0;
    unsigned char buffer[1000] = {0U};
    unsigned int cmdID = 0;
    // int indecode = 0;  //discard

    UARTPasser myUARTPasser;         // To process data received
    Offical_Judge_Handler myHandler; // crc

    /*** just read ***/
    game_state Game_state; // ？ discard:moved to UARTPasser
    game_result Game_result;
    game_robot_HP Game_robot_HP;  // ？ discard:moved to UARTPasser
    dart_status Game_dart_status; // changed
    event_data Game_event_data;
    supply_projectile_action Game_supply_projectile_action;
    refree_warning Game_refree_warning;
    // dart_remaining_time Game_dart_remaining_time; //merged into dart_status

    /*** add ***/
    radar_mark_data Radar_mark_data; // 雷达站对敌方的标记进度
    // TODO:雷达站自主触发易伤机制->0 1 2
    radar_cmd Radar_cmd;

    // TODO:雷达站是否可以与英雄、哨兵通信
    // 可以 但属于需要发送的内容，发的内容不需要定义，但要符合格式
    // delete:robot_interaction_data Robot_interaction_data;

    //?选手端的数据需要删吗？
    map_robot_data Map_robot_data;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    // 联合体 FloatAndByte 可以在浮点数和字节之间进行转换，因为它们共享同一块内存空间
    union FloatAndByte
    {
        float union_float;
        unsigned char union_byte[4];
    } FAB; // FAB是联合体的实例

    void FloatToBytes(unsigned char *data, float float_to_byte)
    {
        FAB.union_float = float_to_byte;
        for (int i = 0; i < 4; i++)
        {
            data[i] = FAB.union_byte[i];
        }
    }

    /*** just read ***/
    void Judge_Refresh_Result();
    void Referee_Game_Result();
    void Referee_dart_status();
    void Referee_event_data();
    void Refree_supply_projectile_action();
    void Refree_Warning();
    // void Refree_dart_remaining_time(); // merged into Referee_dart_status()

    /*** send ***/
    void Radar_DoubleInjury_cmd(unsigned int dataID, unsigned char ReceiverId, unsigned char data[48], MySerial::Ptr ser);
    void Referee_Transmit_BetweenCar(unsigned int dataID, unsigned char ReceiverId, unsigned char data[48], MySerial::Ptr ser);
    void Referee_Transmit_Map(unsigned int cmdID, int targetId, float x, float y, MySerial::Ptr ser);
    void Robot_Data_Transmit_Map(MySerial::Ptr ser);

public:
    UART(int ENEMY);
    ~UART();

    void ControlLoop_red();
    void ControlLoop_blue();
    void read(MySerial::Ptr ser);
    void write(MySerial::Ptr ser);
};

#endif