#include "../include/UART.h"

UART::UART(int ENEMY)
{
    this->ENEMY = ENEMY;
}

UART::~UART()
{
}

void UART::Judge_Refresh_Result()
{
    this->logger->debug("Judge_Refresh_Result");
}

void UART::Referee_Game_Result()
{
    this->Game_result.winner = this->buffer[7];
}

// changed:structure
void UART::Referee_dart_status()
{
    this->Game_dart_status.dart_remaining_time = this->buffer[7];
    this->Game_dart_status.dart_info[0] = this->buffer[8];
    this->Game_dart_status.dart_info[1] = this->buffer[9];
}

void UART::Referee_event_data()
{
    this->Game_event_data.event_type[0] = this->buffer[7];
    this->Game_event_data.event_type[1] = this->buffer[8];
    this->Game_event_data.event_type[2] = this->buffer[9];
    this->Game_event_data.event_type[3] = this->buffer[10];
}

void UART::Refree_supply_projectile_action()
{
    this->Game_supply_projectile_action.supply_projectile_id = this->buffer[7];
    this->Game_supply_projectile_action.supply_robot_id = this->buffer[8];
    this->Game_supply_projectile_action.supply_projectile_step = this->buffer[9];
    this->Game_supply_projectile_action.supply_projectile_num = this->buffer[10];
}

void UART::Refree_Warning()
{
    this->Game_refree_warning.level = this->buffer[7];
    this->Game_refree_warning.foul_robot_id = this->buffer[8];
    this->Game_refree_warning.count = this->buffer[9]; // add
}

// void UART::Refree_dart_remaining_time()  //merged

// 车间通信【与哨兵】-- param in】 子内容id 我方哨兵id 数据段 串口
void UART::Referee_Transmit_BetweenCar(unsigned int dataID, unsigned char ReceiverId, unsigned char data[48], MySerial::Ptr ser)
{
    unsigned char local_buffer[200];
    /**frame_header**/
    local_buffer[0] = 0xA5;
    local_buffer[1] = (54) & 0x00ff; // 数据帧中 data 的长度
    local_buffer[2] = ((54) & 0xff00) >> 8;
    local_buffer[3] = 0; // seq
    local_buffer[4] = this->myHandler.Get_CRC8_Check_Sum(local_buffer, 5 - 1, 0xff);
    /**cmd_id**/
    local_buffer[5] = 0x01;
    local_buffer[6] = 0x03;
    /**data**/
    // 子内容id 2-byte
    // 车间通信id 0x0200-0x02FF data<=113
    // TODO:雷达自主决策id 0x0121 data=1
    local_buffer[7] = dataID & 0x00ff;
    local_buffer[8] = (dataID & 0xff00) >> 8;
    // 发送者id 2-byte
    if (this->ENEMY)
        local_buffer[9] = 9;
    else
        local_buffer[9] = 109;
    local_buffer[10] = 0;
    // 接收者id 2-byte
    local_buffer[11] = ReceiverId;
    local_buffer[12] = 0;
    // 数据段内容 x<=113
    local_buffer[13] = data[0];
    local_buffer[14] = data[1];
    local_buffer[15] = data[2];
    local_buffer[16] = data[3];
    local_buffer[17] = data[4];
    local_buffer[18] = data[5];
    local_buffer[19] = data[6];
    local_buffer[20] = data[7];
    local_buffer[21] = data[8];
    local_buffer[22] = data[9];
    local_buffer[23] = data[10];
    local_buffer[24] = data[11];
    local_buffer[25] = data[12];
    local_buffer[26] = data[13];
    local_buffer[27] = data[14];
    local_buffer[28] = data[15];
    local_buffer[29] = data[16];
    local_buffer[30] = data[17];
    local_buffer[31] = data[18];
    local_buffer[32] = data[19];
    local_buffer[33] = data[20];
    local_buffer[34] = data[21];
    local_buffer[35] = data[22];
    local_buffer[36] = data[23];
    local_buffer[37] = data[24];
    local_buffer[38] = data[25];
    local_buffer[39] = data[26];
    local_buffer[40] = data[27];
    local_buffer[41] = data[28];
    local_buffer[42] = data[29];
    local_buffer[43] = data[30];
    local_buffer[44] = data[31];
    local_buffer[45] = data[32];
    local_buffer[46] = data[33];
    local_buffer[47] = data[34];
    local_buffer[48] = data[35];
    local_buffer[49] = data[36];
    local_buffer[50] = data[37];
    local_buffer[51] = data[38];
    local_buffer[52] = data[39];
    local_buffer[53] = data[40];
    local_buffer[54] = data[41];
    local_buffer[55] = data[42];
    local_buffer[56] = data[43];
    local_buffer[57] = data[44];
    local_buffer[58] = data[45];
    local_buffer[59] = data[46];
    local_buffer[60] = data[47];

    myHandler.Append_CRC16_Check_Sum(local_buffer, 54 + 9);
    unsigned char buffer_tmp_array[54 + 9];
    for (int i = 0; i < 54 + 9; ++i)
        buffer_tmp_array[i] = local_buffer[i];
    ser->mswrite(buffer_tmp_array, 54 + 9);
}

// 裁判系统评估--【param in】 cmd_id=0x0305 协议中的真实enemy_id x y 串口
void UART::Referee_Transmit_Map(unsigned int cmdID, int targetId, float x, float y, MySerial::Ptr ser)
{
    // float to byte
    unsigned char t_x[4], t_y[4];
    this->FloatToBytes(t_x, x);
    this->FloatToBytes(t_y, y);

    unsigned char local_buffer[200]; // 创建缓冲区
    /**frame_header**/
    local_buffer[0] = 0xA5; // 数据帧起始字节0xA5
    // 数据帧中 data 的长度
    local_buffer[1] = (14) & 0x00ff;                                                 // 低八位
    local_buffer[2] = ((14) & 0xff00) >> 8;                                          // 右移-高八位
    local_buffer[3] = 0;                                                             // 包序号
    local_buffer[4] = this->myHandler.Get_CRC8_Check_Sum(local_buffer, 5 - 1, 0xff); // crc8帧头校验

    /**cmd_id 2-byte**/
    local_buffer[5] = cmdID & 0x00ff;        // 低8位
    local_buffer[6] = (cmdID & 0xff00) >> 8; // 高8位

    /**data**/
    // id 2-byte 注id<128
    local_buffer[7] = targetId; // red:1 2 3/4/5 7 blue:101 102 103/104/105 107
    local_buffer[8] = 0;        // remain
    // t_x 4-byte
    local_buffer[9] = t_x[0];
    local_buffer[10] = t_x[1];
    local_buffer[11] = t_x[2];
    local_buffer[12] = t_x[3];
    // t_y 4-byte
    local_buffer[13] = t_y[0];
    local_buffer[14] = t_y[1];
    local_buffer[15] = t_y[2];
    local_buffer[16] = t_y[3];
    // remain
    local_buffer[17] = 0;
    local_buffer[18] = 0;
    local_buffer[19] = 0;
    local_buffer[20] = 0;
    /**crc16整包校验**/
    // Append_CRC16_Check_Sum 方法，为数据帧添加CRC16校验码
    // param in: 缓冲区数据、整包大小【14+9=data+other=data+header+cmd_id+crc16】
    myHandler.Append_CRC16_Check_Sum(local_buffer, 14 + 9); // 计算crc16
    // 重建完整包（含尾部的crc16校验）
    unsigned char buffer_tmp_array[14 + 9];
    for (int i = 0; i < 14 + 9; ++i) // 重写完整包
        buffer_tmp_array[i] = local_buffer[i];
    ser->mswrite(buffer_tmp_array, 14 + 9); // 发送给串口
}

// // Origin
// //  裁判系统评估--主
// void UART::Robot_Data_Transmit_Map(MySerial::Ptr ser)
// {
//     /**格式转换**/
//     bool flag;
//     vector<float> location = myUARTPasser.get_position()[this->ind]; // 返回特定机器人的位置(x y)
//     if (location[0] == 0 && location[1] == 0)                        // （x y）== (0,0)  false
//         flag = false;
//     else
//         flag = true;
//     /**将消息通过串口发给裁判系统**/
//     if (!this->ENEMY && flag) // enemy red
//     {
//         //  cmd_id:0x0305 雷达-所有己方选手端发送enemy的坐标数据-显示到己方选手端的小地图上
//         this->Referee_Transmit_Map(0x0305, this->Id_red, _Float32(location[0]), _Float32(location[1]), ser);
//     }
//     else if (flag) // enemy blue
//     {
//         this->Referee_Transmit_Map(0x0305, this->Id_blue, _Float32(location[0]), _Float32(location[1]), ser);
//     }

//     /**更新**/
//     if (flag)
//         ++this->myUARTPasser.loop_send; //  更新计数器
//     if (this->ind == 5)
//     {
//         // if (this->myUARTPasser.loop_send == 0)
//         if (this->myUARTPasser.loop_send != 0)
//             this->myUARTPasser.loop_send = 0; // 发送完所有敌方坐标 置0
//     }
//     this->ControlLoop_red();
//     this->ControlLoop_blue();
//     this->ind = (this->ind + 1) % 6;
// }
// 裁判系统评估--主
void UART::Robot_Data_Transmit_Map(MySerial::Ptr ser)
{
    /**格式转换**/
    bool flag = true;
    vector<float> location = myUARTPasser.test_get_position()[this->ind]; // 返回特定机器人的位置(x y)
    /**将消息通过串口发给裁判系统**/
    if (!this->ENEMY && flag) // enemy red
    {
        //  cmd_id:0x0305 雷达-所有己方选手端发送enemy的坐标数据-显示到己方选手端的小地图上
        this->Referee_Transmit_Map(0x0305, this->Id_red, _Float32(location[0]), _Float32(location[1]), ser);
    }
    else if (flag) // enemy blue
    {
        this->Referee_Transmit_Map(0x0305, this->Id_blue, _Float32(location[0]), _Float32(location[1]), ser);
    }

    /**更新**/
    if (flag)
        ++this->myUARTPasser.loop_send; //  记录有效位置的个数
    if (this->ind == 5)
    {
        if (this->myUARTPasser.loop_send != 0)
            this->myUARTPasser.loop_send = 0; // 发送完所有敌方坐标 置0
    }
    this->ControlLoop_red();
    this->ControlLoop_blue();
    this->ind = (this->ind + 1) % 6;
}

void UART::ControlLoop_red() // 更新官方手册里的真实id
{
    if (this->Id_red == 5)
        this->Id_red = 7;
    else if (this->Id_red == 7)
        this->Id_red = 1;
    else
        ++this->Id_red;
}

void UART::ControlLoop_blue()
{
    if (this->Id_blue == 105)
        this->Id_blue = 107;
    else if (this->Id_blue == 107)
        this->Id_blue = 101;
    else
        ++this->Id_blue;
}

// 通过串口接收裁判系统信息 用于解析接收到的串口数据帧，并根据帧中的命令ID和数据长度进行相应的处理
void UART::read(MySerial::Ptr ser)
{
    unsigned char tempBuffer[1]; // 读取一个字节的数据到 tempBuffer
    ser->msread(tempBuffer, 1);  // 读串口数据1-byte
    int s = (int)tempBuffer[0];  // 转为整数

    // CHECK:最长的数据段只设置了50-byte 因此 若超过50肯定是错误的
    if (this->buffercnt > 50) // buffercnt用于记录缓冲区接收到的字节数
        this->buffercnt = 0;  // 缓冲区计数器重置为0

    // 把读到的data赋值给相应位置的缓冲区
    this->buffer[this->buffercnt] = s;

    // CHECK：frame_header:SOF:数据段起始位0xa5
    if (this->buffercnt == 0)
    {
        // 如果帧头不为 0xa5 错误，返回
        if (this->buffer[this->buffercnt] != 0xa5)
        {
            this->buffercnt = 0;
            return;
        }
        // 如果帧头为 0xa5，则将该字节存入缓冲区 buffer 中，并在最后将缓冲区计数器 buffercnt 加1
    }
    /** 读完frame_header 进行crc8校验**/
    if (this->buffercnt == 5)
    {
        if (myHandler.Verify_CRC8_Check_Sum(this->buffer, 5) == 0)
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                buffercnt = 1;
            return;
        }
    }

    /**解析cmd_id 并记录**/
    if (this->buffercnt == 7)
        this->cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8); // 完整的16位命令ID

    /**根据cmd_id进行相应处理**/
    // 注：有要处理信息用this->passer.func; 仅读信息用this->func()

    /**Passer**/
    // TODO:主控->对应机器人的x y angle
    if (this->buffercnt == 25 && this->cmdID == 0x0203) // if((data_length+9[other]) && cmd_id)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->myUARTPasser.Refree_MapLocationSelf_Message(); // empty
            this->buffercnt = 0;                                 // 重置 准备接收新的数据帧

            /*检查下一个字节是否是帧头 0xa5，如果是，则将 buffercnt 设置为1，以便继续接收新的数据帧的开始*/
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 比赛类型、状态
    // TODO
    if (this->buffercnt == 20 && this->cmdID == 0x0001)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 20))
        {
            this->myUARTPasser.Referee_Update_GameData(this->buffer); // 显示比赛进程
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 血量
    if (this->buffercnt == 41 && this->cmdID == 0x0003)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 41))
        {
            this->myUARTPasser.Referee_Robot_HP(this->buffer);
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // add:对敌方车辆的标记进度 0-120
    if (this->buffercnt == 15 && this->cmdID == 0x020C)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 15))
        {
            this->myUARTPasser.Mark_Enemy_Process(this->buffer);

            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // add:自主决策信息同步
    // 描述：我方雷达是否拥有易伤机会/判断对方易伤状态
    if (this->buffercnt == 10 && this->cmdID == 0x020E)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->myUARTPasser.Radar_Injury_State(this->buffer);

            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 机器人互交
    // TODO
    // 1 如果雷达站不参与，为什么要获取机器人间的通信
    // 2. 加入雷达站的自主决策命令 子内容id:0x0121
    if (this->buffercnt == 17 && this->cmdID == 0x301)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 17))
        {
            this->myUARTPasser.Receive_Robot_Data(this->buffer);
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    /**UART 读了 但没有处理**/
    // 比赛结果
    if (this->buffercnt == 10 && this->cmdID == 0x0002)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->Referee_Game_Result();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 飞镖 changed：cmd_id
    if (this->buffercnt == 12 && this->cmdID == 0x0105)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 12))
        {
            this->Referee_dart_status();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 场地事件
    if (this->buffercnt == 13 && this->cmdID == 0x0101)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->Referee_event_data();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 补给站动作
    if (this->buffercnt == 13 && this->cmdID == 0x0102)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->Refree_supply_projectile_action();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    // 警告 changed:data_length
    if (this->buffercnt == 12 && this->cmdID == 0x0104)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 11))
        {
            this->Refree_Warning();
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    /**不处理**/
    // 低盘功率、枪口热量 不处理
    if (this->buffercnt == 25 && this->cmdID == 0x202)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // TODO
    //  机器人位置数据 不处理
    if (this->buffercnt == 25 && this->cmdID == 0x203)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 25))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 性能 不处理
    if (this->buffercnt == 27 && this->cmdID == 0x201)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 27))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 增益 不处理
    if (this->buffercnt == 10 && this->cmdID == 0x204)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 伤害 不处理
    if (this->buffercnt == 10 && this->cmdID == 0x206)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 10))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // RFID 不处理
    if (this->buffercnt == 13 && this->cmdID == 0x209)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 13))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // TODO
    //  机器人交互数据 不处理
    if (this->buffercnt == 20 && this->cmdID == 0x0301)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 20))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // TODO
    // 地面机器人位置-服务器->哨兵 不处理
    if (this->buffercnt == 36 && this->cmdID == 0x020B)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 36))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }

    // 警告 不处理
    if (this->buffercnt == 4 && this->cmdID == 0x0104)
    {
        if (myHandler.Verify_CRC16_Check_Sum(this->buffer, 4))
        {
            this->buffercnt = 0;
            if (this->buffer[this->buffercnt] == 0xa5)
                this->buffercnt = 1;
            return;
        }
    }
    ++this->buffercnt;
}

// 发送雷达站处理的信息
void UART::write(MySerial::Ptr ser)
{
    // func1
    // 雷达站敌方车辆位置--by 串口--裁判系统--选手端小地图
    this->Robot_Data_Transmit_Map(ser);

    unsigned int dataID = 0x0200 + (1 & 0xFF); // cmd_id 0x0301 子内容id 0x0200~0x02FF
    unsigned char receiverId;
    if (this->ENEMY)    // red enemy
        receiverId = 7; // [我方]红方哨兵
    else
        receiverId = 107; // [我方]蓝方哨兵
    unsigned char data[48];
    vector<vector<float>> positions = myUARTPasser.get_position(); // all enemy
    for (int i = 0; i < 6; ++i)
    {
        unsigned char t_x[4], t_y[4];
        this->FloatToBytes(t_x, positions[i][0]);
        this->FloatToBytes(t_y, positions[i][1]);
        int data_p = i * 8;
        data[data_p + 0] = t_x[0];
        data[data_p + 1] = t_x[1];
        data[data_p + 2] = t_x[2];
        data[data_p + 3] = t_x[3];
        data[data_p + 4] = t_y[0];
        data[data_p + 5] = t_y[1];
        data[data_p + 6] = t_y[2];
        data[data_p + 7] = t_y[3];
    }
    // func2
    //  给哨兵发敌方的位置
    Referee_Transmit_BetweenCar(dataID, receiverId, data, ser);

    usleep(100000); // 100 000us-100ms--0.1s--10Hz
}