#include "../include/UARTPasser.h"

UARTPasser::UARTPasser()
{
}

UARTPasser::~UARTPasser()
{
}

int UARTPasser::bytes2Int(unsigned char a, unsigned char b)
{
    return (0x0000 | a) | (b << 8);
    // 两个字节合并成一个 16 位整数
    // 例如，如果 a 是 0x01，b 是 0x02，那么合并后的整数将是 0x0201(16进制)，也就是十进制的 513
}

float UARTPasser::bytesToFloat(unsigned char bytes[])
{
    return *((float *)bytes);
}

void UARTPasser::push_loc(vector<vector<float>> &location)
{
    this->_robot_location.swap(location);
}

vector<vector<float>> UARTPasser::get_position()
{
    return this->_robot_location;
}

void UARTPasser::get_message()
{
    // TODO:信息获取接口
}

void UARTPasser::Refree_MapLocationSelf_Message()
{
    // TODO:位置获取接口
}

void UARTPasser::Referee_Update_GameData(unsigned char *buffer)
{
    // 雷达站只需要判断比赛阶段-buffer[7]的高4位信息
    if (this->_Now_stage < 2 && ((buffer[7] >> 4) == 2 || (buffer[7] >> 4) == 3 || (buffer[7] >> 4) == 4))
    {
        this->_Game_Start_Flag = true;
        this->_set_max_flag = true;
        this->Remain_time = 420;
        this->logger->critical("GAME START !");
    }
    // TODO:是否需要读取remain_time 进行决策
    if (this->_Now_stage < 5 && (buffer[7] >> 4) == 5)
    {
        this->_Game_End_Flag = true;
        this->logger->critical("GAME FINISH !");
        for (int i = 0; i < 12; ++i)
        {
            this->_max_hp[i] = this->_init_hp[i];
        }
        this->Remain_time = 0;
    }
    this->_Now_stage = buffer[7] >> 4; // 更新
}

void UARTPasser::Referee_Robot_HP(unsigned char *buffer)
{
    for (int i = 0; i < 16; ++i)
    {
        this->_HP[i] = this->bytes2Int(buffer[(i + 4) * 2 - 1], buffer[(i + 4) * 2]); // 获取相应兵种/基地血量的低、高8位
        /*
        合并后的 16 位整数本身就是一个整数，无需任何转换即可在计算机中使用
        如果想直观的以10进制形式打印数据，可以直接使用 std::cout 输出整数变量，它会默认以十进制形式输出
        */
    }
}

// 标记进度
void UARTPasser::Mark_Enemy_Process(unsigned char *buffer)
{
    for (int i = 0; i < 6; ++i)
    {
        this->_Mark_Enemy_Process[i] = buffer[7 + i];
        if (i == 5)
        {
            std::cout << std::to_string(this->_Mark_Enemy_Process[5]) << endl;
        }
    }
}

void UARTPasser::Radar_Injury_State(unsigned char *buffer)
{
    // add 2 atrributes
    /*
    bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发双倍易伤的机会，至多为 2
    bit 2：对方是否正在被触发双倍易伤
    0：对方未被触发双倍易伤
    1：对方正在被触发双倍易伤
    */
    // 我方易伤机会
    this->_Injury_Num = (buffer[7] & 0x03); // // 提取 0-1 位:使用按位与操作将 buffer[7] 的高 6 位清零，只保留低 2 位
    // TODO:如果_Injury_Num为==0/ ==1 /==2 ,分别怎样？
    if (this->_Injury_Num > 0)
    {
        std::cout << "We have" + std::to_string(this->_Injury_Num) + "injury chance(s)" << std::endl;
    }

    // 敌方被触发易伤状态
    this->_Enemy_Injury_State = (buffer[7] & 0x04) >> 2; // 提取第 2 位:与 只保留第 2 位(其余清0)，并右移 2 位
    if (this->_Enemy_Injury_State)
    {
        std::cout << "Double Injury!" << endl;
    }
    else
    {
        std::cout << "No Injury!" << endl;
    }
}

BOData UARTPasser::One_compete_end()
{
    BOData temp;
    if (this->_Game_End_Flag)
    {
        this->_Game_End_Flag = false;
        ++this->_BO;
        temp.GameEndFlag = true;
        temp.remainBO = this->_BO - MAXBO;
    }
    return temp;
}

bool UARTPasser::One_compete_start()
{
    if (this->_Game_Start_Flag)
    {
        this->_Game_Start_Flag = false;
        return true;
    }
    else
        return false;
}

void UARTPasser::Receive_Robot_Data(unsigned char *buffer)
{
    // 获取子内容id 2-byte
    if ((0x0000 | buffer[7]) | ((buffer[8] << 8) == 0x0200)) //"==0x200是因为机器人之间的通信高8位（二进制）都是0x2_ _"
        this->logger->debug("Receive_Robot_Data");
}

// TEST
vector<vector<float>> UARTPasser::test_get_position()
{
    return this->_test_robot_location;
}
