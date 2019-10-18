#ifndef BT_BLACKBOARD_H
#define BT_BLACKBOARD_H

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <chrono>
#include <queue>
#include <atomic>

#include "robot_tool/log_tool.h"
#include "robot_base_pr/serial_interface.h"
#include "blackboard/blackboard_local.h"
#include "opencv2/core/core.hpp"
#include "robot_tool/time_tool.h"

using namespace std;

//装甲板
struct Armor{
    bool is_hero;
    cv::RotatedRect light_bar_pairs[2];
    cv::Point3d pos;

    Armor(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2, const cv::Point3d& pos_, bool is_hero_ = false):
        pos(pos_), is_hero(is_hero_){
        light_bar_pairs[0] = rect1;
        light_bar_pairs[1] = rect2;
    }
};

//树莓派发送的信息
struct SBInfo{
    unsigned char camera_id;
    unsigned char enemy_counts;
    std::chrono::steady_clock::time_point recevie_time;

    SBInfo():camera_id(0xff), enemy_counts(0){}
    SBInfo(const unsigned char& camera_id_, const unsigned char& enemy_counts_, const std::chrono::steady_clock::time_point recevie_time_):
        camera_id(camera_id_), enemy_counts(enemy_counts_), recevie_time(recevie_time_){}
};

class Blackboard{
public:
    Blackboard();

    virtual ~Blackboard() noexcept = default;

    typedef std::shared_ptr<Blackboard> Ptr;

public:
    //初始化
    void init(const bt::BasicBlackboard::Ptr basic_blackboard_ptr_);

    /*************************基本数据类型统一接口**********************/
    //模板方法声明和定义必须在一起
    template <typename T>
    bool get(const std::string& key, T& value){
        if(key == DEVICE_OFFLINE){ //串口离线信息
            std::lock_guard<std::mutex> device_offline_lock(device_offline_mutex); //加锁
            value = basic_blackboard_ptr->get<T>(DEVICE_OFFLINE);
            return true;
        }
        return basic_blackboard_ptr->get<T>(key, value);
    }

    template <typename T>
    T get(const std::string& key){
        T value;
        bool found = get<T>(key, value);
        if (!found){
            throw std::runtime_error("Missing key");
        }
        return value;
    }

    template <typename T>
    void set(const std::string& key, const T& value){
        if(basic_blackboard_ptr){
            if(key == DEVICE_OFFLINE){
                std::lock_guard<std::mutex> device_offline_lock(device_offline_mutex); //加锁
                basic_blackboard_ptr->set<T>(DEVICE_OFFLINE, value);
                return;
            }
            basic_blackboard_ptr->set<T>(key, value);
        }
    }

    bool contains(const std::string& key) const{
        return basic_blackboard_ptr && basic_blackboard_ptr->contains(key);
    }

    /***************************非基本数据类型接口***************************/
    //32接口，串口编号：0x00为发送串口，0x01为接收串口，0x02为额外相机串口
    void updateStateInfo(short int curr_pos, unsigned char is_shoot_offline, unsigned char energy_remain, unsigned char is_chassis_stop,
        short int blood_remain, unsigned char race_begin_time, float bullet_v, unsigned char other_robots_command);
    void updateYuntaiAngleInfo(float curr_pitch, float curr_yaw, float curr_relative_yaw);

    sentry_base_uestc::SerialInterface& getSerialInterface(unsigned char serial_number);

    //树莓派接口
    void updateEnemyInfo(unsigned char camera_id, unsigned char enemy_counts);

    SBInfo getSBInfo();

    //相机接口
    void updateCameraData(const cv::Mat& src_img_);

    int getCameraData(cv::Mat& src, bool is_need_newest, bool is_copy = false);

private:
    //初始化基本类型
    void initState();

    //基本数据类型的blackboard指针
    bt::BasicBlackboard::Ptr basic_blackboard_ptr;

    //当前是否处于调试模式
    bool is_debug;

    /******************串口数据接口******************/
    //硬件信息
    sentry_base_uestc::SerialInterface send_serial_interface;
    sentry_base_uestc::SerialInterface receive_serial_interface;
    sentry_base_uestc::SerialInterface extra_camera_serial_interface;

    //互斥量
    std::mutex sb_info_mutex; //额外相机
    std::mutex device_offline_mutex; //串口离线信息

    //树莓派发送信息，由于树莓派串行发送，每帧数据间隔100ms+，所以不存在冲突问题
    //由于该信息是整体获取，所以没必要拆分成为基本类型
    SBInfo sb_info;

    /******************相机数据接口******************/
    //相机图像锁
    std::mutex img_mutex;

    //相机图像
    cv::Mat src_img, buffer_img;

public:
    /*******************行为树共有数据*******************/
    //非基本类型，尽可能少地使用非基本类型，尽量使用形式统一的基本类型
    vector<Armor> detected_armors; //检测到的装甲板

    //记录串口数据的接收时间，用以判断串口离线情况
    std::chrono::steady_clock::time_point main_serial_active_last_time;
    std::chrono::steady_clock::time_point front_camera_active_last_time;
    std::chrono::steady_clock::time_point back_camera_active_last_time;

    //记录上次的射击时间
    std::chrono::steady_clock::time_point last_shoot_time;

    /*******************静态常量key*******************/
    //32发送信息
    const static string CURR_POS;
    const static string IS_SHOOT_OFFLINE;
    const static string ENERGY_REMAIN;
    const static string IS_CHASSIS_STOP;
    const static string BLOOD_REMAIN;
    const static string RACE_BEGIN_TIME;
    const static string BULLET_V;
    const static string OTHER_ROBOTS_COMMAND;

    const static string CURR_PITCH;
    const static string CURR_YAW;
    const static string CURR_RELATIVE_YAW; //当前机械角度

    //敌方信息
    //敌方颜色
    const static string ENEMY_COLOR;
    //敌方等级，0x00为远距离步兵，0x01为非远距离步兵，0x02为脚底下步兵，0x03为远距离英雄，0x04为非远距离英雄，0x05为脚底下英雄
    const static string ENEMY_LEVEL;

    //状态信息
    //是否处于debug模式
    const static string IS_DEBUG;
    //上次的血量剩余，用于判断是否被打
    const static string LAST_BLOOD_REMAIN;
    //上次的射速信息，用于剔除异常的射速
    const static string LAST_BULLET_V;
    //行为树的运行周期（理论上）
    const static string TREE_RUNNING_CYCLE;
    //机器人状态，0x00为巡逻，0x01为跟踪并射击英雄，0x02为追踪并射击工程，0x03为跟踪并射击步兵，0x04为被打应激，
    //0x03为不知道编号但是应该是步兵，0x33为跟踪并射击3号步兵，0x43为4号步兵，0x53为5号步兵
    const static string ROBOT_STATE;
    //被打掉血等级，0x00为英雄打击掉血，0x01为步兵打击掉血
    const static string WOUNDED_LEVEL;
    //5s内是否被打
    const static string IS_LOST_BLOOD_IN_5S;
    //30s内是否被打
    const static string IS_LOST_BLOOD_IN_30S;
    //额外相机获取的敌人数量信息，二进制排列，低4位代表前置相机获取的，高4位代表后置相机获取的
    const static string EXTRA_ENEMY_COUNTS;
    //树莓派信息是否不可用
    const static string SB_INFO_INVALID;
    //是否正在扫描桥头
    const static string IS_AIM_BRIDGEHEAD;
    //当前是否是机械角异常状态
    const static string IS_RELATIVE_ANGLE_ABNORMAL;
    //当前是否是安全开局
    const static string IS_SAFE_BEGIN;

    //控制信息
    //巡逻等级，0x00为低速巡逻，0x01为高速巡逻，0x02为超高速巡逻
    const static string PARTROL_MOVE_LEVEL;
    //放弃追踪
    const static string ABANDON_TRACE;
    //狂暴模式，当紧急情况时开启，0x01为开启，0x00为关闭
    const static string START_CRAZY_MODE;
    //是否应该躲避敌方飞机
    const static string SHOULD_AVOID_ENEMY_AIR;
    //是否应该打吊射的英雄
    const static string SHOULD_ATTACK_LONG_SHOOT_HERO;
    //放开装甲板筛选的阈值，0x00为不放开，0x01为轻微放开，0x02为大量放开
    const static string BROADEN_FILTER_ARMOR_THRESHOLD_LEVEL;

    //设备信息
    const static string CAMERA_DATA_UPDATE; //相机更新
    const static string SEND_SERIAL_PATH; //发送串口路径
    const static string RECEIVE_SERIAL_PATH; //接收串口路径
    const static string EXTRA_CAMERA_SERIAL_PATH; //额外相机路径
    const static string CAMERA_CONFIG_PATH; //相机配置文件路径
    const static string SENTRY_CONFIG_PATH; //自瞄配置文件路径
    const static string IMG_WIDTH;
    const static string IMG_HEIGHT;
    //串口挂载的设备离线，按照二进制的排列，最低位为相机离线，后面离线为自定义
    //自定义：从低位向高位依次为主串口，前置相机和后置相机，例如0000 0011为相机和主串口离线
    const static string DEVICE_OFFLINE;

    //场地信息
    const static string TRACK_MAX_LENGTH;
    const static string CURVE_ONE_START;
    const static string CURVE_ONE_END;
    const static string CURVE_TWO_START;
    const static string CURVE_TWO_END;

    const static string CURR_TRACK_START;
    const static string CURR_TRACK_END;

}; //class

#endif //BT_BLACKBOARD