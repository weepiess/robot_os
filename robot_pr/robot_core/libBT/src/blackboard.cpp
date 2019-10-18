#include "blackboard.h"

using namespace sentry_base_uestc;
using namespace robot_tool;

Blackboard::Blackboard(){}

/***************************常量key定义***************************/
//32发送信息
const string Blackboard::CURR_POS = "curr_pos";
const string Blackboard::IS_SHOOT_OFFLINE = "is_shoot_offline";
const string Blackboard::ENERGY_REMAIN = "energy_remain";
const string Blackboard::IS_CHASSIS_STOP = "is_chassis_stop";
const string Blackboard::BLOOD_REMAIN = "blood_remain";
const string Blackboard::RACE_BEGIN_TIME = "race_begin_time";
const string Blackboard::BULLET_V = "bullet_v";
const string Blackboard::OTHER_ROBOTS_COMMAND = "other_robots_command";

const string Blackboard::CURR_PITCH = "curr_pitch";
const string Blackboard::CURR_YAW = "curr_yaw";
const string Blackboard::CURR_RELATIVE_YAW = "curr_relative_yaw";
//敌方信息
const string Blackboard::ENEMY_COLOR = "enemy_color";
const string Blackboard::ENEMY_LEVEL = "enemy_level";
//状态信息
const string Blackboard::IS_DEBUG = "is_debug";
const string Blackboard::LAST_BLOOD_REMAIN = "last_blood_remain";
const string Blackboard::LAST_BULLET_V = "last_bullet_v";
const string Blackboard::TREE_RUNNING_CYCLE = "tree_running_cycle";
const string Blackboard::ROBOT_STATE = "robot_state";
const string Blackboard::WOUNDED_LEVEL = "wounded_level";
const string Blackboard::IS_LOST_BLOOD_IN_5S = "is_lost_blood_in_5s";
const string Blackboard::IS_LOST_BLOOD_IN_30S = "is_lost_blood_in_30s";
const string Blackboard::EXTRA_ENEMY_COUNTS = "extra_enemy_counts";
const string Blackboard::SB_INFO_INVALID = "sb_info_invalid";
const string Blackboard::IS_AIM_BRIDGEHEAD = "is_aim_bridgehead";
const string Blackboard::IS_RELATIVE_ANGLE_ABNORMAL = "is_relative_angle_abnormal";
const string Blackboard::IS_SAFE_BEGIN = "is_safe_begin";
//控制信息
const string Blackboard::PARTROL_MOVE_LEVEL = "partrol_move_level";
const string Blackboard::ABANDON_TRACE = "abandon_trace";
const string Blackboard::START_CRAZY_MODE = "start_crazy_mode";
const string Blackboard::SHOULD_AVOID_ENEMY_AIR = "should_avoid_enemy_air";
const string Blackboard::SHOULD_ATTACK_LONG_SHOOT_HERO = "should_attack_long_shoot_hero";
const string Blackboard::BROADEN_FILTER_ARMOR_THRESHOLD_LEVEL = "broaden_filter_armor_threshold_level";
//设备信息
const string Blackboard::CAMERA_DATA_UPDATE = "camera_data_update";
const string Blackboard::SEND_SERIAL_PATH = "send_serial_path";
const string Blackboard::RECEIVE_SERIAL_PATH = "receive_serial_path";
const string Blackboard::EXTRA_CAMERA_SERIAL_PATH = "extra_camera_serial_path";
const string Blackboard::CAMERA_CONFIG_PATH = "camera_config_path";
const string Blackboard::SENTRY_CONFIG_PATH = "sentry_config_path";
const string Blackboard::IMG_WIDTH = "img_width";
const string Blackboard::IMG_HEIGHT = "img_height";
const string Blackboard::DEVICE_OFFLINE = "device_offline";
//场地信息
const string Blackboard::TRACK_MAX_LENGTH = "track_max_length";
const string Blackboard::CURR_TRACK_START = "curr_track_start";
const string Blackboard::CURR_TRACK_END = "curr_track_end";
const string Blackboard::CURVE_ONE_START = "curve_one_start";
const string Blackboard::CURVE_ONE_END = "curve_one_end";
const string Blackboard::CURVE_TWO_START = "curve_two_start";
const string Blackboard::CURVE_TWO_END = "curve_two_end";

/***************************初始化***************************/
void Blackboard::init(const bt::BasicBlackboard::Ptr basic_blackboard_ptr_){
    basic_blackboard_ptr = basic_blackboard_ptr_;
    //串口初始化
    if(send_serial_interface.init(basic_blackboard_ptr->get<string>(SEND_SERIAL_PATH)) != 0)
        LOG_ERROR<<"send serial interface init failed!!!";
    if(receive_serial_interface.init(basic_blackboard_ptr->get<string>(RECEIVE_SERIAL_PATH)) != 0)
        LOG_ERROR<<"receive serial interface init failed!!!";
    //if(extra_camera_serial_interface.init(basic_blackboard_ptr->get<string>(EXTRA_CAMERA_SERIAL_PATH)) != 0)
    //    LOG_ERROR<<"extra camera serial interface init failed!!!";
    //状态初始化
    initState();
    //变量初始化
    detected_armors.reserve(10);
    is_debug = (unsigned char)basic_blackboard_ptr->get<unsigned>(Blackboard::IS_DEBUG);
}

void Blackboard::initState(){
    //初始常量
    basic_blackboard_ptr->set<short int>(TRACK_MAX_LENGTH, 495);
    basic_blackboard_ptr->set<short int>(CURVE_ONE_START, 65);
    basic_blackboard_ptr->set<short int>(CURVE_ONE_END, 175);
    basic_blackboard_ptr->set<short int>(CURVE_TWO_START, 320);
    basic_blackboard_ptr->set<short int>(CURVE_TWO_END, 430);

    basic_blackboard_ptr->set<short int>(CURR_TRACK_START, 0);
    basic_blackboard_ptr->set<short int>(CURR_TRACK_END, basic_blackboard_ptr->get<int>(TRACK_MAX_LENGTH));
    //初始控制信息
    basic_blackboard_ptr->set<uint8_t>(ABANDON_TRACE, 0x00); //允许追踪
    basic_blackboard_ptr->set<uint8_t>(SHOULD_AVOID_ENEMY_AIR, 0x00); //不用躲避飞机
    basic_blackboard_ptr->set<uint8_t>(SHOULD_ATTACK_LONG_SHOOT_HERO, 0x00); //不打远程吊射的英雄
    basic_blackboard_ptr->set<uint8_t>(BROADEN_FILTER_ARMOR_THRESHOLD_LEVEL, 0x00); //不放开装甲板筛选的阈值
    //初始状态信息
    basic_blackboard_ptr->set<uint8_t>(ROBOT_STATE, 0x00); //巡逻
    basic_blackboard_ptr->set<uint8_t>(PARTROL_MOVE_LEVEL, 0x01); //高速巡逻
    basic_blackboard_ptr->set<uint8_t>(START_CRAZY_MODE, 0x00); //不开启狂暴模式
    basic_blackboard_ptr->set<uint8_t>(IS_LOST_BLOOD_IN_5S, 0x00); //5s内没掉血
    basic_blackboard_ptr->set<uint8_t>(IS_LOST_BLOOD_IN_30S, 0x00); //30s内没掉血
    basic_blackboard_ptr->set<uint8_t>(EXTRA_ENEMY_COUNTS, 0x00); //敌人数量为0
    basic_blackboard_ptr->set<uint8_t>(SB_INFO_INVALID, 0x01); //树莓派信息不可用
    basic_blackboard_ptr->set<uint8_t>(IS_AIM_BRIDGEHEAD, 0x00); //当前不在瞄准桥头
    basic_blackboard_ptr->set<uint8_t>(IS_RELATIVE_ANGLE_ABNORMAL, 0x00); //当前机械角度正常
    basic_blackboard_ptr->set<uint8_t>(IS_SAFE_BEGIN, 0x00); //当前不是安全开局
    //初始设备信息
    basic_blackboard_ptr->set<uint8_t>(CAMERA_DATA_UPDATE, 0x00); //相机数据没有更新
    basic_blackboard_ptr->set<uint8_t>(DEVICE_OFFLINE, 0x00); //都没有离线
}

/***************************32数据接口***************************/
void Blackboard::updateStateInfo(short int curr_pos, unsigned char is_shoot_offline, unsigned char energy_remain, unsigned char is_chassis_stop,
        short int blood_remain, unsigned char race_begin_time, float bullet_v, unsigned char other_robots_command){
    LOG_WARNING_IF(other_robots_command != 0x00)<<" other_robots_command: "<<(int)other_robots_command;
    LOG_WARNING_EVERY(5)<<"curr_pos: "<<curr_pos<<" is_shoot_offline: "<<(int)is_shoot_offline<<" energy_remain: "<<(int)energy_remain
            <<" is_chassis_stop: "<<(int)is_chassis_stop<<" blood_remain: "<<blood_remain
            <<" race_begin_time: "<<((short int)race_begin_time)*2<<" bullet_v: "<<bullet_v;
    //位置校验
    if(curr_pos >= 0 && curr_pos <= basic_blackboard_ptr->get<int>(TRACK_MAX_LENGTH))
        basic_blackboard_ptr->set<short int>(CURR_POS, curr_pos);
    else if(!basic_blackboard_ptr->contains(CURR_POS))
        basic_blackboard_ptr->set<short int>(CURR_POS, 250);

    //射击机构是否断电校验
    if(is_shoot_offline == 0x00 || is_shoot_offline == 0x01)
        basic_blackboard_ptr->set<uint8_t>(IS_SHOOT_OFFLINE, is_shoot_offline);

    //剩余缓冲能量校验
    if(energy_remain >= 0 && energy_remain <= 250)
        basic_blackboard_ptr->set<uint8_t>(ENERGY_REMAIN, energy_remain);
    else if(!basic_blackboard_ptr->contains(ENERGY_REMAIN))
        basic_blackboard_ptr->set<uint8_t>(ENERGY_REMAIN, 200);

    //底盘是否停止校验
    if(is_chassis_stop == 0x00 || is_chassis_stop == 0x01)
        basic_blackboard_ptr->set<uint8_t>(IS_CHASSIS_STOP, is_chassis_stop);

    //剩余血量校验
    if(blood_remain > 0 && blood_remain <= 600)
        basic_blackboard_ptr->set<short int>(BLOOD_REMAIN, blood_remain);
    else if(!basic_blackboard_ptr->contains(BLOOD_REMAIN))
        basic_blackboard_ptr->set<short int>(BLOOD_REMAIN, 600);

    //比赛开始时间校验
    if(race_begin_time >= 0 && race_begin_time <= 210)
        basic_blackboard_ptr->set<short int>(RACE_BEGIN_TIME, ((short int)race_begin_time)*2);
    else if(!basic_blackboard_ptr->contains(RACE_BEGIN_TIME))
        basic_blackboard_ptr->set<short int>(RACE_BEGIN_TIME, 0);

    //子弹速度校验
    if(bullet_v >= 10 && bullet_v <= 30){
        if(basic_blackboard_ptr->contains(BULLET_V))
            basic_blackboard_ptr->set<float>(LAST_BULLET_V, basic_blackboard_ptr->get<float>(BULLET_V));
        basic_blackboard_ptr->set<float>(BULLET_V, bullet_v);
    } else if(!basic_blackboard_ptr->contains(BULLET_V))
        basic_blackboard_ptr->set<float>(BULLET_V, 28.0f);

    //机间通信校验
    if(other_robots_command == 0x00 || other_robots_command == 0x01 || other_robots_command == 0x02 || other_robots_command == 0x03)
        basic_blackboard_ptr->set<uint8_t>(OTHER_ROBOTS_COMMAND, other_robots_command);

    main_serial_active_last_time = TimeTool::getCurrTime();
}

void Blackboard::updateYuntaiAngleInfo(float curr_pitch, float curr_yaw, float curr_relative_yaw){
    LOG_WARNING_EVERY(5)<<"curr_pitch: "<<curr_pitch<<" curr_yaw: "<<curr_yaw<<" curr_relative_yaw: "<<curr_relative_yaw;
    //pitch校验
    if(curr_pitch > -45 && curr_pitch < 90)
        basic_blackboard_ptr->set<float>(CURR_PITCH, curr_pitch);
    //yaw校验
    if(!basic_blackboard_ptr->contains(CURR_YAW) || fabsf(curr_yaw - basic_blackboard_ptr->get<float>(CURR_YAW)) <= 360)
        basic_blackboard_ptr->set<float>(CURR_YAW, curr_yaw);
    //yaw机械角校验
    if(fabsf(curr_relative_yaw) <= 180)
        basic_blackboard_ptr->set<float>(CURR_RELATIVE_YAW, curr_relative_yaw);
    main_serial_active_last_time = TimeTool::getCurrTime();
}

SerialInterface& Blackboard::getSerialInterface(unsigned char serial_number){
    switch (serial_number){
        case 0x00:
            if(!is_debug && !send_serial_interface.isOpen())
                send_serial_interface.init(basic_blackboard_ptr->get<string>(SEND_SERIAL_PATH));
            return send_serial_interface;
        case 0x01:
            if(!is_debug && !receive_serial_interface.isOpen())
                receive_serial_interface.init(basic_blackboard_ptr->get<string>(RECEIVE_SERIAL_PATH));
            return receive_serial_interface;
        case 0x02:
            if(!is_debug && !extra_camera_serial_interface.isOpen())
                extra_camera_serial_interface.init(basic_blackboard_ptr->get<string>(EXTRA_CAMERA_SERIAL_PATH));    
            return extra_camera_serial_interface;
        default:
            LOG_ERROR<<"wrong serial number!";
            return receive_serial_interface;
    }
}

/***************************树莓派数据接口***************************/
void Blackboard::updateEnemyInfo(unsigned char camera_id, unsigned char enemy_counts){
    std::lock_guard<std::mutex> enemy_info_lock(sb_info_mutex); //加锁
    auto now = TimeTool::getCurrTime();
    sb_info.camera_id = camera_id;
    sb_info.enemy_counts = enemy_counts;
    sb_info.recevie_time = now;

    if(camera_id == 0x00){ //前置相机的串口
        front_camera_active_last_time = now;
    } else if(camera_id == 0x01){ //后置
        back_camera_active_last_time = now;
    }

    basic_blackboard_ptr->set<uint8_t>(SB_INFO_INVALID, 0x00);
}

SBInfo Blackboard::getSBInfo(){
    std::lock_guard<std::mutex> enemy_info_lock(sb_info_mutex); //加锁
    return sb_info;
}

/***************************相机数据接口***************************/
void Blackboard::updateCameraData(const cv::Mat& src_img_){
    std::lock_guard<std::mutex> img_lock(img_mutex); //加锁
    src_img_.copyTo(buffer_img);
    basic_blackboard_ptr->set<uint8_t>(CAMERA_DATA_UPDATE, 0x01);
}

int Blackboard::getCameraData(cv::Mat& src, bool is_need_newest, bool is_copy){
    std::lock_guard<std::mutex> img_lock(img_mutex); //加锁
    if(is_need_newest && basic_blackboard_ptr->get<uint8_t>(CAMERA_DATA_UPDATE) == 0x01){ //第一次更新后
        if(is_copy)
            buffer_img.copyTo(src);
        else
            src = buffer_img;
        buffer_img.copyTo(src_img); //保存作为行为树全局可用
        basic_blackboard_ptr->set<uint8_t>(CAMERA_DATA_UPDATE, 0x00);
        return 0;
    } else { //没有更新可能是相机没更新也可能是行为树中调用
        if(!src_img.empty()){
            if(is_copy)
                src_img.copyTo(src);
            else
                src = src_img; 
        }
        return -1;
    }
}