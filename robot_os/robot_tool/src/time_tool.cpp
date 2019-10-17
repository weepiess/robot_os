/****************************************************************************
 *  Copyright (C) 2019 UESTC RoboMaster .
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  file  : time_tool.h
 *  brief : 时间工具类
 *  author: fwc
 *  date  : 2019-6-13
 ***************************************************************************/
#include "robot_tool/time_tool.h"
using namespace std;
using namespace robot_tool;

const int TimeTool::MICROSECONDS = 0;
const int TimeTool::MILLISECONDS = 1;

struct timeval TimeTool::start=startInitGet();

struct timeval TimeTool::startInitGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv;
}
std::chrono::steady_clock::time_point TimeTool::getCurrTime(){
    return std::chrono::steady_clock::now();
}

int TimeTool::currentTimeMsGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (int)((tv.tv_sec-start.tv_sec)*1000+(tv.tv_usec-start.tv_usec)/1000);
}

int64_t TimeTool::countTimeDuration(const std::chrono::steady_clock::time_point& pre_time, int time_units){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if(time_units == MICROSECONDS){
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(now - pre_time);
        return duration.count();
    } else if(time_units == MILLISECONDS){
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - pre_time);
        return duration.count();
    }
}

int64_t TimeTool::countTimeDuration(const std::chrono::steady_clock::time_point& former_time, 
    const std::chrono::steady_clock::time_point& later_time, int time_units){
    if(time_units == MICROSECONDS){
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(later_time - former_time);
        return duration.count();
    } else if(time_units == MILLISECONDS){
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(later_time - former_time);
        return duration.count();
    }
}
