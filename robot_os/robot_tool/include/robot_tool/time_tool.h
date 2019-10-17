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
#ifndef ROBOT_TOOL_TIME_TOOL_H
#define ROBOT_TOOL_TIME_TOOL_H
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

namespace robot_tool {

class TimeTool{
public:
    public:
        const static int MILLISECONDS;
        const static int MICROSECONDS;

    public:
        /** 获取当前时间
         * @return: std::chrono::steady_clock::time_point，可用auto接收
         */
        static std::chrono::steady_clock::time_point getCurrTime();
        static int currentTimeMsGet();
        /** 配合上面获取当前时间的函数，将之前获取的时间传入该函数中，并选择计时单位（默认ms），即可计时
         * @param: pre_time, 开始计时的时间
         * @param: time_units, 计时单位
         * @return: int64_t，过去的时间
         */
        static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& pre_time, int time_units = MILLISECONDS);

        /** 重载计时函数
         * @param: former_time, 计时的开始时间
         * @param: later_time, 计时的结束时间
         */
        static int64_t countTimeDuration(const std::chrono::steady_clock::time_point& former_time, 
            const std::chrono::steady_clock::time_point& later_time, int time_units = MILLISECONDS);
    public:
        static struct timeval start;
        static struct timeval startInitGet();
};  
};

#endif //ROBOT_TOOL_TIME_TOOL_H

