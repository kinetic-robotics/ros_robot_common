#ifndef ROBOT_TOOLBOX_CRC_H_
#define ROBOT_TOOLBOX_CRC_H_

#include <ros/ros.h>

namespace robot_toolbox
{
    namespace CRC {
        /**
         * 计算CRC8校验码
         * 
         * @param data 数据
         * @return 校验码
         */
        uint8_t getCRC8(std::vector<uint8_t>& data);
    
        /**
         * 计算CRC16校验码
         * 
         * @param data 数据
         * @return 校验码
         */
        uint16_t getCRC16(std::vector<uint8_t>& data);
    }  // namespace CRC
}  // namespace robot_toolbox

#endif