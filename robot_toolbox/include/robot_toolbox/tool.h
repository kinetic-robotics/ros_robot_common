#ifndef ROBOT_TOOLBOX_TOOL_H_
#define ROBOT_TOOLBOX_TOOL_H_

#include <boost/format.hpp>

#define CONFIG_ASSERT(name, cond)                                   \
    do {                                                            \
        if (!(cond)) {                                              \
            ROS_FATAL("Read parameter [%s] failed!", (static_cast<std::string>(name)).c_str()); \
            return false;                                           \
        }                                                           \
    } while (false)

/* 限制函数 */
#define LIMIT(a, min, max)        \
    if ((a) > (max)) (a) = (max); \
    if ((a) < (min)) (a) = (min);

/* 绝对值限制函数 */
#define ABS_LIMIT(a, max) LIMIT(a, -(max), (max));

/* 获取位值 */
#define GET_BIT(value, bit) (((value) >> (bit)) & 0x01)

/* 设置位值 */
#define SET_BIT(value, bit, val)                           \
    if ((val) == 0) (value) = ((~(1 << (bit))) & (value)); \
    if ((val) == 1) (value) = ((1 << (bit)) | (value));
#endif

/* 获取特定位值 */
#define GET_BITS(value, start, end) (((value) >> (start)) & ((1 << ((end) - (start) + 1)) - 1))

/* Bytes数组转float类型,小端模式 */
#define CONVERT_BYTES_TO_FLOAT(bytes, start, output)                                                      \
    {                                                                                                     \
        union {                                                                                           \
            float out;                                                                                    \
            uint32_t input;                                                                               \
        } f;                                                                                              \
        f.input = bytes[start] | bytes[start + 1] << 8 | bytes[start + 2] << 16 | bytes[start + 3] << 24; \
        output = f.out;                                                                                   \
    }
