/**
 * @file      : serial.h
 * @brief     : Linux平台串口驱动头文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 14:27:38
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-01-18 huenrong        创建文件
 *
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <termios.h>

// 串口波特率
typedef enum
{
    E_SERIAL_BAUD_RATE_0 = B0,
    E_SERIAL_BAUD_RATE_50 = B50,
    E_SERIAL_BAUD_RATE_75 = B75,
    E_SERIAL_BAUD_RATE_110 = B110,
    E_SERIAL_BAUD_RATE_134 = B134,
    E_SERIAL_BAUD_RATE_150 = B150,
    E_SERIAL_BAUD_RATE_200 = B200,
    E_SERIAL_BAUD_RATE_300 = B300,
    E_SERIAL_BAUD_RATE_600 = B600,
    E_SERIAL_BAUD_RATE_1200 = B1200,
    E_SERIAL_BAUD_RATE_1800 = B1800,
    E_SERIAL_BAUD_RATE_2400 = B2400,
    E_SERIAL_BAUD_RATE_4800 = B4800,
    E_SERIAL_BAUD_RATE_9600 = B9600,
    E_SERIAL_BAUD_RATE_19200 = B19200,
    E_SERIAL_BAUD_RATE_38400 = B38400,
    E_SERIAL_BAUD_RATE_57600 = B57600,
    E_SERIAL_BAUD_RATE_115200 = B115200,
    E_SERIAL_BAUD_RATE_230400 = B230400,
    E_SERIAL_BAUD_RATE_460800 = B460800,
    E_SERIAL_BAUD_RATE_500000 = B500000,
    E_SERIAL_BAUD_RATE_576000 = B576000,
    E_SERIAL_BAUD_RATE_921600 = B921600,
    E_SERIAL_BAUD_RATE_1000000 = B1000000,
    E_SERIAL_BAUD_RATE_1152000 = B1152000,
    E_SERIAL_BAUD_RATE_1500000 = B1500000,
    E_SERIAL_BAUD_RATE_2000000 = B2000000,
    E_SERIAL_BAUD_RATE_2500000 = B2500000,
    E_SERIAL_BAUD_RATE_3000000 = B3000000,
    E_SERIAL_BAUD_RATE_3500000 = B3500000,
    E_SERIAL_BAUD_RATE_4000000 = B4000000,
    // 特殊波特率
    E_SERIAL_BAUD_RATE_SPECIAL = 0xFFFFFFFF,
} serial_baud_rate_e;

// 串口数据位
typedef enum
{
    E_SERIAL_DATA_BIT_5 = CS5,
    E_SERIAL_DATA_BIT_6 = CS6,
    E_SERIAL_DATA_BIT_7 = CS7,
    E_SERIAL_DATA_BIT_8 = CS8,
} serial_data_bit_e;

// 串口奇偶校验位
typedef enum
{
    // 无校验
    E_SERIAL_PARITY_BIT_N = 'n',
    // 奇校验
    E_SERIAL_PARITY_BIT_O = 'o',
    // 偶校验
    E_SERIAL_PARITY_BIT_E = 'e',
} serial_parity_bit_e;

// 串口停止位
typedef enum
{
    E_SERIAL_STOP_BIT_1 = 1,
    E_SERIAL_STOP_BIT_2 = 2,
} serial_stop_bit_e;

/**
 * @brief  打开串口
 * @param  serial_dev_name  : 输入参数, 串口设备名(例: /dev/ttyS1)
 * @param  std_baud_rate    : 输入参数, 标准波特率(若使用特殊波特率必须填SERIAL_BAUD_RATE_SPECIAL)
 * @param  special_baud_rate: 输入参数, 特殊波特率(不使用填任意值)
 * @param  data_bit         : 输入参数, 数据位
 * @param  parity_bit       : 输入参数, 奇偶检验位, 默认为无校验'n'或'N'
 * @param  stop_bit         : 输入参数, 停止位, 默认为1位停止位
 * @return true : 成功
 * @return false: 失败
 */
bool serial_open(const char *serial_dev_name, const serial_baud_rate_e std_baud_rate, const int special_baud_rate,
                 const serial_data_bit_e data_bit, const serial_parity_bit_e parity_bit,
                 const serial_stop_bit_e stop_bit);

/**
 * @brief  关闭串口
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_close(const char *serial_dev_name);

/**
 * @brief  清空串口输入缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_input_cache(const char *serial_dev_name);

/**
 * @brief  清空串口输出缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_output_cache(const char *serial_dev_name);

/**
 * @brief  清空串口输入和输出缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_both_cache(const char *serial_dev_name);

/**
 * @brief  串口发送数据
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @param  send_data      : 输入参数, 待发送数据
 * @param  send_data_len  : 输入参数, 待发送数据长度
 * @return 成功: 实际发送数据长度
 * @return 失败: -1
 */
int serial_write_data(const char *serial_dev_name, const uint8_t *send_data, const uint32_t send_data_len);

/**
 * @brief  串口接收数据
 * @param  recv_data      : 输出参数, 接收数据
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @param  recv_data_len  : 输入参数, 接收数据长度
 * @param  timeout:       : 输入参数, 接收超时(单位：ms)
 * @return 成功: 实际接收数据长度
 *         失败: -1
 */
int serial_read_data(uint8_t *recv_data, const char *serial_dev_name, const size_t recv_data_len, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_H
