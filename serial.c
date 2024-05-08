/**
 * @file      : serial.c
 * @brief     : Linux平台串口驱动源文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 14:28:05
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-02-05 huenrong        1. 删除无用变量
 *                                         2. 修改函数调用错误问题
 *              2023-01-18 huenrong        创建文件
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <pthread.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <assert.h>

#include "./serial.h"

// 串口设备名最大长度
#define SERIAL_DEV_NAME_MAX_LEN 15

// 串口设备最大数量
#define SERIAL_DEV_MAX_NUM 10

// 串口信息结构体
typedef struct
{
    char serial_dev_name[SERIAL_DEV_NAME_MAX_LEN]; // 串口设备名
    int serial_dev_fd;                             // 串口设备文件描述符
    pthread_mutex_t serial_dev_mutex;              // 串口设备互斥锁
} serial_dev_info_t;

// 已打开的串口设备数量
static uint8_t g_serial_dev_num = 0;
// 串口设备信息
static serial_dev_info_t g_serial_dev_info[SERIAL_DEV_MAX_NUM];

/**
 * @brief  查找指定串口设备信息
 * @param  serial_dev_info: 输出参数, 查找到的串口设备设备信息
 * @param  serial_dev_name: 输入参数, 待查找的串口设备名
 * @return true : 成功
 * @return false: 失败
 */
static bool serial_find_dev_info(serial_dev_info_t *serial_dev_info, const char *serial_dev_name)
{
    assert((serial_dev_info != NULL) && (serial_dev_name != NULL));

    int ret = -1;

    for (uint8_t i = 0; i < g_serial_dev_num; i++)
    {
        ret = memcmp(g_serial_dev_info[i].serial_dev_name, serial_dev_name, strlen(serial_dev_name));
        if (0 == ret)
        {
            memcpy(serial_dev_info, &g_serial_dev_info[i], sizeof(serial_dev_info_t));

            return true;
        }
    }

    return false;
}

/**
 * @brief  设置串口标准波特率
 * @param  options  : 输出参数, 串口属性
 * @param  baud_rate: 输入参数, 波特率
 * @return true : 成功
 * @return false: 失败
 */
static bool serial_set_std_baud_rate(struct termios *options, const serial_baud_rate_e baud_rate)
{
    assert(options != NULL);

    // 设置输入波特率
    if (0 != cfsetispeed(options, baud_rate))
    {
        return false;
    }

    // 设置输出波特率
    if (0 != cfsetospeed(options, baud_rate))
    {
        return false;
    }

    return true;
}

/**
 * @brief  设置串口特殊波特率
 * @param  options  : 输出参数, 串口属性
 * @param  fd       : 输入参数, 文件描述符
 * @param  baud_rate: 输入参数, 波特率
 * @return true : 成功
 * @return false: 失败
 */
static bool serial_set_special_baud_rate(struct termios *options, const int fd, const int baud_rate)
{
    assert(options != NULL);

    struct serial_struct serial = {0};

    // 设置波特率为38400
    if (0 != cfsetispeed(options, B38400))
    {
        return false;
    }

    if (0 != cfsetospeed(options, B38400))
    {
        return false;
    }

    if (0 != ioctl(fd, TIOCGSERIAL, &serial))
    {
        return false;
    }

    // 设置标志位和系数
    serial.flags = ASYNC_SPD_CUST;
    serial.custom_divisor = (serial.baud_base / baud_rate);
    if (0 != ioctl(fd, TIOCSSERIAL, &serial))
    {
        return false;
    }

    return true;
}

/**
 * @brief  设置串口数据位
 * @param  options : 输出参数, 串口属性
 * @param  data_bit: 输入参数, 数据位
 */
static void serial_set_data_bit(struct termios *options, const serial_data_bit_e data_bit)
{
    assert(options != NULL);

    options->c_cflag &= ~CSIZE;
    options->c_cflag |= data_bit;
}

/**
 * @brief  设置串口奇偶检验位
 * @param  options   : 输出参数, 串口属性
 * @param  parity_bit: 输入参数, 奇偶检验位, 默认为无校验'n'或'N'
 */
static void serial_set_parity_bit(struct termios *options, const serial_parity_bit_e parity_bit)
{
    assert(options != NULL);

    switch (parity_bit)
    {
    // 无校验
    case E_SERIAL_PARITY_BIT_N:
    {
        options->c_cflag &= ~PARENB;
        options->c_iflag &= ~INPCK;

        break;
    }

    // 奇校验
    case E_SERIAL_PARITY_BIT_O:
    {
        options->c_cflag |= (PARODD | PARENB);
        options->c_iflag |= INPCK;

        break;
    }

    // 偶校验
    case E_SERIAL_PARITY_BIT_E:
    {
        options->c_cflag |= PARENB;
        options->c_cflag &= ~PARODD;
        options->c_iflag |= INPCK;

        break;
    }

    // 默认为无校验
    default:
    {
        options->c_cflag &= ~PARENB;
        options->c_iflag &= ~INPCK;

        break;
    }
    }
}

/**
 * @brief  设置串口停止位
 * @param  options : 输出参数, 串口属性
 * @param  stop_bit: 输入参数, 停止位, 默认为1位停止位
 */
static void serial_set_stop_bit(struct termios *options, const serial_stop_bit_e stop_bit)
{
    assert(options != NULL);

    switch (stop_bit)
    {
    // 1位停止位
    case E_SERIAL_STOP_BIT_1:
    {
        options->c_cflag &= ~CSTOPB;

        break;
    }

    // 2位停止位
    case E_SERIAL_STOP_BIT_2:
    {
        options->c_cflag |= CSTOPB;

        break;
    }

    // 默认为1位停止位
    default:
    {
        options->c_cflag &= ~CSTOPB;

        break;
    }
    }
}

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
                 const serial_stop_bit_e stop_bit)
{
    assert(serial_dev_name != NULL);

    int fd = -1;
    // 串口信息
    serial_dev_info_t serial_dev_info = {0};
    // 串口属性
    struct termios options = {0};

    // 超过支持的串口数量, 直接返回错误
    if (g_serial_dev_num > SERIAL_DEV_MAX_NUM)
    {
        return false;
    }

    // 串口已打开, 先关闭串口
    if (serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        // 关闭串口
        serial_close(serial_dev_info.serial_dev_name);
    }

    // 打开串口
    fd = open(serial_dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        return false;
    }

    // 获取终端属性
    if (0 != tcgetattr(fd, &options))
    {
        close(fd);

        return false;
    }

    // 设置波特率
    if (E_SERIAL_BAUD_RATE_SPECIAL != std_baud_rate)
    {
        if (!serial_set_std_baud_rate(&options, std_baud_rate))
        {
            close(fd);

            return false;
        }
    }
    else
    {
        if (!serial_set_special_baud_rate(&options, fd, special_baud_rate))
        {
            close(fd);

            return false;
        }
    }

    // 设置数据位
    serial_set_data_bit(&options, data_bit);

    // 设置奇偶检验位
    serial_set_parity_bit(&options, parity_bit);

    // 设置停止位
    serial_set_stop_bit(&options, stop_bit);

    // 一般必设置的标志
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_oflag &= ~(OPOST);
    options.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    options.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF | IXANY);

    // 清空输入输出缓冲区
    if (0 != tcflush(fd, TCIOFLUSH))
    {
        close(fd);

        return false;
    }

    // 设置最小接收字符数和超时时间
    // 当MIN=0, TIME=0时, 如果有数据可用, 则read最多返回所要求的字节数,
    // 如果无数据可用, 则read立即返回0
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    // 设置终端属性
    if (0 != tcsetattr(fd, TCSANOW, &options))
    {
        close(fd);

        return false;
    }

    // 记录串口信息
    memcpy(g_serial_dev_info[g_serial_dev_num].serial_dev_name, serial_dev_name, strlen(serial_dev_name));
    g_serial_dev_info[g_serial_dev_num].serial_dev_fd = fd;

    // 初始化互斥锁
    pthread_mutex_init(&g_serial_dev_info[g_serial_dev_num].serial_dev_mutex, NULL);

    g_serial_dev_num++;

    return true;
}

/**
 * @brief  关闭串口
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_close(const char *serial_dev_name)
{
    assert(serial_dev_name != NULL);

    int ret = -1;

    for (uint8_t i = 0; i < g_serial_dev_num; i++)
    {
        ret = memcmp(g_serial_dev_info[i].serial_dev_name, serial_dev_name, strlen(serial_dev_name));
        // 当前串口已打开
        if (0 == ret)
        {
            pthread_mutex_lock(&g_serial_dev_info[i].serial_dev_mutex);

            // 关闭串口
            ret = close(g_serial_dev_info[i].serial_dev_fd);
            if (ret < 0)
            {
                pthread_mutex_unlock(&g_serial_dev_info[i].serial_dev_mutex);

                return false;
            }

            pthread_mutex_unlock(&g_serial_dev_info[i].serial_dev_mutex);

            // 清空串口信息
            g_serial_dev_info[i].serial_dev_fd = -1;
            memset(g_serial_dev_info[i].serial_dev_name, 0, SERIAL_DEV_NAME_MAX_LEN);

            // 销毁互斥锁
            pthread_mutex_destroy(&g_serial_dev_info[i].serial_dev_mutex);

            // 将串口信息放到数组最前面
            memcpy(&g_serial_dev_info[i], &g_serial_dev_info[i + 1],
                   (sizeof(serial_dev_info_t) * (SERIAL_DEV_MAX_NUM - i - 1)));

            (g_serial_dev_num > 0) ? g_serial_dev_num-- : 0;

            return true;
        }
    }

    return true;
}

/**
 * @brief  清空串口输入缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_input_cache(const char *serial_dev_name)
{
    assert(serial_dev_name != NULL);

    // 串口信息
    serial_dev_info_t serial_dev_info = {0};

    // 串口未打开, 直接返回成功
    if (!serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        return true;
    }

    pthread_mutex_lock(&serial_dev_info.serial_dev_mutex);

    if (0 == tcflush(serial_dev_info.serial_dev_fd, TCIFLUSH))
    {
        pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

        return true;
    }

    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

    return false;
}

/**
 * @brief  清空串口输出缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_output_cache(const char *serial_dev_name)
{
    assert(serial_dev_name != NULL);

    // 串口信息
    serial_dev_info_t serial_dev_info = {0};

    // 串口未打开, 直接返回成功
    if (!serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        return true;
    }

    pthread_mutex_lock(&serial_dev_info.serial_dev_mutex);

    if (0 == tcflush(serial_dev_info.serial_dev_fd, TCOFLUSH))
    {
        pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

        return true;
    }

    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

    return false;
}

/**
 * @brief  清空串口输入和输出缓存
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @return true : 成功
 * @return false: 失败
 */
bool serial_flush_both_cache(const char *serial_dev_name)
{
    assert(serial_dev_name != NULL);

    // 串口信息
    serial_dev_info_t serial_dev_info = {0};

    // 串口未打开, 直接返回成功
    if (!serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        return true;
    }

    pthread_mutex_lock(&serial_dev_info.serial_dev_mutex);

    if (0 == tcflush(serial_dev_info.serial_dev_fd, TCIOFLUSH))
    {
        pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

        return true;
    }

    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

    return false;
}

/**
 * @brief  串口发送数据
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @param  send_data      : 输入参数, 待发送数据
 * @param  send_data_len  : 输入参数, 待发送数据长度
 * @return 成功: 实际发送数据长度
 * @return 失败: -1
 */
int serial_write_data(const char *serial_dev_name, const uint8_t *send_data, const uint32_t send_data_len)
{
    assert((serial_dev_name != NULL) && (send_data != NULL) && (send_data_len > 0));

    int ret = -1;
    // 串口信息
    serial_dev_info_t serial_dev_info = {0};

    // 串口未打开, 直接返回失败
    if (!serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        return -1;
    }

    pthread_mutex_lock(&serial_dev_info.serial_dev_mutex);

    ret = write(serial_dev_info.serial_dev_fd, send_data, send_data_len);

    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

    return ret;
}

/**
 * @brief  串口接收数据
 * @param  recv_data      : 输出参数, 接收数据
 * @param  serial_dev_name: 输入参数, 串口设备名(例: /dev/ttyS1)
 * @param  recv_data_len  : 输入参数, 接收数据长度
 * @param  timeout:       : 输入参数, 接收超时(单位：ms)
 * @return 成功: 实际接收数据长度
 *         失败: -1
 */
int serial_read_data(uint8_t *recv_data, const char *serial_dev_name, const size_t recv_data_len, uint32_t timeout)
{
    assert((recv_data != NULL) && (serial_dev_name != NULL) && (recv_data_len > 0));

    int ret = -1;
    // 指定fds数组中的项目数
    nfds_t nfds = 1;
    // 指定要监视的文件描述符集
    struct pollfd fds[1] = {0};
    // 已读取数据长度
    size_t total_data_len = 0;
    // 未读取数据长度
    size_t remain_data_len = 0;
    // 串口信息
    serial_dev_info_t serial_dev_info = {0};

    // 串口未打开, 直接返回失败
    if (!serial_find_dev_info(&serial_dev_info, serial_dev_name))
    {
        return -1;
    }

    memset(recv_data, 0, recv_data_len);

    remain_data_len = recv_data_len;

    // 加锁
    pthread_mutex_lock(&serial_dev_info.serial_dev_mutex);

    while (1)
    {
        // 设置需要监听的文件描述符
        memset(fds, 0, sizeof(fds));
        fds[0].fd = serial_dev_info.serial_dev_fd;
        fds[0].events = POLLIN;

        ret = poll(fds, nfds, timeout);
        // 返回负值, 发生错误
        if (ret < 0)
        {
            // 解锁
            pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

            return -1;
        }
        // 返回0, 超时
        else if (0 == ret)
        {
            // 如果超时后, 已读取数据长度大于0, 返回实际接收数据长度
            if (total_data_len > 0)
            {
                // 解锁
                pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

                return total_data_len;
            }

            // 解锁
            pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

            return -1;
        }
        // 返回值大于0, 成功
        else
        {
            // 判断是否是期望的返回
            if (fds[0].revents & POLLIN)
            {
                // 从文件起始位置开始读数据
                lseek(fds[0].fd, 0, SEEK_SET);
                ret = read(fds[0].fd, &recv_data[total_data_len],
                           remain_data_len);
                if (ret < 0)
                {
                    // 解锁
                    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

                    return -1;
                }

                // 计算已读取数据长度
                total_data_len += ret;
                // 计算剩余需要读取长度
                remain_data_len = (recv_data_len - total_data_len);
                // 读取完毕
                if (total_data_len == recv_data_len)
                {
                    break;
                }
            }
        }
    }

    // 解锁
    pthread_mutex_unlock(&serial_dev_info.serial_dev_mutex);

    return total_data_len;
}
