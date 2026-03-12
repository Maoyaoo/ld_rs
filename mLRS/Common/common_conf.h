//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// common config
//*******************************************************
#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H
#pragma once

#define VERSION 10307  // leading zero makes it octal!
#define VERSIONONLYSTR "v1.3.07"
#define SETUPLAYOUT 10304  // this should be changed then Setup struct and/or serial changes

//-------------------------------------------------------
// 可选系统配置
//-------------------------------------------------------
// 用户可以根据需要修改这些配置

// 取消注释以启用接收端开机后自动进入配对模式
// #define RX_BIND_MODE_AFTER_POWERUP

// 开发功能。注意：这些功能仅用于测试，不适用于生产环境
// 目前暂无

//-------------------------------------------------------
// 设置 (Setup)
//-------------------------------------------------------
// 这些宏定义决定了设置参数的默认值
// 用户可以根据需要修改

// 取消注释以强制使用下面的默认配置，否则从EEPROM读取配置
// #define SETUP_FORCE_COMMON_CONF

#define CPOWER RFPOWER_DEFAULT
// #define CPOWER                          0 // 0: 最小功率, 255: 最大功率

#define DIVERSITY 0

#define SETUP_TX_SERIAL_DESTINATION 0  // 0: 串口, 1: 串口2 (BT/ESP), 2: mBridge

#define SETUP_TX_CHANNELS_SOURCE 1  // 0: 无, 1: Crsf (pin5), 2: In (In 或 pin1), 3: mBridge (pin5)

#define SETUP_TX_CHANNEL_ORDER CHANNEL_ORDER_AETR

#define SETUP_TX_IN_MODE 0  // 0: IN_CONFIG_SBUS, 1: IN_CONFIG_SBUS_INVERTED

#define SETUP_TX_SERIAL_BAUDRATE 4  // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200, 5: 230400

#define SETUP_TX_POWER CPOWER

#define SETUP_TX_DIVERSITY DIVERSITY  // 0: 默认, 1: 天线1 (如果支持分集), 2: 天线2 (如果支持分集)

#define SETUP_TX_SEND_RADIO_STATUS 1  // 0: 关闭, 1: 1 Hz
#define SETUP_TX_MAV_COMPONENT 1      // 0: 关闭, 1: 启用

#define SETUP_TX_BUZZER 0  // 0: 关闭, 1: 丢包报警, 2: 接收质量

#define SETUP_RX_CHANNEL_ORDER CHANNEL_ORDER_AETR

#define SETUP_RX_OUT_MODE 1  // 0: OUT_CONFIG_SBUS, 1: OUT_CONFIG_CRSF, 2: OUT_CONFIG_SBUS_INVERTED

#define SETUP_RX_FAILSAFE_MODE 0  // 0: 无信号 1: 低油门, 4: CH1-CH4 中位信号

#define SETUP_RX_SERIAL_PORT 0  // 0: 串口, 1: CAN

#define SETUP_RX_SERIAL_BAUDRATE 4  // 0: 9600, 1: 19200, 2: 38400, 3: 57600, 4: 115200, 5: 230400

#define SETUP_RX_POWER CPOWER

#define SETUP_RX_DIVERSITY DIVERSITY  // 0: 默认, 1: 天线1 (如果支持分集), 2: 天线2 (如果支持分集)

#define SETUP_RX_SERIAL_LINK_MODE 0  // 0: 透明传输, 1: mavlink, 2: mavlinkX, 3: mspX

#define SETUP_RX_SEND_RADIO_STATUS 1  // 0: 关闭, 1: ardu_1, 2: px4 (也叫 "brad")
#define SETUP_RX_SEND_RC_CHANNELS 0   // 0: 关闭, 1: RC_CHANNEL_OVERRIDE, 2: RC_CHANNELS

#define SETUP_RX_OUT_RSSI_CHANNEL 0  // 0: 关闭, 5: CH5, 16: CH16
#define SETUP_RX_OUT_LQ_CHANNEL 0    // 0: 关闭, 5: CH5, 16: CH16

#define BIND_PHRASE "ldrs.0"  // 6字符字符串，允许使用 'a'-'z','0'-'9','_','-','#','.'


//#define SETUP_MODE                      MODE_50HZ
//#define SETUP_MODE                      MODE_31HZ
#define SETUP_MODE                      MODE_19HZ


#define SETUP_RF_BAND                    SETUP_FREQUENCY_BAND_915_MHZ_FCC
//#define SETUP_RF_BAND                    SETUP_FREQUENCY_BAND_868_MHZ // that's my privilege :)

#define SETUP_RF_ORTHO 0  // 0: 关闭, 1: 1/3, 2: 2/3, 3: 3/3

//-------------------------------------------------------
// 系统配置
//-------------------------------------------------------
// 设置各种内部值
// 用户禁止修改

#define MODE_50HZ_SEND_FRAME_TMO_MS 10       // 只需要大于toa，不严格要求
#define MODE_31HZ_SEND_FRAME_TMO_MS 15       // 只需要大于toa，不严格要求
#define MODE_19HZ_SEND_FRAME_TMO_MS 25       // 只需要大于toa，不严格要求
#define MODE_FLRC_111HZ_SEND_FRAME_TMO_MS 7  // 只需要大于toa，不严格要求
#define MODE_FSK_50HZ_SEND_FRAME_TMO_MS 10   // 只需要大于toa，不严格要求

#define FHSS_NUM_BAND_433_MHZ 2               // 2，因为需要1个用于配对
#define FHSS_NUM_BAND_70_CM_HAM_19HZ_MODE 12  // 匹配2.4GHz在19Hz模式
#define FHSS_NUM_BAND_70_CM_HAM 18            // 匹配2.4GHz在31Hz模式
#define FHSS_NUM_BAND_868_MHZ 6               // 频带很窄
#define FHSS_NUM_BAND_915_MHZ_FCC \
    25  // https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/subject-group-ECFR2f2e5828339709e/section-15.247#p-15.247(a)(1)(i)
#define FHSS_NUM_BAND_866_MHZ_IN 3          // 3，因为需要1个用于配对
#define FHSS_NUM_BAND_2P4_GHZ_19HZ_MODE 12  // 原为24，但一个周期需要1.3秒！需要更长的断开时间
#define FHSS_NUM_BAND_2P4_GHZ_31HZ_MODE 18
#define FHSS_NUM_BAND_2P4_GHZ 24

#define FRAME_TX_RX_LEN 91  // 当前只支持等长

#define CONNECT_TMO_MS 1250  // 断开连接时间，原为500，后改为750以更好地处理19Hz模式，现为1250

#define CONNECT_SYNC_CNT 5  // 连接所需的包数量

#define LQ_AVERAGING_MS 1000

#define TX_SERIAL_BAUDRATE 115200  // 会被setup覆盖
#define TX_SERIAL_TXBUFSIZE 512
#define TX_SERIAL_RXBUFSIZE 2048  // MissionPlanner真的很霸道

#define TX_MBRIDGE_TXBUFSIZE 512
#define TX_MBRIDGE_RXBUFSIZE 2048  // MissionPlanner真的很霸道

#define RX_SERIAL_BAUDRATE 115200  // 会被setup覆盖
#define RX_SERIAL_TXBUFSIZE 1024
#define RX_SERIAL_RXBUFSIZE 2048  // ArduPilot也很霸道

#define TX_COM_BAUDRATE 115200
#define TX_COM_TXBUFSIZE 512         // 2048 // cli需要超过1024，自4.2.2025起我们有了cli chunks
#define TX_COM_TXBUFSIZE_SMALL 256   // 512 // 我们没有足够的RAM
#define TX_COM_TXBUFSIZE_LARGE 2048  // 我们有足够的空间，可以轻松承担
#define TX_COM_RXBUFSIZE 512

#define RX_BIND_MODE_AFTER_POWERUP_TIME_SEC 60

#endif  // COMMON_CONFIG_H
