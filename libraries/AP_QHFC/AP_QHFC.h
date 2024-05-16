/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS/AP_AHRS.h"
#include "AP_SerialManager/AP_SerialManager.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS.h>
//
//
#define QHFC_COMMAND_ONOFF       (0x80)

#define QHFC_CMD_PARAM_NONE     (0x00)
#define QHFC_CMD_PARAM_ON       (0x55)
#define QHFC_CMD_PARAM_OFF      (0xAA)

#define CMD_TIMEOUT_MAX         (300)
#define CMD_RETRY_CNT           (3)
//
#define QHFC_TEMP_MIN           (-600)      //-600 -> -60degree
#define QHFC_TEMP_MAX           (3000)      //3000 -> 300degree

#define QHFC_RECVBUF_SIZE       (64)

#define QHFC_V1_WARNING_VOLTAGELOW      (0x0001)
#define QHFC_V1_WARNING_TEMPHIGH        (0x0002)
#define QHFC_V1_WARNING_PRESSLOW        (0x0004)
#define QHFC_V1_WARNING_FANSPEED        (0x0008)
#define QHFC_V1_WARNING_PERFORMLOW      (0x0010)

#define QHFC_V1_FAULT_VOLTAGELOW        (0x0001)
#define QHFC_V1_FAULT_TEMPHIGH          (0x0002)

#define QHFC_V2_OFF                     (0x00000000)
#define QHFC_V2_ON                      (0x00000001)

#define QHFC_V2_WARNING_PRESSLOW        (0x00000004)
#define QHFC_V2_WARNING_TEMPHIGH        (0x00000010)
#define QHFC_V2_WARNING_FCVOLTAGELOW    (0x00000040)
#define QHFC_V2_WARNING_LIVOLTAGELOW    (0x00000100)
#define QHFC_V2_WARNING_FANSPEED        (0x00000400)
#define QHFC_V2_WARNING_H2LEAKAGE       (0x00001000)
#define QHFC_V2_WARNING_PERFORMLOW      (0x00004000)

#define QHFC_V2_FAULT_PRESSLOW          (0x00000008)
#define QHFC_V2_FAULT_TEMPHIGH          (0x00000020)
#define QHFC_V2_FAULT_FCVOLTAGELOW      (0x00000080)
#define QHFC_V2_FAULT_LIVOLTAGELOW      (0x00000200)
#define QHFC_V2_FAULT_FANSPEED          (0x00000800)
#define QHFC_V2_FAULT_H2LEAKAGE         (0x00002000)
#define QHFC_V2_FAULT_PERFORMLOW        (0x00008000)

enum class FCFailsafeAction : uint8_t {
        NONE               = 0,
        LAND               = 1,
        RTL                = 2,
        SMARTRTL           = 3
    };

typedef struct PACKED _bagQH_FCStatus{
    uint32_t FCStatus;
    int16_t FCTemperature[4];
    uint16_t FCVoltage;
    uint16_t FCCurrent;
    uint16_t LIVoltage;
    int16_t LICurrent;
    uint16_t Press;
    
    int16_t AmbTemperature;
    uint8_t AmbHumidity;
    uint8_t AmbControlStatus;
}QH_GCStatus;
typedef struct PACKED _bagQH_HPSStatusV2{
    uint32_t SystemTick;

    uint16_t FCStatus;

    uint16_t FC1Volt;
    uint16_t FC1Current;
    int16_t FC1Temp1;
    int16_t FC1Temp2;
    uint16_t FC1Error;
    uint16_t FC1FanDuty;
    uint16_t FC1FanSpeed1;
    uint16_t FC1FanSpeed2;

    uint16_t FC2Volt;
    uint16_t FC2Current;
    int16_t FC2Temp1;
    int16_t FC2Temp2;
    uint16_t FC2Error;
    uint16_t FC2FanDuty;
    uint16_t FC2FanSpeed1;
    uint16_t FC2FanSpeed2;

    uint16_t FC3Volt;
    uint16_t FC3Current;
    int16_t FC3Temp1;
    int16_t FC3Temp2;
    uint16_t FC3Error;
    uint16_t FC3FanDuty;
    uint16_t FC3FanSpeed1;
    uint16_t FC3FanSpeed2;

    uint16_t FC4Volt;
    uint16_t FC4Current;
    int16_t FC4Temp1;
    int16_t FC4Temp2;
    uint16_t FC4Error;
    uint16_t FC4FanDuty;
    uint16_t FC4FanSpeed1;
    uint16_t FC4FanSpeed2;

    uint16_t LiVolt;
    int16_t LiCurrent;
    uint16_t LiError;

    uint16_t H2PressureH;
    uint16_t H2PressureL;

    int16_t AmbTemperature;
    int8_t AmbHumidity;
    int8_t AmbControlStatus;
}QH_HPSStatusV2;

typedef struct PACKED _bagQH_HPSStatusV1{
    uint32_t _FCV;
    uint32_t _FCA;
    uint32_t _FCWENDU;
    uint32_t _FCW;
    uint32_t _FCDCV;
    uint32_t _FCDCA;
    uint32_t _FCKW;
    uint32_t _FCMPA;

    uint16_t Warning;
    uint16_t Fault;
}QH_HPSStatusV1;
//
class AP_QHFC_Parameters;
 
class AP_QHFC {

public:
 
   friend class AP_QHFC_uAvionix_MAVLink;
    AP_QHFC();


/*不允许复制*/
    AP_QHFC(const AP_QHFC &other) = delete;
    AP_QHFC &operator=(const AP_QHFC&) = delete;
    static AP_QHFC *get_singleton(void) {        //后加程序
        return _singleton;                       //后加程序
    }

//初始化-执行所需的初始化
    void init(const AP_SerialManager& serial_manager);
    bool update(void);
    void send_guan();
    void send_kai();
    bool qhsc(void);
    
   // float qh_kaiguan;
    //uint8_t qh_onoff;

  //  float qh_off;
    //float qh_on;
    uint32_t debug_cnt;

    int16_t CalFCTemperature(uint8_t Ch);
    void CalFCVoltCur(uint16_t &Volt,uint16_t &Cur);

    bool is_armed_old;
    FCFailsafeAction CurAction;
    FCFailsafeAction handle_FC_failsafe(bool is_armed);

    uint32_t last_frame_ms;
    uint32_t last_send_frame_ms;
//
    uint32_t FCkai;
    uint32_t FCguan;
    uint8_t _D0;
    uint8_t crch;
    uint8_t chal;
    uint16_t QHFC_crc;
    
    QH_GCStatus GCStatus;
    QH_HPSStatusV1 HPSStatusV1;
    QH_HPSStatusV2 HPSStatusV2;

    void Set_Cmd(uint16_t Param);
////*************临时数据要删除****************


///////////////////////////////////////////////////////


private:
    static AP_QHFC *_singleton;   ////后加程序
     
    AP_HAL::UARTDriver *_port;    //读取 ->read
    AP_SerialManager::SerialProtocol protocol;
    void send();
    void tick(void);
    //与发送数据的螺母和螺栓相关的方法
    void send_byte(uint8_t value);
    void send_uint16();

    void send_woshoubao();

    //<-- ------------------------------------------------------------------- ->//
    uint16_t OnOff_Cmd;
    uint16_t OnOff_Cmd_buf;
    uint16_t OnOff_Status;
    uint16_t OnOff_HPSAck;
    uint32_t Cmd_Timeout_Cnt;
    uint16_t Cmd_Retry_Cnt;

    uint32_t last_tick_ms;

    void packedReceived(uint8_t *buf,uint16_t len);
    void _Send_Cmd(void);
    void _Clear_Cmd(void);
    void HPSStatusV2_To_GC(void);
    void HPSStatusV1_To_GC(void);
    void Update_GC_OnOff(void);
    bool IsFCFault(void);
    bool IsFCWarning(void);
    //<-- ------------------------------------------------------------------- ->//
    //
         uint8_t recv_buf[QHFC_RECVBUF_SIZE];
         uint8_t recv_cnt;
         uint8_t data;
         uint8_t _step;
 struct {
        uint32_t last_200ms_frame;
        uint32_t last_1000ms_frame;
    } _D;

};

////
//
////后加程序
//
namespace AP {
    AP_QHFC &qhfc();
};
