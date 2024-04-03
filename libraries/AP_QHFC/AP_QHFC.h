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

#define QHFC_CMD_PARAM_ON       (0x55)
#define QHFC_CMD_PARAM_OFF      (0xAA)
//
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
    float qh_onoff;
  //  float qh_off;
    float qh_on;

    //<-- ------------------------------------------------------------------- ->//
    uint16_t OnOff_Cmd;
    //<-- ------------------------------------------------------------------- ->//

    uint32_t last_frame_ms;
    uint32_t _last_frame_ms;
    uint32_t _FCV;
    uint32_t _FCA;
    uint32_t _FCMPA;
    uint32_t _FCWENDU;
    uint32_t _FCKW;
    uint32_t _FCDCA;
    uint32_t _FCDCV;
    uint32_t _FCW;
    uint32_t FCkai;
    uint32_t FCguan;
    uint8_t _D0;
    uint8_t crch;
    uint8_t chal;
    uint16_t QHFC_crc;
    
////*************临时数据要删除****************


///////////////////////////////////////////////////////


private:
    static AP_QHFC *_singleton;   ////后加程序
     
    AP_HAL::UARTDriver *_port;    //读取 ->read
    void send();
    void tick(void);
    //与发送数据的螺母和螺栓相关的方法
    void send_byte(uint8_t value);
    void send_uint16();

    void send_woshoubao();
    //
         uint8_t recv_duf[43];
         uint8_t recv_vnt;
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
