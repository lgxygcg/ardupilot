/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/QHFCetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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

/* 
   QHFC Telemetry library
*/

#include "AP_QHFC.h"
#define AP_SERIALMANAGER_OPEN_MV_BAUD               57600
#define AP_SERIALMANAGER_OPENMV_BUFSIZS_RX          128
#define AP_SERIALMANAGER_OPENMV_BUFSIZS_TX          128
//#define AP_SHEBEIDUZ                                0x01
#include <AP_SerialManager/AP_SerialManager.h>
//
#include <AP_Math/crc.h>
//
//
//#include "AP_ADSB_uAvionix_MAVLink.h"
//#include "AP_ADSB_Sagetech.h"
//#include <AP_Vehicle/AP_Vehicle.h>   //后加程序
#include <GCS_MAVLink/GCS.h>     //后加程序
#include <AP_Logger/AP_Logger.h>   //后加程序
//
#include <AP_BattMonitor/AP_BattMonitor.h>


// 

extern const AP_HAL::HAL& hal;
AP_QHFC *AP_QHFC::_singleton;   ////后加程序

AP_QHFC::AP_QHFC(void)
{   
    
    _singleton = this;  //后加程序
    _port = NULL;
    _step = 0;

    //<-- ------------------------------------------------------------------- ->//
    OnOff_Cmd = 0;
    is_armed_old = false;
    debug_cnt = 0;
    //<-- ------------------------------------------------------------------- ->//
}

/*
 * init - perform required initialisation
 */
void AP_QHFC::init(const AP_SerialManager& serial_manager)
{
  protocol = AP_SerialManager::SerialProtocol_None;

  if((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_QHFC_V1, 0)) != nullptr)
  {
    protocol = AP_SerialManager::SerialProtocol_QHFC_V1;
  }
  else if((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_QHFC_V2, 0)) != nullptr)
  {
    protocol = AP_SerialManager::SerialProtocol_QHFC_V2;
  }

  if(protocol != AP_SerialManager::SerialProtocol_None)
  {
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZS_RX,AP_SERIALMANAGER_OPENMV_BUFSIZS_TX);

    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_QHFC::tick, void));//定时函数用
    update();
  } 
}

void AP_QHFC::packedReceived(uint8_t *buf,uint16_t len)
{
  switch(buf[1])
  {
    case 0x03:
      if(len == 103)
      {
        FCStatus.SystemTick = ((uint32_t)buf[9] << 24) | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 8) | (uint32_t)buf[12];
        FCStatus.FC1Volt =      ((uint16_t)buf[13] << 8) | (uint16_t)buf[14];
        FCStatus.FC1Current =   ((uint16_t)buf[15] << 8) | (uint16_t)buf[16];
        FCStatus.FC1Temp1 =     ((uint16_t)buf[17] << 8) | (uint16_t)buf[18];
        FCStatus.FC1Temp2 =     ((uint16_t)buf[19] << 8) | (uint16_t)buf[20];
        FCStatus.FC1Error =     ((uint16_t)buf[21] << 8) | (uint16_t)buf[22];
        FCStatus.FC1FanDuty =   ((uint16_t)buf[23] << 8) | (uint16_t)buf[24];
        FCStatus.FC1FanSpeed1 = ((uint16_t)buf[25] << 8) | (uint16_t)buf[26];
        FCStatus.FC1FanSpeed2 = ((uint16_t)buf[27] << 8) | (uint16_t)buf[28];

        FCStatus.FC2Volt =      ((uint16_t)buf[29] << 8) | (uint16_t)buf[30];
        FCStatus.FC2Current =   ((uint16_t)buf[31] << 8) | (uint16_t)buf[32];
        FCStatus.FC2Temp1 =     ((uint16_t)buf[33] << 8) | (uint16_t)buf[34];
        FCStatus.FC2Temp2 =     ((uint16_t)buf[35] << 8) | (uint16_t)buf[36];
        FCStatus.FC2Error =     ((uint16_t)buf[37] << 8) | (uint16_t)buf[38];
        FCStatus.FC2FanDuty =   ((uint16_t)buf[39] << 8) | (uint16_t)buf[40];
        FCStatus.FC2FanSpeed1 = ((uint16_t)buf[41] << 8) | (uint16_t)buf[42];
        FCStatus.FC2FanSpeed2 = ((uint16_t)buf[43] << 8) | (uint16_t)buf[44];

        FCStatus.FC3Volt =      ((uint16_t)buf[45] << 8) | (uint16_t)buf[46];
        FCStatus.FC3Current =   ((uint16_t)buf[47] << 8) | (uint16_t)buf[48];
        FCStatus.FC3Temp1 =     ((uint16_t)buf[49] << 8) | (uint16_t)buf[50];
        FCStatus.FC3Temp2 =     ((uint16_t)buf[51] << 8) | (uint16_t)buf[52];
        FCStatus.FC3Error =     ((uint16_t)buf[53] << 8) | (uint16_t)buf[54];
        FCStatus.FC3FanDuty =   ((uint16_t)buf[55] << 8) | (uint16_t)buf[56];
        FCStatus.FC3FanSpeed1 = ((uint16_t)buf[57] << 8) | (uint16_t)buf[58];
        FCStatus.FC3FanSpeed2 = ((uint16_t)buf[59] << 8) | (uint16_t)buf[60];

        FCStatus.FC4Volt =      ((uint16_t)buf[61] << 8) | (uint16_t)buf[62];
        FCStatus.FC4Current =   ((uint16_t)buf[63] << 8) | (uint16_t)buf[64];
        FCStatus.FC4Temp1 =     ((uint16_t)buf[65] << 8) | (uint16_t)buf[66];
        FCStatus.FC4Temp2 =     ((uint16_t)buf[67] << 8) | (uint16_t)buf[68];
        FCStatus.FC4Error =     ((uint16_t)buf[69] << 8) | (uint16_t)buf[70];
        FCStatus.FC4FanDuty =   ((uint16_t)buf[71] << 8) | (uint16_t)buf[72];
        FCStatus.FC4FanSpeed1 = ((uint16_t)buf[73] << 8) | (uint16_t)buf[74];
        FCStatus.FC4FanSpeed2 = ((uint16_t)buf[75] << 8) | (uint16_t)buf[76];

        FCStatus.LiVolt =           ((uint16_t)buf[77] << 8) | (uint16_t)buf[78];
        FCStatus.LiCurrent =        ((uint16_t)buf[79] << 8) | (uint16_t)buf[80];
        FCStatus.LiError =          ((uint16_t)buf[81] << 8) | (uint16_t)buf[82];

        FCStatus.H2PressureH =      ((uint16_t)buf[83] << 8) | (uint16_t)buf[84];
        FCStatus.H2PressureL =      ((uint16_t)buf[85] << 8) | (uint16_t)buf[86];
        FCStatus.AmbTemperature =   ((uint16_t)buf[87] << 8) | (uint16_t)buf[88];
        FCStatus.AmbHumidity =      ((uint16_t)buf[89] << 8) | (uint16_t)buf[90];
        FCStatus.AmbControlStatus = ((uint16_t)buf[91] << 8) | (uint16_t)buf[92];
      }
      else if(len == 47)
      {
        #if(1)
        FCStatus.FC1Volt = (buf[11]<<8) | buf[12];
        FCStatus.FC1Current = (buf[13]<<8) | buf[14]; 
        FCStatus.FC1Temp1 = (buf[15]<<8) | buf[16];
        FCStatus.LiVolt = (buf[19]<<8) | buf[20];
        FCStatus.LiCurrent = (buf[21]<<8) | buf[22];
        FCStatus.H2PressureH = (buf[31]<<8) | buf[32];
        FCStatus.Warning = (buf[41]<<8) | buf[42];
        FCStatus.Fault = (buf[43]<<8) | buf[44];
        #else
        FCStatusOld._FCV = (buf[11]<<8) | buf[12];   //两数值合并
        FCStatusOld._FCA = (buf[13]<<8) | buf[14]; 
        FCStatusOld._FCWENDU = (buf[15]<<8) | buf[16];
        FCStatusOld._FCW = (buf[17]<<8) | buf[18]; 
        FCStatusOld._FCDCV = (buf[19]<<8) | buf[20];
        FCStatusOld._FCDCA = (buf[21]<<8) | buf[22];
        FCStatusOld._FCKW = (buf[27]<<8) | buf[28];
        FCStatusOld._FCMPA = (buf[31]<<8) | buf[32];
        #endif
      }
      
      last_frame_ms = AP_HAL::millis();
      //gcs().send_message(MSG_QH_FCSTATUS);
      //Log_Write_qhfc();
      break;
    case 0x06:
      qh_onoff = buf[5];

      last_frame_ms = AP_HAL::millis();
      //gcs().send_message(MSG_QH_FCSTATUS);
      //Log_Write_qhfc();
      break;
    default:
      break;
  }
}

bool AP_QHFC::update()
{
  if (_port == NULL) 
    return false;   

  int16_t numc = _port->available();
      
  data = 0;

  if(numc <= 0)_step = 0;
  if(recv_cnt >= QHFC_RECVBUF_SIZE)_step = 0;

  for(int16_t i = 0; i < numc; i++)
  {
    data =_port->read();

    switch(_step)
    {
      case 0:
        recv_cnt = 0;
        recv_buf[recv_cnt] = data;
        if(recv_buf[0] == (0x01))
        {
          recv_cnt ++;
          _step = 1;
        }
        break;
      case 1:     //command
        recv_buf[recv_cnt] = data;
        recv_cnt++;
        _step = 2;
        break;
      case 2:     //len
        recv_buf[recv_cnt] = data;          
        recv_cnt ++;
        _step = 3;
        break;
      case 3:
        recv_buf[recv_cnt] = data;
        recv_cnt ++;
        if(recv_cnt == recv_buf[2] + 5)
        {
          QHFC_crc = calc_crc_modbus(recv_buf, recv_cnt - 2);
          crch = QHFC_crc >> 8;
          chal = QHFC_crc & 0xFF;

          if((recv_buf[recv_cnt - 2] == chal) && (recv_buf[recv_cnt - 1] == crch))
          {
            packedReceived(recv_buf,recv_cnt);
            return true;
          }
          _step = 0;
        }
        break;
    }
  }
  return false;
}

//////
int16_t AP_QHFC::CalFCTemperature(uint8_t Ch)
{
  int16_t temp1,temp2;

  switch(Ch)
  {
    case 0:
      temp1 = FCStatus.FC1Temp1;
      temp2 = FCStatus.FC1Temp2;
      break;
    case 1:
      temp1 = FCStatus.FC2Temp1;
      temp2 = FCStatus.FC2Temp2;
      break;
    case 2:
      temp1 = FCStatus.FC3Temp1;
      temp2 = FCStatus.FC3Temp2;
      break;
    case 3:
      temp1 = FCStatus.FC4Temp1;
      temp2 = FCStatus.FC4Temp2;
      break;
    default:
      return QHFC_TEMP_MAX;
  }

  if((temp1 > QHFC_TEMP_MAX) || (temp1 < QHFC_TEMP_MIN))return QHFC_TEMP_MAX;
  if((temp2 > QHFC_TEMP_MAX) || (temp2 < QHFC_TEMP_MIN))return QHFC_TEMP_MAX;

  if(temp1 >= temp2)
    return temp1;
  else
    return temp2;
}
void AP_QHFC::CalFCVoltCur(uint16_t &Volt,uint16_t &Cur)
{
  uint16_t v = FCStatus.FC1Volt;
  uint16_t i;
  uint32_t p;

  if(FCStatus.FC2Volt > v)v = FCStatus.FC2Volt;
  if(FCStatus.FC3Volt > v)v = FCStatus.FC3Volt;
  if(FCStatus.FC4Volt > v)v = FCStatus.FC4Volt;

  p = (uint32_t)FCStatus.FC1Volt * FCStatus.FC1Current;
  p += (uint32_t)FCStatus.FC2Volt * FCStatus.FC2Current;
  p += (uint32_t)FCStatus.FC3Volt * FCStatus.FC3Current;
  p += (uint32_t)FCStatus.FC4Volt * FCStatus.FC4Current;

  i = p / v;

  Volt = v;
  Cur = i;
}
//
//
// ******************发送氢航握手包
void AP_QHFC::send_woshoubao(){
      uint8_t woshou[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x13, 0x04, 0x07};

      _port->write(woshou,8);

}


//关闭氢航燃料电池
void AP_QHFC::send_guan()  //发送需要别的地方触发
{
     uint8_t guan[]={0x01, 0x06, 0x00, 0x13, 0x00, 0x00, 0x78, 0x0F};
    if (_port == nullptr) {
        return;
      }
          _port->write(guan,8); 
      
    }

//开启氢航燃料电池
void AP_QHFC::send_kai()  //发送需要别的地方触发
{
     uint8_t kai[]={0x01, 0x06, 0x00, 0x13, 0x00, 0x01, 0xB9, 0xCF};
    if (_port == nullptr) {
        return;
    }
        _port->write(kai,8); //发送数据
        
    }


void AP_QHFC::tick(void)// 定时器函数
{
  uint32_t now = AP_HAL::millis();

  if (now - last_send_frame_ms > 100)
  {
    last_send_frame_ms = now;
    if((OnOff_Cmd == QHFC_CMD_PARAM_ON ) || (OnOff_Cmd == QHFC_CMD_PARAM_OFF ))
    {
      if (OnOff_Cmd == QHFC_CMD_PARAM_ON )
      {
        send_kai();
        OnOff_Cmd = FCkai;
        if (qh_onoff ==0x01)
        {
          OnOff_Cmd = 1;
          /// qh_kaiguan = QHFC_CMD_PARAM_ON;
          //gcs().send_message(MSG_QH_FCSTATUS);
        }
      }
      if (OnOff_Cmd == QHFC_CMD_PARAM_OFF )
      {
        send_guan();
        OnOff_Cmd = FCguan;
        if (qh_onoff == 0x00)
        {
          OnOff_Cmd = 0;
          //qh_kaiguan = QHFC_CMD_PARAM_OFF;
          //gcs().send_message(MSG_QH_FCSTATUS);
        }
      }
    }
    else
    {
      send_woshoubao();
    }
  }
  return ;
     //}
}   //uint8_t checksum =0;

#define DEBUG_FC_FAULTSAFE      (0)

FCFailsafeAction AP_QHFC::handle_FC_failsafe(bool is_armed)
{
  //FCFailsafeAction Action = FCFailsafeAction::NONE;

  if(is_armed == true)
  {
    if(is_armed_old == false)
    {
      CurAction = FCFailsafeAction::NONE;
      #if DEBUG_FC_FAULTSAFE
      debug_cnt = 0;
      #endif
    }
    else
    {
      #if DEBUG_FC_FAULTSAFE
      debug_cnt++;
      if(debug_cnt == 1000)
      {
        FCStatus.Warning = 1;
        gcs().send_text(MAV_SEVERITY_INFO,"debug:set Warning!");
      }
      if(debug_cnt == 1200)
      {
        FCStatus.Fault = 1;
        gcs().send_text(MAV_SEVERITY_INFO,"debug:set Fault!");
      }
      #endif
    }
  }
  else
  {
    CurAction = FCFailsafeAction::NONE;
  }
  is_armed_old = is_armed;

  //
  if(is_armed == true)
  {
    if(FCStatus.Fault != 0)
    {
      if((CurAction == FCFailsafeAction::NONE) || (CurAction == FCFailsafeAction::SMARTRTL))
      {
        CurAction = FCFailsafeAction::LAND;
        gcs().send_text(MAV_SEVERITY_ERROR,"Fault:set mode LAND!");
      }
    }
    else if(FCStatus.Warning != 0)
    {
      if(CurAction == FCFailsafeAction::NONE)
      {
        CurAction = FCFailsafeAction::SMARTRTL;
        gcs().send_text(MAV_SEVERITY_ERROR,"Fault:set mode SMARTRTL!");
      }
    }
  }

  return CurAction;
}
///后加程序 
////后加的
namespace AP 
{
  AP_QHFC &qhfc()
  {
    return *AP_QHFC::get_singleton();
  }
};
  /*/AP_QHFC *AP::QHFC()
//{
    return AP_QHFC::get_singleton();
}
//#endif // HAL_ADSB_ENABLED*/
