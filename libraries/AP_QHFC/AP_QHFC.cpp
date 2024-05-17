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
    OnOff_Cmd = QHFC_CMD_PARAM_NONE;
    OnOff_Cmd_buf = OnOff_Cmd;
    OnOff_Status = QHFC_CMD_PARAM_NONE;
    OnOff_HPSAck = QHFC_CMD_PARAM_NONE;
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
        HPSStatusV2.SystemTick = ((uint32_t)buf[9] << 24) | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 8) | (uint32_t)buf[12];
        HPSStatusV2.FC1Volt =      ((uint16_t)buf[13] << 8) | (uint16_t)buf[14];
        HPSStatusV2.FC1Current =   ((uint16_t)buf[15] << 8) | (uint16_t)buf[16];
        HPSStatusV2.FC1Temp1 =     ((uint16_t)buf[17] << 8) | (uint16_t)buf[18];
        HPSStatusV2.FC1Temp2 =     ((uint16_t)buf[19] << 8) | (uint16_t)buf[20];
        HPSStatusV2.FC1Error =     ((uint16_t)buf[21] << 8) | (uint16_t)buf[22];
        HPSStatusV2.FC1FanDuty =   ((uint16_t)buf[23] << 8) | (uint16_t)buf[24];
        HPSStatusV2.FC1FanSpeed1 = ((uint16_t)buf[25] << 8) | (uint16_t)buf[26];
        HPSStatusV2.FC1FanSpeed2 = ((uint16_t)buf[27] << 8) | (uint16_t)buf[28];

        HPSStatusV2.FC2Volt =      ((uint16_t)buf[29] << 8) | (uint16_t)buf[30];
        HPSStatusV2.FC2Current =   ((uint16_t)buf[31] << 8) | (uint16_t)buf[32];
        HPSStatusV2.FC2Temp1 =     ((uint16_t)buf[33] << 8) | (uint16_t)buf[34];
        HPSStatusV2.FC2Temp2 =     ((uint16_t)buf[35] << 8) | (uint16_t)buf[36];
        HPSStatusV2.FC2Error =     ((uint16_t)buf[37] << 8) | (uint16_t)buf[38];
        HPSStatusV2.FC2FanDuty =   ((uint16_t)buf[39] << 8) | (uint16_t)buf[40];
        HPSStatusV2.FC2FanSpeed1 = ((uint16_t)buf[41] << 8) | (uint16_t)buf[42];
        HPSStatusV2.FC2FanSpeed2 = ((uint16_t)buf[43] << 8) | (uint16_t)buf[44];

        HPSStatusV2.FC3Volt =      ((uint16_t)buf[45] << 8) | (uint16_t)buf[46];
        HPSStatusV2.FC3Current =   ((uint16_t)buf[47] << 8) | (uint16_t)buf[48];
        HPSStatusV2.FC3Temp1 =     ((uint16_t)buf[49] << 8) | (uint16_t)buf[50];
        HPSStatusV2.FC3Temp2 =     ((uint16_t)buf[51] << 8) | (uint16_t)buf[52];
        HPSStatusV2.FC3Error =     ((uint16_t)buf[53] << 8) | (uint16_t)buf[54];
        HPSStatusV2.FC3FanDuty =   ((uint16_t)buf[55] << 8) | (uint16_t)buf[56];
        HPSStatusV2.FC3FanSpeed1 = ((uint16_t)buf[57] << 8) | (uint16_t)buf[58];
        HPSStatusV2.FC3FanSpeed2 = ((uint16_t)buf[59] << 8) | (uint16_t)buf[60];

        HPSStatusV2.FC4Volt =      ((uint16_t)buf[61] << 8) | (uint16_t)buf[62];
        HPSStatusV2.FC4Current =   ((uint16_t)buf[63] << 8) | (uint16_t)buf[64];
        HPSStatusV2.FC4Temp1 =     ((uint16_t)buf[65] << 8) | (uint16_t)buf[66];
        HPSStatusV2.FC4Temp2 =     ((uint16_t)buf[67] << 8) | (uint16_t)buf[68];
        HPSStatusV2.FC4Error =     ((uint16_t)buf[69] << 8) | (uint16_t)buf[70];
        HPSStatusV2.FC4FanDuty =   ((uint16_t)buf[71] << 8) | (uint16_t)buf[72];
        HPSStatusV2.FC4FanSpeed1 = ((uint16_t)buf[73] << 8) | (uint16_t)buf[74];
        HPSStatusV2.FC4FanSpeed2 = ((uint16_t)buf[75] << 8) | (uint16_t)buf[76];

        HPSStatusV2.LiVolt =           ((uint16_t)buf[77] << 8) | (uint16_t)buf[78];
        HPSStatusV2.LiCurrent =        ((uint16_t)buf[79] << 8) | (uint16_t)buf[80];
        HPSStatusV2.LiError =          ((uint16_t)buf[81] << 8) | (uint16_t)buf[82];

        HPSStatusV2.H2PressureH =      ((uint16_t)buf[83] << 8) | (uint16_t)buf[84];
        HPSStatusV2.H2PressureL =      ((uint16_t)buf[85] << 8) | (uint16_t)buf[86];
        HPSStatusV2.AmbTemperature =   ((uint16_t)buf[87] << 8) | (uint16_t)buf[88];
        HPSStatusV2.AmbHumidity =      ((uint16_t)buf[89] << 8) | (uint16_t)buf[90];
        HPSStatusV2.AmbControlStatus = ((uint16_t)buf[91] << 8) | (uint16_t)buf[92];
        HPSStatusV2_To_GC();
      }
      else if(len == 47)
      {
        HPSStatusV1._FCV = (buf[11]<<8) | buf[12];   //两数值合并
        HPSStatusV1._FCA = (buf[13]<<8) | buf[14]; 
        HPSStatusV1._FCWENDU = (buf[15]<<8) | buf[16];
        HPSStatusV1._FCW = (buf[17]<<8) | buf[18]; 
        HPSStatusV1._FCDCV = (buf[19]<<8) | buf[20];
        HPSStatusV1._FCDCA = (buf[21]<<8) | buf[22];
        HPSStatusV1._FCKW = (buf[27]<<8) | buf[28];
        HPSStatusV1._FCMPA = (buf[31]<<8) | buf[32];

        HPSStatusV1.Warning = (buf[41]<<8) | buf[42];
        HPSStatusV1.Fault = (buf[43]<<8) | buf[44];
        HPSStatusV1_To_GC();
      }
      
      last_frame_ms = AP_HAL::millis();
      //gcs().send_message(MSG_QH_FCSTATUS);
      //Log_Write_qhfc();
      break;
    case 0x06:
      if(buf[5] == 0x01)
      {
        OnOff_HPSAck = QHFC_CMD_PARAM_ON;
      }
      else if(buf[5] == 0x00)
      {
        OnOff_HPSAck = QHFC_CMD_PARAM_OFF;
      }
      _Clear_Cmd();

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

  Update_GC_OnOff();
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
        //------------------------------------------//
        uint16_t payload_len;
        if((recv_buf[1] == 0x06) && (protocol == AP_SerialManager::SerialProtocol_QHFC_V1))
          payload_len = 3;
        else
          payload_len = recv_buf[2];
        //------------------------------------------//
        if(recv_cnt == payload_len + 5)
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
void AP_QHFC::HPSStatusV2_To_GC(void)
{
  uint32_t Status = GCStatus.FCStatus & 0x03;
  uint16_t FCVoltage,FCCurrent;

  Status |= (HPSStatusV2.FCStatus & (~(uint32_t)0x00000003));
  GCStatus.FCStatus = Status;

  GCStatus.FCTemperature[0] = CalFCTemperature(0);
  GCStatus.FCTemperature[1] = CalFCTemperature(1);
  GCStatus.FCTemperature[2] = CalFCTemperature(2);
  GCStatus.FCTemperature[3] = CalFCTemperature(3);
  CalFCVoltCur(FCVoltage,FCCurrent);
  GCStatus.FCVoltage = FCVoltage;
  GCStatus.FCCurrent = FCCurrent;
  GCStatus.LIVoltage = HPSStatusV2.LiVolt;
  GCStatus.LICurrent = HPSStatusV2.LiCurrent;
  GCStatus.Press = HPSStatusV2.H2PressureH;
    
  GCStatus.AmbTemperature = HPSStatusV2.AmbTemperature;
  GCStatus.AmbHumidity = HPSStatusV2.AmbHumidity;
  GCStatus.AmbControlStatus = HPSStatusV2.AmbControlStatus;
}
void AP_QHFC::HPSStatusV1_To_GC(void)
{
  uint32_t Status = GCStatus.FCStatus & 0x03;

  //press
  if(HPSStatusV1.Warning & QHFC_V1_WARNING_PRESSLOW)
  {
    Status |= QHFC_V2_WARNING_PRESSLOW;
  }
  //temperature high
  if(HPSStatusV1.Fault & QHFC_V1_FAULT_TEMPHIGH)
  {
    Status |= QHFC_V2_FAULT_TEMPHIGH;
  }
  else if(HPSStatusV1.Warning & QHFC_V1_WARNING_TEMPHIGH)
  {
    Status |= QHFC_V2_WARNING_TEMPHIGH;
  }
  //FC voltage low
  if(HPSStatusV1.Fault & QHFC_V1_FAULT_VOLTAGELOW)
  {
    Status |= QHFC_V2_FAULT_FCVOLTAGELOW;
  }
  else if(HPSStatusV1.Warning & QHFC_V1_WARNING_VOLTAGELOW)
  {
    Status |= QHFC_V2_WARNING_FCVOLTAGELOW;
  }
  //LI voltage low
  //fan speed error
  if(HPSStatusV1.Warning & QHFC_V1_WARNING_FANSPEED)
  {
    Status |= QHFC_V2_WARNING_FANSPEED;
  }
  //h2 leakage
  //performance low
  if(HPSStatusV1.Warning & QHFC_V1_WARNING_PERFORMLOW)
  {
    Status |= QHFC_V2_WARNING_PERFORMLOW;
  }

  GCStatus.FCStatus = Status;
  GCStatus.FCTemperature[0] = HPSStatusV1._FCWENDU;
  GCStatus.FCTemperature[1] = 0;
  GCStatus.FCTemperature[2] = 0;
  GCStatus.FCTemperature[3] = 0;
  GCStatus.FCVoltage = HPSStatusV1._FCV;
  GCStatus.FCCurrent = HPSStatusV1._FCA;
  GCStatus.LIVoltage = HPSStatusV1._FCDCV;
  GCStatus.LICurrent = HPSStatusV1._FCDCA;
  GCStatus.Press = HPSStatusV1._FCMPA;
    
  GCStatus.AmbTemperature = 0;
  GCStatus.AmbHumidity = 0;
  GCStatus.AmbControlStatus = 0;
}

void AP_QHFC::Update_GC_OnOff(void)
{
  uint32_t Status = GCStatus.FCStatus;

  Status = Status & (~((uint32_t)0x00000003));
  if(OnOff_Status == QHFC_CMD_PARAM_ON)
  {
    Status |= QHFC_V2_ON;
  }
  GCStatus.FCStatus = Status;
}

uint16_t AP_QHFC::GetFCFault(void)
{
  uint8_t i;
  uint32_t Status = GCStatus.FCStatus;
  uint16_t Result = 0;

     // on/off bit
  for(i = 0;i < 15;i++)
  {
    Status >>= 2;
    if((Status & 0x03) == 0x02)
      Result |= 0x0001;
    Result <<= 1;
  }
  return Result;
}
uint16_t AP_QHFC::GetFCWarning(void)
{
  uint8_t i;
  uint32_t Status = GCStatus.FCStatus;
  uint16_t Result = 0;

     // on/off bit
  for(i = 0;i < 15;i++)
  {
    Status >>= 2;
    if((Status & 0x03) == 0x01)
      Result |= 0x0001;
    Result <<= 1;
  }
  return Result;
}

int16_t AP_QHFC::CalFCTemperature(uint8_t Ch)
{
  int16_t temp1,temp2;

  switch(Ch)
  {
    case 0:
      temp1 = HPSStatusV2.FC1Temp1;
      temp2 = HPSStatusV2.FC1Temp2;
      break;
    case 1:
      temp1 = HPSStatusV2.FC2Temp1;
      temp2 = HPSStatusV2.FC2Temp2;
      break;
    case 2:
      temp1 = HPSStatusV2.FC3Temp1;
      temp2 = HPSStatusV2.FC3Temp2;
      break;
    case 3:
      temp1 = HPSStatusV2.FC4Temp1;
      temp2 = HPSStatusV2.FC4Temp2;
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
  uint16_t v = HPSStatusV2.FC1Volt;
  uint16_t i;
  uint32_t p;

  if(HPSStatusV2.FC2Volt > v)v = HPSStatusV2.FC2Volt;
  if(HPSStatusV2.FC3Volt > v)v = HPSStatusV2.FC3Volt;
  if(HPSStatusV2.FC4Volt > v)v = HPSStatusV2.FC4Volt;

  p = (uint32_t)HPSStatusV2.FC1Volt * HPSStatusV2.FC1Current;
  p += (uint32_t)HPSStatusV2.FC2Volt * HPSStatusV2.FC2Current;
  p += (uint32_t)HPSStatusV2.FC3Volt * HPSStatusV2.FC3Current;
  p += (uint32_t)HPSStatusV2.FC4Volt * HPSStatusV2.FC4Current;

  i = p / v;

  Volt = v;
  Cur = i;
}

void AP_QHFC::Set_Cmd(uint16_t Param)
{
  if(Param == QHFC_CMD_PARAM_ON)
    OnOff_Cmd = QHFC_CMD_PARAM_ON;
  else if(Param == QHFC_CMD_PARAM_OFF)
    OnOff_Cmd = QHFC_CMD_PARAM_OFF;
}
//
//
// ******************发送氢航握手包
void AP_QHFC::send_woshoubao()
{
  uint8_t woshou[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x13, 0x04, 0x07};
  if (_port == nullptr)
  {
    return;
  }

  _port->write(woshou,8);
}


//关闭氢航燃料电池
void AP_QHFC::send_guan()  //发送需要别的地方触发
{
  uint8_t guan[]={0x01, 0x06, 0x00, 0x13, 0x00, 0x00, 0x78, 0x0F};
  if (_port == nullptr)
  {
    return;
  }
  _port->write(guan,8); 
}

//开启氢航燃料电池
void AP_QHFC::send_kai()  //发送需要别的地方触发
{
  uint8_t kai[]={0x01, 0x06, 0x00, 0x13, 0x00, 0x01, 0xB9, 0xCF};
  if (_port == nullptr)
  {
    return;
  }
  _port->write(kai,8); //发送数据
}

void AP_QHFC::_Send_Cmd(void)
{
  if(OnOff_Cmd_buf == QHFC_CMD_PARAM_ON)
  {
    send_kai();
  }
  else if(OnOff_Cmd_buf == QHFC_CMD_PARAM_OFF)
  {
    send_guan();
  }
  OnOff_HPSAck = QHFC_CMD_PARAM_NONE;
}
void AP_QHFC::_Clear_Cmd(void)
{
  OnOff_Cmd_buf = QHFC_CMD_PARAM_NONE;
}

void AP_QHFC::tick(void)// 定时器函数
{
  uint32_t now = AP_HAL::millis();

  //
  uint32_t deltaT = now - last_tick_ms;
  last_tick_ms = now;

  Cmd_Timeout_Cnt += deltaT;

  if((OnOff_Cmd == QHFC_CMD_PARAM_ON) || (OnOff_Cmd == QHFC_CMD_PARAM_OFF ))
  {
    Cmd_Retry_Cnt = 0;

    OnOff_Cmd_buf = OnOff_Cmd;
    
    OnOff_Cmd = QHFC_CMD_PARAM_NONE;
  }

  if((OnOff_Cmd_buf == QHFC_CMD_PARAM_ON) || (OnOff_Cmd_buf == QHFC_CMD_PARAM_OFF))
  {
    if(Cmd_Timeout_Cnt >= CMD_TIMEOUT_MAX)
    {
      if(Cmd_Retry_Cnt < CMD_RETRY_CNT)
      {
        _Send_Cmd();
        Cmd_Retry_Cnt++;
      }
      else
      {
        _Clear_Cmd();
      }
      Cmd_Timeout_Cnt = 0;
    }
  }
  else
  {
    if(Cmd_Timeout_Cnt >= CMD_TIMEOUT_MAX)
    {
      send_woshoubao();
      Cmd_Timeout_Cnt = 0;
    }
  }

  if(OnOff_HPSAck == QHFC_CMD_PARAM_ON)
  {
    OnOff_Status = QHFC_CMD_PARAM_ON;
  }
  else if(OnOff_HPSAck == QHFC_CMD_PARAM_OFF)
  {
    OnOff_Status = QHFC_CMD_PARAM_OFF;
  }
}

FCFailsafeAction AP_QHFC::handle_FC_failsafe(bool is_armed)
{
  uint16_t FCFault_Cur;
  uint16_t FCWarning_Cur;
  FCFailsafeAction Ret = FCFailsafeAction::NONE;

  FCFault_Cur = GetFCFault();
  FCWarning_Cur = GetFCWarning();

  if(is_armed == true)
  {
    if((FCFault_Last != 0) || (FCFault_Cur != 0))
    {
      CurAction = FCFailsafeAction::LAND;
      if(((FCFault_Last ^ FCFault_Cur) & FCFault_Cur) != 0)
      {
        LastAction = FCFailsafeAction::NONE;
      }
      FCFault_Last |= FCFault_Cur;
    }
    else if((FCWarning_Last != 0) || (FCWarning_Cur != 0))
    {
      CurAction = FCFailsafeAction::RTL;
      if(((FCWarning_Last ^ FCWarning_Cur) & FCWarning_Cur) != 0)
      {
        LastAction = FCFailsafeAction::NONE;
      }
      FCWarning_Last |= FCWarning_Cur;
    }
  }
  else
  {
    CurAction = FCFailsafeAction::NONE;
    FCFault_Last = 0;
    FCWarning_Last = 0;
  }

  if(LastAction != CurAction)
  {
    Ret = CurAction;
    LastAction = CurAction;
  }

  return Ret;
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
