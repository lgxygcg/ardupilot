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
    //<-- ------------------------------------------------------------------- ->//
}

/*
 * init - perform required initialisation
 */
void AP_QHFC::init(const AP_SerialManager& serial_manager)
{
     if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_QHFC, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZS_RX,AP_SERIALMANAGER_OPENMV_BUFSIZS_TX);
         
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_QHFC::tick, void));//定时函数用
        update();
      } 
}

bool AP_QHFC::update()
{
    if (_port == NULL) 
        return false;   

       int16_t numc = _port->available();
      
        data = 0;

   for(int16_t i = 0; i < numc; i++){
          data =_port->read();
        switch(_step) {
        case 0:
           recv_vnt = 0;
           recv_duf[recv_vnt] = data;
           if(recv_duf[0] == (0x01)){
             recv_vnt ++;
             _step = 1;
               } 
             break;
            
        case 1:
           recv_duf[recv_vnt] = data;          
           if(recv_duf[1] == (0x03)){    //判断数据包
           recv_vnt ++;
              _step = 2;
               } 
            else if (recv_duf[1] == (0x06)){       //判断是不是控制指令数据   
           recv_vnt ++;
              _step = 4;
             }
             else{
             _step = 0;
              } 
             break;
//******************状态数据包***************
        case 2:        
           recv_duf[recv_vnt] = data;          
          if(recv_duf[2] == (0x26)){
           recv_vnt ++;
              _step = 3;
               } 
             else{
             _step = 0;
              } 
             break;
             
        case 3:
            recv_duf[recv_vnt] = data;
            recv_vnt ++;
          if(recv_vnt == 43){
              _step = 0;
              QHFC_crc = calc_crc_modbus(recv_duf, recv_vnt - 2);
              crch = QHFC_crc >> 8;
              chal = QHFC_crc & 0xFF;

            if((recv_duf[recv_vnt - 2] == chal) && (recv_duf[recv_vnt - 1] == crch)) {
               _FCV = (recv_duf[11]<<8) | recv_duf[12];   //两数值合并
               _FCA = (recv_duf[13]<<8) | recv_duf[14]; 
               _FCWENDU = (recv_duf[15]<<8) | recv_duf[16];
               _FCW = (recv_duf[17]<<8) | recv_duf[18]; 
               _FCDCV = (recv_duf[19]<<8) | recv_duf[20];
               _FCDCA = (recv_duf[21]<<8) | recv_duf[22];
               _FCKW = (recv_duf[27]<<8) | recv_duf[28];
               _FCMPA = (recv_duf[31]<<8) | recv_duf[32];

               last_frame_ms = AP_HAL::millis();
               gcs().send_message(MSG_QH_FCSTATUS);
               //<-- ------------------------------------------------------------------- ->//
                //Log_Write_qhfc();
                //<-- ------------------------------------------------------------------- ->//
               return true;
               }
             else{
               _step = 0;
                break;
               }      
             }
          else{
             _step = 3;
              } 
             break;
//******************** 控制指令数据***********
        case 4:
            recv_duf[recv_vnt] = data;
            recv_vnt ++;
          if(recv_vnt == 8){
              _step = 0;
              QHFC_crc = calc_crc_modbus(recv_duf, recv_vnt - 2);
              crch = QHFC_crc >> 8;
              chal = QHFC_crc & 0xFF;
              if((recv_duf[recv_vnt - 2] == chal) && (recv_duf[recv_vnt - 1] == crch)) {
                qh_onoff = recv_duf[5];

                 last_frame_ms = AP_HAL::millis();
                 gcs().send_message(MSG_QH_FCSTATUS);
                 //<-- ------------------------------------------------------------------- ->//
                 //Log_Write_qhfc();
                 //<-- ------------------------------------------------------------------- ->//
                 return true;
               }
              else{
                  _step = 0;
                  break;
                  }
              }
           else{
             _step = 4;
              } 
             break;       
        }
    }

    return false;
}

//////
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
    
    if (now - _last_frame_ms > 1000) {
        _last_frame_ms = now;
      if((qh_on == 5 ) || (qh_on == 8 )){
        if (qh_on == 5 ){
            send_kai();
            qh_on = FCkai;
            if (qh_onoff ==0x01) {
              qh_on = 1;
             /// qh_kaiguan = 5;
              gcs().send_message(MSG_QH_FCSTATUS);
            }
          }
        if (qh_on == 8 ){
           send_guan();
           qh_on = FCguan;
           if (qh_onoff == 0x00) {
             qh_on = 0;
             //qh_kaiguan = 8;
             gcs().send_message(MSG_QH_FCSTATUS);
            }
       }
        
      }
      else{
          send_woshoubao();
         }
    }
     return ;
     //}
}   //uint8_t checksum =0;
///后加程序 
////后加的
namespace AP {

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
