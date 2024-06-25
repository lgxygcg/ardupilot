#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_QHFC/AP_QHFC.h>

#define AP_BATTMONITOR_QHFC_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class BattInfoCb;
class BattInfoAuxCb;
class MpptStreamCb;

class AP_BattMonitor_QHFC : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_QHFC_Type {
        QHFC_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_QHFC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_QHFC_Type type, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    void init() override;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    bool has_temperature() const override { return _has_temperature; }

    bool has_current() const override { return true; }

private:
    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_QHFC_Type _type;

    AP_QHFC* _ap_qhfc;
 
    bool _has_temperature;
    uint8_t _instance;                  // instance of this battery monitor
    AP_Float _curr_mult;                 // scaling multiplier applied to current reports for adjustment
};
