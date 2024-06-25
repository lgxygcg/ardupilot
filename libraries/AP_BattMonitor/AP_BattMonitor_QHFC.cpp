#include "AP_BattMonitor_config.h"

#if AP_BATTERY_QHFC_BATTERYINFO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_QHFC.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_QHFC/AP_QHFC.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <ardupilot/equipment/power/BatteryInfoAux.hpp>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_QHFC::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no QHFC param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_QHFC, _curr_mult, 1.0),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_QHFC::AP_BattMonitor_QHFC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_QHFC_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_QHFC::init()
{
}

// read - read the voltage and current
void AP_BattMonitor_QHFC::read()
{
    AP_QHFC *fc = AP_QHFC::get_singleton();
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > AP_BATTMONITOR_QHFC_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
    }
    // Copy over relevant states over to main state
    _state.temperature = 0;
    _state.temperature_time = 0;
    _state.voltage = fc->GCStatus.FCVoltage;
    _state.current_amps = fc->GCStatus.FCCurrent;
    _state.consumed_mah = 0;
    _state.consumed_wh = 0;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    _state.time_remaining = 0;
    _state.has_time_remaining = 0;
    _state.is_powering_off = _interim_state.is_powering_off;
    memset(_state.cell_voltages.cells, 0, sizeof(_state.cell_voltages));

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

#endif

