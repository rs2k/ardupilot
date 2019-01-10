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

/*
  backend driver for airspeed from a I2C EAGLETREE sensor
 */
#include "AP_Airspeed_Eagletree.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define EAGLETREE_I2C_ADDR 0x4D

#define EAGLE_TREE_RAW_VALUE_MAX        4048.0f //Max value we can expect from sensor
#define EAGLE_TREE_RAW_VALUE_MIN        896.0f  //Min value we can expect from the sensor, anything less than this is negative pressure and useless for our needs
#define EAGLE_TREE_PA_VALUE_MAX         3920.0f //Max value the sensor can read in Pascals

// no temp sensor in Eagle Tree Pitot EXP, use standard day temp, can we pull this from elsewhere?
#define STANDARD_DAY_TEMP_CELS          15.0f   //SEE: https://en.wikipedia.org/wiki/Standard_day


AP_Airspeed_EAGLETREE::AP_Airspeed_EAGLETREE(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

// probe and initialise the sensor
bool AP_Airspeed_EAGLETREE::init()
{
    uint16_t i = 0;

    bool found = false;

    FOREACH_I2C_EXTERNAL(i) {
        _dev = hal.i2c_mgr->get_device(i, EAGLETREE_I2C_ADDR);
        if (!_dev) {
            continue;
        }
        WITH_SEMAPHORE(_dev->get_semaphore());

        // lots of retries during probe
        _dev->set_retries(10);
    
        _measure();
        hal.scheduler->delay(10);
        _collect();

        found = (_last_sample_time_ms != 0);
        if (found) {
            printf("EAGLETREE: Found sensor on bus %u address 0x%02x\n", i, EAGLETREE_I2C_ADDR);
            break;
        }
    }
    if (!found) {
        printf("EAGLETREE: no sensor found\n");
        return false;
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_EAGLETREE::_timer, void));
    return true;
}

// start a measurement
void AP_Airspeed_EAGLETREE::_measure()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0;
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

// scale the raw data to pascals
float AP_Airspeed_EAGLETREE::_get_pressure(int16_t dp_raw) const
{
    // constrain the readings to positive values onlt
    float rawFloat = constrain_float((float)dp_raw,EAGLE_TREE_RAW_VALUE_MIN,EAGLE_TREE_RAW_VALUE_MAX);

    // range the raw data to pascals
    press = (rawFloat - EAGLE_TREE_RAW_VALUE_MIN) * (EAGLE_TREE_PA_VALUE_MAX - EAGLE_TREE_PA_VALUE_MIN) / (EAGLE_TREE_RAW_VALUE_MAX - EAGLE_TREE_RAW_VALUE_MIN);

    return press;
}

// read the values from the sensor
void AP_Airspeed_EAGLETREE::_collect()
{
    uint8_t data[2];
    uint8_t data2[2];

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return;
    }
    // reread the data, so we can attempt to detect bad inputs
    if (!_dev->transfer(nullptr, 0, data2, sizeof(data2))) {
        return;
    }

    uint8_t status = (data[0] & 0xC0) >> 6;
    // only check the status on the first read, the second read is expected to be stale
    if (status == 2 || status == 3) {
        return;
    }

    int16_t dp_raw;
    dp_raw = (data[0] << 8) + data[1];
    dp_raw = 0x3FFF & dp_raw;

    int16_t dp_raw2;
    dp_raw2 = (data2[0] << 8) + data2[1];
    dp_raw2 = 0x3FFF & dp_raw2;

    // reject any values that are the absolute minimum or maximums these
    // can happen due to gnd lifts or communication errors on the bus
    if (dp_raw  == 0x3FFF || dp_raw  == 0 ||
        dp_raw2 == 0x3FFF || dp_raw2 == 0) {
        return;
    }

    // reject any double reads where the value has shifted in the upper more than
    // 0xFF
    if (abs(dp_raw - dp_raw2) > 0xFF) {
        return;
    }

    float press  = _get_pressure(dp_raw);
    float press2 = _get_pressure(dp_raw2);
    float temp  = STANDARD_DAY_TEMP_CELS;
    float temp2 = STANDARD_DAY_TEMP_CELS;

    WITH_SEMAPHORE(sem);

    _press_sum += press + press2;
    _temp_sum += temp + temp2;
    _press_count += 2;
    _temp_count += 2;

    _last_sample_time_ms = AP_HAL::millis();
}

// 50Hz timer
void AP_Airspeed_EAGLETREE::_timer()
{
    if (_measurement_started_ms == 0) {
        _measure();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_EAGLETREE::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (_press_count > 0) {
        _pressure = _press_sum / _press_count;
        _press_count = 0;
        _press_sum = 0;
    }

    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
// no temp sensor in Eagle Tree Pitot EXP, does this still need to be here?
bool AP_Airspeed_EAGLETREE::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (_temp_count > 0) {
        _temperature = _temp_sum / _temp_count;
        _temp_count = 0;
        _temp_sum = 0;
    }

    temperature = _temperature;
    return true;
}
