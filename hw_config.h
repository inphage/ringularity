/**
 * Copyright (c) 2021 Open Ring Project, All rights reserved
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without 
 * restriction, including without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#define TWI_MNGR_QUEUE_SIZE 1
#define TWI_MSG_BUFF_SIZE 16

#include "dev_twi.h"
#include "dev_battery.h"
#include "dev_temperature_sensor.h"
#include "dev_heart_rate_sensor.h"
#include "dev_pulse_oximeter.h"
#include "dev_inertial_measurement_unit.h"

//#define TWI_BUS0
DECL_DEV_TWI(0);
DECL_DEV_BATTERY(0);
DECL_DEV_TEMPERATURE_SENSOR(0);
DECL_DEV_HEART_RATE_SENSOR(0);
DECL_DEV_PULSE_OXIMETER(0);
DECL_DEV_INERTIAL_MEASUREMENT_UNIT(0);

#define DEVICE_DEFS \
    DEF_DEV_TWI(0);\
    DEF_DEV_BATTERY(0);\
    DEF_DEV_TEMPERATURE_SENSOR(0);\
    DEF_DEV_HEART_RATE_SENSOR(0);\
    DEF_DEV_PULSE_OXIMETER(0);\
    DEF_DEV_INERTIAL_MEASUREMENT_UNIT(0);

#define SYSTEM_MODE_SIMULATOR 1
#define SYSTEM_MODE_PHYSICAL 2
#define SYSTEM_MODE_TEST 3
#define SYSTEM_MODE SYSTEM_MODE_PHYSICAL



#include "dev_soc_temp.h"
#include "dev_max30101.h"
#include "battery_simulator.h"
#include "temperature_sensor_simulator.h"

#define BATTERY_SIMULATOR_COUNT 1
#define DEVICE_BATTERY_COUNT BATTERY_SIMULATOR_COUNT

#define TEMPERATURE_SIMULATOR_COUNT 0
#define SOC_TEMPERATURE_SENSOR_COUNT 1
#define TEMPERATURE_SENSOR_COUNT SOC_TEMPERATURE_SENSOR_COUNT+TEMPERATURE_SIMULATOR_COUNT

#define HEART_RATE_SIMULATOR_COUNT 0
#define HEART_RATE_SENSOR_COUNT HEART_RATE_SIMULATOR_COUNT
#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */
#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define PULSE_OXIMETER_SIMULATOR_COUNT 0
#define DEVICE_MAX30101_COUNT 1
#define PULSE_OXIMETER_COUNT PULSE_OXIMETER_SIMULATOR_COUNT+PULSE_OXIMETER_MAX30101_COUNT

#define PULSE_OXIMETER_MEAS_INTERVAL        APP_TIMER_TICKS(1000)                       /**< SpO2 level measurement interval (ticks). */

#define INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT 1
#define INERTIAL_MEASUREMENT_UNIT_COUNT INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT


#include "inertial_measurement_unit_simulator.h"
#if SYSTEM_MODE == SYSTEM_MODE_SIMULATOR
#include "sim_config.h"
#include "temperature_sensor_simulator.h"
#include "heart_rate_sensor_simulator.h"
#include "pulse_oximeter_simulator.h"
#elseif SYSTEM_MODE == SYSTEM_MODE_PHYSICAL

#endif

//// DEVICE FACTORY //
#define DEVICE0_OPEN ret_code_t device0_open() { NRF_LOG_INFO("device0_open: twi bus 0");\
     nrf_drv_twi_config_t init_params = \
        {\
            .scl = ARDUINO_SCL_PIN, /*ARDUINO_SCL_PIN,                 ///< SCL pin number. */\
            .sda = ARDUINO_SDA_PIN, /*ARDUINO_SDA_PIN,                 ///< SDA pin number. */\
            .frequency = NRF_TWI_FREQ_100K, /*< TWI frequency. 0-400KHz */\
            .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  /*< Interrupt priority.*/\
            .clear_bus_init = true,      /*< Clear bus during init.*/\
            .hold_bus_uninit = false     /*< Hold pull up state on gpio pins after uninit.*/\
        };\
     return device$twi_bus$open(0, &init_params, &dev_twi_bus0);\
     }
//DEVICE0_OPEN;
#define DEVICE0_OPENER_DEFS DEVICE0_OPEN
#define DEVICE0_OPEN_SEQ device0_open

#define DEVICE1_OPEN ret_code_t device1_open() {\
    NRF_LOG_INFO("device1_open: battery simulator");\
    device_battery_simulator_init_t init_params = {};\
    return device$battery_simulator$open(0, &init_params, &dev_battery0);\
} 
#define DEVICE1_OPENER_DEFS DEVICE0_OPENER_DEFS DEVICE1_OPEN
#define DEVICE1_OPEN_SEQ DEVICE0_OPEN_SEQ,device1_open

#define DEVICE2_OPEN ret_code_t device2_open() {\
    NRF_LOG_INFO("device2_open: soc temperature");\
    device_soc_temperature_params_t params={};\
    return device$dev_soc_temp$open(0, &params, &dev_temperature_sensor0);\
}
#define DEVICE2_OPENER_DEFS DEVICE1_OPENER_DEFS DEVICE2_OPEN
#define DEVICE2_OPEN_SEQ DEVICE1_OPEN_SEQ,device2_open

#define DEVICE3_OPEN ret_code_t device3_open() {\
    NRF_LOG_INFO("device3_open: max30101 pulse oximeter");\
    device_max30101_params_t params = {.p_sensor_data = &_twi_bus0,.sensor_addr=MAX30101_WRITE_ADDRESS};\
    ret_code_t ret_code = device$max30101$open(0, &params, &dev_pulse_oximeter0);\
    ASSERT(dev_pulse_oximeter0);\
    return ret_code;\
}
#define DEVICE3_OPENER_DEFS DEVICE2_OPENER_DEFS DEVICE3_OPEN
#define DEVICE3_OPEN_SEQ DEVICE2_OPEN_SEQ,device3_open

#define DEVICE4_OPEN ret_code_t device4_open() {\
    NRF_LOG_INFO("device4_open: IMU simulator");\
    device_inertial_measurement_unit_sim_params_t params = {};\
    return device$inertial_measurement_unit_sim$open(0, &params, &dev_inertial_measurement_unit0);\
}
#define DEVICE4_OPENER_DEFS DEVICE3_OPENER_DEFS DEVICE4_OPEN
#define DEVICE4_OPEN_SEQ DEVICE3_OPEN_SEQ,device4_open


// EXPERIMENTAL - doesn't do anything useful yet
#define DEV_SEQ_CONCAT(X1,X2) device ## X1 ## _open,
#define DEV_SEQUENCE(X) MACRO_MAP_FOR_N(X,DEV_SEQ_CONCAT)

#define DEV_OPENER(seq,dev_id,dev_type_int,dev_type_ext,init_params)             \
    //dev_type##_t dev_type ## dev_id;                                                         \
    ret_code_t device##seq##_open() {                             \
    device_##dev_type_ext##_init_params_t params = init_params;        \
    return device$##dev_type_int##$open(dev_id,init_params, &dev_ ## dev_type_ext ## dev_id);   \
    }                                                                                       \
#undef DEVICE_INIT_SEQUENCE                                                                 \
#define DEVICE_INIT_SEQUENCE2 {DEV_SEQUENCE(4)}

#define DEV4 DEV_OPENER(4,0,inertial_measurement_unit_sim,inertial_measurement_unit,{})
//DEV_SEQUENCE(4)

#define DEVICE_OPENER_DEFS DEVICE4_OPENER_DEFS
#define DEVICE_INIT_SEQUENCE {DEVICE4_OPEN_SEQ}
//#define DEVICE_INIT_SEQUENCE {DEV_SEQUENCE(4)}
