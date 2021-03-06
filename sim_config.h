/**
 * Copyright (c) 2020 Open Ring Project, All rights reserved
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

//// simulator settings ////
#ifndef __SIM_CONFIG_H__
#define __SIM_CONFIG_H__

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */  
#define MIN_CELCIUS_DEGREES             3688                                        /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS             3972                                        /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT       36                                          /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */

#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define MIN_SPO2_LEVEL                      80                                          /**< Minimum SpO2 level as returned by the simulated measurement function. */
#define MAX_SPO2_LEVEL                      100                                         /**< Maximum SpO2 level as returned by the simulated measurement function. */
#define SPO2_LEVEL_INCREMENT                1                                           /**< Value by which the SpO2 level is incremented/decremented for each call to the simulated measurement function. */


#define SPEED_AND_CADENCE_MEAS_INTERVAL     1000                                    /**< Speed and cadence measurement interval (milliseconds). */

#define MIN_SPEED_MPS                       0.5                                     /**< Minimum speed in meters per second for use in the simulated measurement function. */
#define MAX_SPEED_MPS                       6.5                                     /**< Maximum speed in meters per second for use in the simulated measurement function. */
#define SPEED_MPS_INCREMENT                 1.5                                     /**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */
#define MIN_RUNNING_SPEED                   3                                       /**< speed threshold to set the running bit. */

#define MIN_CADENCE_RPM                 40                                      /**< Minimum cadence in revolutions per minute for use in the simulated measurement function. */
#define MAX_CADENCE_RPM                 160                                     /**< Maximum cadence in revolutions per minute for use in the simulated measurement function. */
#define CADENCE_RPM_INCREMENT           20                                      /**< Value by which cadence is incremented/decremented in the simulated measurement function. */

#define MIN_STRIDE_LENGTH               20                                      /**< Minimum stride length in decimeter for use in the simulated measurement function. */
#define MAX_STRIDE_LENGTH               125                                     /**< Maximum stride length in decimeter for use in the simulated measurement function. */
#define STRIDE_LENGTH_INCREMENT         5                                       /**< Value by which stride length is incremented/decremented in the simulated measurement function. */

#define BATTERY_SIMULATOR_COUNT 1
#define DEVICE_BATTERY_COUNT BATTERY_SIMULATOR_COUNT

#define TEMPERATURE_SIMULATOR_COUNT 0
#define SOC_TEMPERATURE_SENSOR_COUNT 1
#define TEMPERATURE_SENSOR_COUNT SOC_TEMPERATURE_SENSOR_COUNT+TEMPERATURE_SIMULATOR_COUNT

//#define HEART_RATE_SENSOR_COUNT HEART_RATE_SIMULATOR_COUNT

//#define PULSE_OXIMETER_SIMULATOR_COUNT 1
//#define PULSE_OXIMETER_COUNT PULSE_OXIMETER_SIMULATOR_COUNT

#define INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT 1
#define INERTIAL_MEASUREMENT_UNIT_COUNT INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT

#endif //__SIM_CONFIG_H__