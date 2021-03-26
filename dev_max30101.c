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

#include "dev_include.h"
#include "hw_config.h"
#include "dev_max30101.h"
#include "max30101.h"
#include "app_timer.h"
#include "spo2_algorithm.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#define BUFFER_LENGTH (100)
#define FIFO_POLL_INTERVAL APP_TIMER_TICKS(250)

APP_TIMER_DEF(_fifo_read_timer);                                 /**< RSC measurement timer. */

typedef struct device_max30101_s {
    device_pulse_oximeter_t ext_iface;
    max30101_instance_t sensor;
    uint32_t ir_buffer[BUFFER_LENGTH*2]; //infrared LED sensor data
    uint32_t red_buffer[BUFFER_LENGTH*2];  //red LED sensor data
    uint32_t green_buffer[BUFFER_LENGTH*2]; //green LED sensor data
    uint32_t write_idx;
    bool data_ready;
    //uint32_t ts_last_good_read;
    uint32_t ts_last_processing_time;
    uint16_t spo2_measurement;
    bool spo2_valid;
    uint16_t hr_measurement;
    bool hr_valid;
} device_max30101_t;


static void start_polling(device_max30101_t *p_dev);
static void process_data(device_max30101_t *p_dev);
static void read_sensor_data(void *p_context);
ret_code_t config_sensor(device_max30101_t *p_dev);
static void read_samples_cbx(device_max30101_t *p_dev, uint8_t *pb_reg_data);

//work-around for linking to C++ code
extern void _Z38maxim_heart_rate_and_oxygen_saturationPjiS_PiPaS0_S1_(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
                int32_t *pn_heart_rate, int8_t *pch_hr_valid);

#define MAXIM_HEART_RATE_AND_OXYGEN_SATURATION _Z38maxim_heart_rate_and_oxygen_saturationPjiS_PiPaS0_S1_

static void sensor_spin_wait(device_max30101_t *p_dev) {
    while(nrf_drv_twi_is_busy(&p_dev->sensor.p_sensor_data->p_twi_mngr->twi)) {
        nrf_delay_ms(1);
    }
}

static void wait_until_data_ready(device_max30101_t *p_instance) {
    ASSERT(_fifo_read_timer->active);
    while(false == p_instance->data_ready) {
        nrf_delay_ms(100);
    }
}

static ret_code_t plx_get_spo2(device_pulse_oximeter_t *p_dev, uint16_t *p_measurement, bool *p_valid) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_max30101_t *p_instance = (device_max30101_t *)p_dev;
    start_polling(p_instance);
    if(p_instance->data_ready) {
        process_data(p_instance);
        *p_measurement = p_instance->spo2_measurement;
        *p_valid = p_instance->spo2_valid;
    } else {
        return NRF_ERROR_BUSY;
    }

    return NRF_SUCCESS;
}

static ret_code_t plx_get_pulse_rate(device_pulse_oximeter_t *p_dev, uint16_t *p_measurement, bool *p_valid) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_max30101_t *p_instance = (device_max30101_t *)p_dev;
    start_polling(p_instance);
    if(p_instance->data_ready) {
        process_data(p_instance);
        *p_measurement = p_instance->hr_measurement;
        *p_valid = p_instance->hr_valid;
    } else {
        return NRF_ERROR_BUSY;
    } 

    return NRF_SUCCESS;
}

static device_max30101_t _devs[DEVICE_MAX30101_COUNT];
ret_code_t device$max30101$open(uint32_t device_id, device_max30101_params_t *p_params, 
    device_pulse_oximeter_t **pp_dev) {
    ret_code_t err_code;

    if(device_id >= DEVICE_MAX30101_COUNT) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    ASSERT(p_params);
    ASSERT(pp_dev);
    if(NULL == pp_dev || NULL == p_params) {
        return NRF_ERROR_NULL;
    }

    device_max30101_t *p_instance = &_devs[device_id];
    p_instance->spo2_measurement = 0;
    p_instance->spo2_valid = false;
    p_instance->hr_measurement = 0;
    p_instance->hr_valid = false;

    max30101_init_params_t api_init_params = {
        .p_sensor_data = p_params->p_sensor_data,
        .sensor_addr = p_params->sensor_addr
    };
    err_code = max30101_init(&p_instance->sensor, &api_init_params);
    if(err_code != NRF_SUCCESS) {
        return err_code;
    }

    //fifo read timer
    err_code = app_timer_create(&_fifo_read_timer,
                                APP_TIMER_MODE_REPEATED,
                                read_sensor_data);
    if(err_code != NRF_SUCCESS) {
        return err_code;
    }

    p_instance->ext_iface.read_pulse_rate = plx_get_pulse_rate;
    p_instance->ext_iface.read_spo2 = plx_get_spo2;
    p_instance->ext_iface.initialized = true;

    *pp_dev = &p_instance->ext_iface;

    return NRF_SUCCESS;
}

void start_polling(device_max30101_t *p_dev) {
    if(_fifo_read_timer->active == true) {
        return;
    }

    p_dev->data_ready = false;
    p_dev->write_idx = 0;
    config_sensor(p_dev);
    app_timer_start(_fifo_read_timer, FIFO_POLL_INTERVAL, p_dev);
}

void stop_polling(device_max30101_t *p_dev) {
    app_timer_stop(_fifo_read_timer);
    max30101_shutdown(&p_dev->sensor, true);
}

#define UNPACK_MAX30101_SAMPLE(b0, b1, b2) ((b0 << 16) + (b1 << 8) + b2)

static void read_samples_cb(ret_code_t result, void * p_register_data) {
    for(int dev_idx = 0; dev_idx<DEVICE_MAX30101_COUNT; dev_idx++) {
        device_max30101_t *p_dev = &_devs[dev_idx];
        read_samples_cbx(p_dev, (uint8_t *)p_register_data);
    }
}

static void read_samples_cbx(device_max30101_t *p_dev, uint8_t *pb_reg_data) {
    uint32_t write_idx = p_dev->write_idx;
    p_dev->red_buffer[write_idx] = UNPACK_MAX30101_SAMPLE(pb_reg_data[0], pb_reg_data[1], pb_reg_data[2]);
    p_dev->ir_buffer[write_idx] =  UNPACK_MAX30101_SAMPLE(pb_reg_data[3], pb_reg_data[4], pb_reg_data[5]);
    p_dev->green_buffer[write_idx] =  UNPACK_MAX30101_SAMPLE(pb_reg_data[6], pb_reg_data[7], pb_reg_data[8]);
    write_idx++;
    if(write_idx > BUFFER_LENGTH) {
        p_dev->data_ready = true;
    }
    if(write_idx >= 2*BUFFER_LENGTH) {
        write_idx = 0;
    }
    p_dev->write_idx = write_idx;
    //NRF_LOG_INFO("fifo write pointer: %X", result); 
    //NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[0], pb_reg_data[1], pb_reg_data[2]);
    //NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[3], pb_reg_data[4], pb_reg_data[5]);
    //NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[6], pb_reg_data[7], pb_reg_data[8]);
}


ret_code_t config_sensor(device_max30101_t *p_dev) {
    ret_code_t err_code;

    //reset
    err_code = max30101_reset(&p_dev->sensor);
    NRF_LOG_INFO("max30101_reset: %u", err_code);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_dev);

    //err_code = max30101_init(&plx_sensor);
    //NRF_LOG_INFO("max30101_init: %u", err_code);

    //config pulse oximeter
    err_code = max30101_set_led_pulse_amplitudes(&p_dev->sensor, 60, 60, 0, 0);
    NRF_LOG_INFO("max30101_set_led_pulse_amplitudes: %u", err_code);
    sensor_spin_wait(p_dev);
    err_code = max30101_set_fifo_sample_average(&p_dev->sensor, MAX30101_SMP_AVE_4);
    NRF_LOG_INFO("max30101_set_fifo_sample_average: %u", err_code);
    sensor_spin_wait(p_dev);
    err_code = max30101_set_operation_mode(&p_dev->sensor, MAX30101_OPERATING_MODE_SPO2); //red+IR
    NRF_LOG_INFO("max30101_set_operation_mode: %u", err_code);
    sensor_spin_wait(p_dev);
    err_code = max30101_set_led_mode_control(&p_dev->sensor, MAX30101_LED_MODE_CONTROL_RED, MAX30101_LED_MODE_CONTROL_IR, 
                                              MAX30101_LED_MODE_CONTROL_NONE, MAX30101_LED_MODE_CONTROL_NONE);
    NRF_LOG_INFO("max30101_set_led_mode_control: %u", err_code);
    sensor_spin_wait(p_dev);
    err_code = max30101_set_spo2_config(&p_dev->sensor, MAX30101_ADC_RANGE2, MAX30101_SAMPLING_RATE_100, MAX30101_LED_PULSE_WIDTH_411);
    NRF_LOG_INFO("max30101_set_spo2_config: %u", err_code);
}

//void twi_wait_while_busy(const nrf_drv_twi_t *p_twi) {
//    //idle_state_handle();
//    app_sched_execute();
//    while(nrf_drv_twi_is_busy(p_twi)) {
//        idle_state_handle();
//    }
//}


static void read_sensor_data(void *p_context) {
    ret_code_t err_code;
    device_max30101_t *p_dev = (device_max30101_t *)p_context;
    uint32_t num_samples_available;
    uint8_t fifo_read_buffer[9];

    while(true) {
        sensor_spin_wait(p_dev);
        err_code = max30101_samples_available(&p_dev->sensor, &num_samples_available);
        if(num_samples_available == 0) {
            return;
        }
        do {
            sensor_spin_wait(p_dev);
            err_code = max30101_read_fifo_data(&p_dev->sensor, read_samples_cb, fifo_read_buffer);
            if((err_code) != NRF_SUCCESS) {
                NRF_LOG_INFO("max30101_get_samples: %u", err_code);
                return;
            }
        } while(num_samples_available--);
    }
}

static void process_data(device_max30101_t *p_dev) {
    return;

    ASSERT(p_dev);
    app_timer_resume();
    uint32_t ts = app_timer_cnt_get();
    if(ts == p_dev->ts_last_processing_time) {
        return;
    }
    uint32_t start = (p_dev->write_idx < BUFFER_LENGTH) ? BUFFER_LENGTH : 0;
    int32_t spo2_measurement, hr_measurement;
    uint8_t spo2_valid, hr_valid;
    MAXIM_HEART_RATE_AND_OXYGEN_SATURATION(p_dev->ir_buffer+start, BUFFER_LENGTH, p_dev->red_buffer+start, 
        &spo2_measurement, &hr_valid, &hr_measurement, &hr_valid);
    p_dev->spo2_measurement = (uint32_t)spo2_measurement; 
    p_dev->spo2_valid = (spo2_valid == 1);
    p_dev->hr_measurement = (uint32_t)hr_measurement;
    p_dev->hr_valid = (hr_valid == 1);
    p_dev->ts_last_processing_time = ts;
}

#if SYSTEM_MODE == SYSTEM_MODE_TEST
static void read_int_status1_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("interrupt status 1: %u, %u", (unsigned)(*pb_reg_data), result);
}

static void read_int_status2_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("interrupt status 2: %u, %u", (unsigned)(*pb_reg_data), result);
}

static void read_int_enable1_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("interrupt enable 1: %u, %u", (unsigned)(*pb_reg_data), result);
}

static void read_int_enable2_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("interrupt enable 2: %u, %u", (unsigned)(*pb_reg_data), result);
}

static void read_fifo_write_pointer_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("fifo write pointer: %X, %u", (unsigned)(*pb_reg_data), result);
}

void exercise_max30101_api() {
    ret_code_t err_code;
    NRF_LOG_INFO("exercise_max30101_api");
    
    twi0_init();
    //APP_ERROR_CHECK(err_code);

    static uint8_t reg_int_status1 = 0;
    twi_wait_while_busy(&m_twi);
    err_code = max30101_read_interrupt_status_1(&plx_sensor, read_int_status1_cb, &reg_int_status1);
    NRF_LOG_INFO("max30101_read_interrupt_status_1: %u", err_code);
    err_code = max30101_read_interrupt_status_2(&plx_sensor, read_int_status2_cb, &reg_int_status1);
    static uint8_t reg_int_enable1 = 0;
    twi_wait_while_busy(&m_twi);
    err_code = max30101_read_interrupt_enable_1(&plx_sensor, read_int_enable1_cb, &reg_int_status1);
    NRF_LOG_INFO("max30101_read_interrupt_enable_1: %X", err_code);

    err_code = max30101_write_interrupt_enable_1(&plx_sensor, MAX30101_FIFO_ALMOST_FULL| 
        MAX30101_PPG_RDY|MAX30101_ALC_OF|MAX30101_PWR_RDY);
    NRF_LOG_INFO("max30101_write_interrupt_enable_1: %u", err_code);
    
    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_read_interrupt_enable_2(&plx_sensor, read_int_enable2_cb, &reg_int_status1);
    NRF_LOG_INFO("max30101_read_interrupt_enable_2: %X", err_code);

    err_code = max30101_write_interrupt_enable_2(&plx_sensor, MAX30101_DIE_TEMP_RDY);
    NRF_LOG_INFO("max30101_write_interrupt_enable_2: %u", err_code);

    twi_wait_while_busy(&m_twi);

    err_code = max30101_write_fifo_write_pointer(&plx_sensor, 0x1B);
    NRF_LOG_INFO("max30101_write_fifo_write_pointer: %u", err_code);

    static uint8_t reg8=0;
    err_code = max30101_read_fifo_write_pointer(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_fifo_write_pointer: %u", err_code);

    twi_wait_while_busy(&m_twi);

    err_code = max30101_write_overflow_counter(&plx_sensor, 0x1B);
    NRF_LOG_INFO("max30101_write_overflow_counter: %u", err_code);

    err_code = max30101_read_overflow_counter(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_overflow_counter: %u", err_code);

    twi_wait_while_busy(&m_twi);

    err_code = max30101_write_fifo_read_pointer(&plx_sensor, 0x1B);
    NRF_LOG_INFO("max30101_write_fifo_read_pointer: %u", err_code);

    err_code = max30101_read_fifo_read_pointer(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_fifo_read_pointer: %u", err_code);

    twi_wait_while_busy(&m_twi);
    static uint8_t fifo_sample_data[9] = {9,8,7,6,5,4,3,2,1};
    err_code = max30101_write_fifo_data(&plx_sensor, fifo_sample_data);
    NRF_LOG_INFO("max30101_write_fifo_data: %u", err_code);

    static uint8_t fifo_datum[9];
    err_code = max30101_read_fifo_data(&plx_sensor, read_fifo_write_pointer_cb, fifo_datum);
    NRF_LOG_INFO("max30101_read_fifo_read_pointer: %u", err_code);

    twi_wait_while_busy(&m_twi);

    err_code = max30101_write_fifo_configuration(&plx_sensor, 0x1B);
    NRF_LOG_INFO("max30101_write_fifo_configuration: %u", err_code);

    err_code = max30101_read_fifo_configuration(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_fifo_configuration: %u", err_code);

    twi_wait_while_busy(&m_twi);  
    err_code = max30101_set_fifo_sample_average(&plx_sensor, 0b001);
    NRF_LOG_INFO("max30101_set_fifo_sample_average: %u", err_code);

    twi_wait_while_busy(&m_twi);  
    err_code = max30101_set_fifo_rollover(&plx_sensor, true);
    NRF_LOG_INFO("max30101_set_fifo_rollover: %u", err_code);

    twi_wait_while_busy(&m_twi);  
    err_code = max30101_set_fifo_almost_full_threshold(&plx_sensor, 0b001);
    NRF_LOG_INFO("max30101_set_fifo_almost_full_threshold: %u", err_code);

    twi_wait_while_busy(&m_twi);

    // includes SHDN, RESET, and LED mode control
    err_code = max30101_write_mode_configuration(&plx_sensor, 0x07);
    NRF_LOG_INFO("max30101_write_mode_configuration: %u", err_code);

    err_code = max30101_read_mode_configuration(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_mode_configuration: %u", err_code);

    twi_wait_while_busy(&m_twi);

    // includes SPO2 ADC range, SpO2 sample rate, LED pulse width
    err_code = max30101_write_spo2_configuration(&plx_sensor, 0x62);
    NRF_LOG_INFO("max30101_write_spo2_configuration: %u", err_code);

    err_code = max30101_read_spo2_configuration(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_spo2_configuration: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_spo2_adc_range(&plx_sensor, 3);
    NRF_LOG_INFO("max30101_set_spo2_adc_range: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_spo2_sampling_rate(&plx_sensor, 101);
    NRF_LOG_INFO("max30101_set_spo2_sampling_rate: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_write_led1_pulse_amplitude(&plx_sensor, 0xFF);
    NRF_LOG_INFO("max30101_write_led1_pulse_amplitude: %u", err_code);

    err_code = max30101_read_led1_pulse_amplitude(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led1_pulse_amplitude: %u", err_code);

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_write_led2_pulse_amplitude(&plx_sensor, 0xFF);
    NRF_LOG_INFO("max30101_write_led2_pulse_amplitude: %u", err_code);

    err_code = max30101_read_led2_pulse_amplitude(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led2_pulse_amplitude: %u", err_code)

    twi_wait_while_busy(&m_twi);

    err_code = max30101_write_led3_pulse_amplitude(&plx_sensor, 0xFF);
    NRF_LOG_INFO("max30101_write_led3_pulse_amplitude: %u", err_code);

    err_code = max30101_read_led3_pulse_amplitude(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led3_pulse_amplitude: %u", err_code)

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_write_led4_pulse_amplitude(&plx_sensor, 0xFF);
    NRF_LOG_INFO("max30101_write_led4_pulse_amplitude: %u", err_code);

    err_code = max30101_read_led4_pulse_amplitude(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led4_pulse_amplitude: %u", err_code)    

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_write_led_mode_control12(&plx_sensor, 0x12);
    NRF_LOG_INFO("max30101_write_led_mode_control12: %u", err_code);

    err_code = max30101_read_led_mode_control12(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led_mode_control12: %u", err_code);
    
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_mode_control_slot1(&plx_sensor, 0b100);
    NRF_LOG_INFO("max30101_set_led_mode_control_slot1: %u", err_code);

    
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_mode_control_slot2(&plx_sensor, 0b11);
    NRF_LOG_INFO("max30101_set_led_mode_control_slot2: %u", err_code);

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_write_led_mode_control34(&plx_sensor, 0x34);
    NRF_LOG_INFO("max30101_write_led_mode_control34: %u", err_code);

    err_code = max30101_read_led_mode_control34(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_led_mode_control34: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_mode_control_slot3(&plx_sensor, 0b010);
    NRF_LOG_INFO("max30101_set_led_mode_control_slot3: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_mode_control_slot4(&plx_sensor, 0b001);
    NRF_LOG_INFO("max30101_set_led_mode_control_slot4: %u", err_code);

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_write_die_temp_config(&plx_sensor, 0x01);
    NRF_LOG_INFO("max30101_write_die_temp_config: %u", err_code);
    
    err_code = max30101_read_die_temp_config(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_die_temp_config: %u", err_code);

    twi_wait_while_busy(&m_twi);
    
    err_code = max30101_read_die_temperature_int(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_die_temperature_int: %u", err_code);

    err_code = max30101_read_die_temperature_frac(&plx_sensor, read_fifo_write_pointer_cb, &reg8);
    NRF_LOG_INFO("max30101_read_die_temperature_frac: %u", err_code);

    twi_wait_while_busy(&m_twi);
    uint16_t die_temp = 0;
    err_code = max30101_get_die_temp(&plx_sensor, &die_temp);
    NRF_LOG_INFO("max30101_get_die_temperature: %u, %d", err_code, (int32_t)die_temp);

    twi_wait_while_busy(&m_twi);
    NRF_LOG_INFO("doing soft reset");
    err_code = max30101_reset(&plx_sensor);
    NRF_LOG_INFO("max30101_reset: %u",err_code);

    twi_wait_while_busy(&m_twi);
    NRF_LOG_INFO("power down");
    err_code = max30101_shutdown(&plx_sensor, true);
    NRF_LOG_INFO("max30101_shutdown: %u", err_code);
    
    twi_wait_while_busy(&m_twi);
    NRF_LOG_INFO("power up");
    err_code = max30101_shutdown(&plx_sensor, false);
    NRF_LOG_INFO("max30101_shutdown: %u", err_code);

    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_operation_mode(&plx_sensor, 0b111);
    NRF_LOG_INFO("max30101_set_led_mode: %u", err_code);
///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //no tear down - operations may complete outside the scope of this function   
}

typedef struct plx_sample_s {
    uint32_t r;
    uint32_t ir;
    uint32_t g;
} plx_sample_t;


NRF_QUEUE_DEF(plx_sample_t, plx_sample_queue, 32, NRF_QUEUE_MODE_NO_OVERFLOW);
ret_code_t poll_plx_data(size_t buffer_length, size_t batch_size, uint32_t *red_buffer, uint32_t *ir_buffer, uint32_t *green_buffer) {
    ret_code_t err_code;
    
    //shift data since SpO2 algorithm doesn't know about ring buffers
    for (int i = batch_size; i < buffer_length; i++)  {
        red_buffer[i - batch_size] = red_buffer[i];
        ir_buffer[i - batch_size] = ir_buffer[i];
        green_buffer[i - batch_size] = green_buffer[i];
    }
    
    plx_sample_t plx_sample;
    static uint8_t fifo_read_buffer[9];
    bool samples_available = false;
    for(int i = buffer_length-batch_size; i < buffer_length;) {

        //check for data, start the fetch process if data available
        twi_wait_while_busy(&m_twi);
        err_code = max30101_samples_available(&plx_sensor, &samples_available);
        if(NRF_SUCCESS == err_code && samples_available) {
            //should check return values in case of device error
            twi_wait_while_busy(&m_twi);
            err_code = max30101_read_fifo_data(&plx_sensor, max30101_read_samples_cb, fifo_read_buffer);
            //nrf_delay_ms(1);
            if((err_code) != NRF_SUCCESS) {
                NRF_LOG_INFO("max30101_get_samples: %u", err_code);
            }

            
        } 
        //while(app_sched_queue_space_get() < SCHED_QUEUE_SIZE) {
        //  idle_state_handle();
        //}
        //twi_wait_while_busy(&m_twi);
        
        //pop data off of queue and store
        err_code = nrf_queue_pop(&plx_sample_queue, &plx_sample);
        if(NRF_SUCCESS == err_code) {
            red_buffer[i] = plx_sample.r;
            ir_buffer[i] = plx_sample.ir;
            green_buffer[i] = plx_sample.g;
            //NRF_LOG_INFO("red: %u, ir: %u, green: %u", red_buffer[i],  ir_buffer[i], green_buffer[i]);
            idle_state_handle();
            i++;
        }
    }

    return NRF_SUCCESS;
}

void exercise_spo2_algorithm() {
    ret_code_t err_code;
    twi0_init();
    err_code = max30101_init(&plx_sensor);
    NRF_LOG_INFO("max30101_init: %u", err_code);

    //config pulse oximeter
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_pulse_amplitudes(&plx_sensor, 60, 60, 0, 0);
    NRF_LOG_INFO("max30101_set_led_pulse_amplitudes: %u", err_code);
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_fifo_sample_average(&plx_sensor, MAX30101_SMP_AVE_4);
    NRF_LOG_INFO("max30101_set_fifo_sample_average: %u", err_code);
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_operation_mode(&plx_sensor, MAX30101_OPERATING_MODE_SPO2); //red+IR
    NRF_LOG_INFO("max30101_set_operation_mode: %u", err_code);
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_led_mode_control(&plx_sensor, MAX30101_LED_MODE_CONTROL_RED, MAX30101_LED_MODE_CONTROL_IR, 
                                              MAX30101_LED_MODE_CONTROL_NONE, MAX30101_LED_MODE_CONTROL_NONE);
    NRF_LOG_INFO("max30101_set_led_mode_control: %u", err_code);
    twi_wait_while_busy(&m_twi);
    err_code = max30101_set_spo2_config(&plx_sensor, MAX30101_ADC_RANGE2, MAX30101_SAMPLING_RATE_100, MAX30101_LED_PULSE_WIDTH_411);
    NRF_LOG_INFO("max30101_set_spo2_config: %u", err_code);

    //poll for data+process data
    //probably should put the actual implementation on a timer, or use the interrupt line (almost full signal)
    #define BUFFER_LENGTH (100)
    const uint32_t batch_size = BUFFER_LENGTH/4;
    static uint32_t ir_buffer[BUFFER_LENGTH]; //infrared LED sensor data
    static uint32_t red_buffer[BUFFER_LENGTH];  //red LED sensor data
    static uint32_t green_buffer[BUFFER_LENGTH]; //green LED sensor data

    //prime buffers
    bool samples_available = false;
    while(app_sched_queue_space_get() < SCHED_QUEUE_SIZE) {
        idle_state_handle();
    }
    
    //prime the buffer
    poll_plx_data(BUFFER_LENGTH, BUFFER_LENGTH-batch_size, red_buffer, ir_buffer, green_buffer);
    

    uint32_t spo2 = 0;
    uint8_t spo2_valid = false;
    uint32_t heart_rate = 0;
    uint8_t heart_rate_valid = false;
    
    while(true) {
        maxim_heart_rate_and_oxygen_saturation(ir_buffer, BUFFER_LENGTH, red_buffer, &spo2, &spo2_valid, &heart_rate, &heart_rate_valid);
        //NRF_LOG_INFO("SpO2: %u, %u", spo2, (uint32_t)spo2_valid);
        //NRF_LOG_INFO("Heart Rate: %u, %u", heart_rate, (uint32_t)heart_rate_valid);

        poll_plx_data(BUFFER_LENGTH, batch_size, red_buffer, ir_buffer, green_buffer);
        // idle_state_handle();
        //maxim_heart_rate_and_oxygen_saturation(ir_buffer, buffer_length, red_buffer, &spo2, &spo2_valid, &heart_rate, &heart_rate_valid);
        
        //NRF_LOG_INFO("SpO2: %u, %u", spo2, spo2_valid);
        //NRF_LOG_INFO("Heart Rate: %u, %u", heart_rate, heart_rate_valid);
    }
}
#endif //SYSTEM_MODE_TEST