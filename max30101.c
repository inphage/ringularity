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
#include "sdk_common.h"
#include "nrf_twi_sensor.h"
#include "max30101.h"
#include "nrf_delay.h"
#include "nrf_log.h"

#define FIFO_SIZE (32)

ret_code_t max30101_read_reg(max30101_instance_t *p_instance,
      uint8_t reg_address,
      nrf_twi_sensor_reg_cb_t  user_cb,
      uint8_t *p_data,
      uint8_t  length ) {
    ASSERT(p_instance != NULL);
    ASSERT(p_data != NULL);
    return nrf_twi_sensor_reg_read(p_instance->p_sensor_data,
                                   p_instance->sensor_addr,
                                   reg_address,
                                   user_cb,
                                   p_data,
                                   length);
}

ret_code_t max30101_read_reg8(max30101_instance_t *p_instance,
      uint8_t reg_address,
      nrf_twi_sensor_reg_cb_t  user_cb,
      uint8_t *p_data ) {
    return max30101_read_reg(p_instance,
                             reg_address,
                             user_cb,
                             p_data,
                             1);
}


ret_code_t max30101_write_reg(max30101_instance_t *p_instance, uint8_t reg_address, uint8_t *p_data, uint8_t length) {
    return nrf_twi_sensor_reg_write(p_instance->p_sensor_data, p_instance->sensor_addr, reg_address, p_data, length); 
}

ret_code_t max30101_write_reg8(max30101_instance_t *p_instance, uint8_t reg_address, uint8_t data) {
    return max30101_write_reg(p_instance, reg_address, &data, 1); 
}

// should have some sortof wait override during sensor setup
void sensor_spin_wait(max30101_instance_t *p_instance) {
    while(nrf_drv_twi_is_busy(&p_instance->p_sensor_data->p_twi_mngr->twi)) {
        nrf_delay_ms(1);
    }
}

ret_code_t update_reg8(max30101_instance_t *p_instance, uint8_t reg_address, uint8_t mask, uint8_t position, uint8_t value) {
    uint8_t reg = 0;
    ret_code_t err_code;
    err_code = max30101_read_reg8(p_instance, reg_address, NULL, &reg);
    __WFE(); //wait for op to complete
    NRF_TWI_SENSOR_REG_SET(reg, mask, position, value);
    err_code = max30101_write_reg8(p_instance, reg_address, reg);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE(); //wait for write op to complete

    return NRF_SUCCESS;
}
 
static void max30101_read_part_id_cb(ret_code_t result, void * p_register_data) {
    ASSERT(p_register_data != NULL);
    max30101_instance_t *p_instance = (max30101_instance_t *)p_register_data;
    p_instance->detected = (p_instance->read_data.u8) == MAX30101_PART_ID; 
    NRF_LOG_INFO("max30101 part id: %u, %u", (unsigned)p_instance->read_data.u8, result);
}

ret_code_t max30101_read_part_id(max30101_instance_t *p_instance) {
    return max30101_read_reg8(p_instance, MAX30101_REG_PART_ID, max30101_read_part_id_cb, p_instance->read_data.buff);
}

static void max30101_read_revision_id_cb(ret_code_t result, void * p_register_data) {
    max30101_instance_t *p_instance = (max30101_instance_t *)p_register_data;
    p_instance->rev_id = p_instance->read_data.u8;
    p_instance->initialized = true;
    NRF_LOG_INFO("max30101 revision id: %u, %u", (unsigned)p_instance->read_data.u8, result);
}
 
ret_code_t max30101_read_revision_id(max30101_instance_t *p_instance) {
    return max30101_read_reg8(p_instance, MAX30101_REG_REV_ID, max30101_read_revision_id_cb, p_instance->read_data.buff);
}

ret_code_t max30101_read_interrupt_status_1(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_INT_STATUS1, user_cb, p_data);
}

ret_code_t max30101_read_interrupt_status_2(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_INT_STATUS2, user_cb, p_data);
}

ret_code_t max30101_read_interrupt_enable_1(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_INT_ENABLE1, user_cb, p_data);
}

ret_code_t max30101_write_interrupt_enable_1(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_INT_ENABLE1, data);
}
    
ret_code_t max30101_read_interrupt_enable_2(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_INT_ENABLE2, user_cb, p_data);
}

ret_code_t max30101_write_interrupt_enable_2(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_INT_ENABLE2, data);
}


ret_code_t max30101_read_fifo_write_pointer(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_FIFO_WR_PTR, user_cb, p_data);
}

ret_code_t max30101_write_fifo_write_pointer(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_FIFO_WR_PTR, data);
}

ret_code_t max30101_write_fifo_overflow_counter(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_OVF_COUNTER, data);
}

ret_code_t max30101_read_fifo_overflow_counter(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_OVF_COUNTER, user_cb, p_data);
}

ret_code_t max30101_write_fifo_read_pointer(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_FIFO_RD_PTR, data);
}

ret_code_t max30101_read_fifo_read_pointer(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_FIFO_RD_PTR, user_cb, p_data);
}

//gets the number of samples ready for reading in the fifo buffer
ret_code_t max30101_samples_available(max30101_instance_t *p_instance, uint32_t *num_samples_available) {
    static uint8_t fifo_write_ptr = 0, fifo_read_ptr = 0, fifo_overflow_counter=0;
    ret_code_t err_code;
    err_code = max30101_read_fifo_write_pointer(p_instance, NULL, &fifo_write_ptr);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    //sensor_spin_wait(p_instance);
    
    err_code = max30101_read_fifo_overflow_counter(p_instance, NULL, &fifo_overflow_counter);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();

    err_code = max30101_read_fifo_read_pointer(p_instance, NULL, &fifo_read_ptr);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_instance);
    
    //if(fifo_write_ptr != fifo_read_ptr) {
    //    NRF_LOG_INFO("write ptr: %u, read ptr: %u", (uint32_t)fifo_write_ptr, (uint32_t)fifo_read_ptr);
    //}
    if(fifo_overflow_counter == 0) {
        *num_samples_available = fifo_write_ptr - fifo_read_ptr;
        if(fifo_write_ptr < fifo_read_ptr) {
          *num_samples_available += FIFO_SIZE;
        }
    } else {
        *num_samples_available = FIFO_SIZE;
    }

    return NRF_SUCCESS;
}

ret_code_t max30101_write_fifo_data(max30101_instance_t *p_instance, uint8_t data[]) {
   return max30101_write_reg(p_instance, MAX30101_REG_FIFO_DATA, data, 9);
}

ret_code_t max30101_read_fifo_data(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    if((p_instance->op_mode) == MAX30101_OPERATING_MODE_HR) {
        return max30101_read_reg(p_instance, MAX30101_REG_FIFO_DATA, user_cb, p_data, 3);
    } else if ((p_instance->op_mode) == MAX30101_OPERATING_MODE_SPO2) {
        return max30101_read_reg(p_instance, MAX30101_REG_FIFO_DATA, user_cb, p_data, 6);
    } else {
        return max30101_read_reg(p_instance, MAX30101_REG_FIFO_DATA, user_cb, p_data, 9);
    }

    return NRF_ERROR_FORBIDDEN;
}

static void max30101_get_samples_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("fifo write pointer: %X", result); 
    NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[0], pb_reg_data[1], pb_reg_data[2]);
    NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[3], pb_reg_data[4], pb_reg_data[5]);
    NRF_LOG_INFO("\t%02X%02X%02X", pb_reg_data[6], pb_reg_data[7], pb_reg_data[8]);
}


ret_code_t max30101_get_samples(max30101_instance_t *p_instance, uint32_t *p_red, uint32_t *p_ir, uint32_t *p_green) {
    ret_code_t err_code;
    static uint8_t samples[9];

    err_code = max30101_read_fifo_data(p_instance, max30101_get_samples_cb, samples);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }

    __WFE();
    sensor_spin_wait(p_instance);

    if(p_instance->op_mode & 0b010) {
      *p_red = ((uint32_t)samples[0]) << 16 + ((uint32_t)samples[1]) << 8 + samples[2];
     } else {
      *p_red = 0;
     }
    if(p_instance->op_mode & 0b001) {
      *p_ir = ((uint32_t)samples[3]) << 16 + ((uint32_t)samples[4]) << 8 + samples[5];
    } else {
      *p_ir = 0;
    }
    if(p_instance->op_mode & 0b100) {
      *p_green = ((uint32_t)samples[6]) << 16 + ((uint32_t)samples[7]) << 8 + samples[8];
    } else {
      *p_green;
    }
    return NRF_SUCCESS;
}

ret_code_t max30101_write_fifo_configuration(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_FIFO_CONF, data);
}

ret_code_t max30101_read_fifo_configuration(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_FIFO_CONF, user_cb, p_data);
}

ret_code_t max30101_set_fifo_sample_average(max30101_instance_t *p_instance, uint8_t data) {
    return update_reg8(p_instance, MAX30101_REG_FIFO_CONF, FIFO_CONF_SMP_AVE_MASK, FIFO_CONF_SMP_AVE_POS, data);
}

ret_code_t max30101_set_fifo_rollover(max30101_instance_t *p_instance, bool enable) {
    return update_reg8(p_instance, MAX30101_REG_FIFO_CONF, FIFO_CONF_FIFO_ROLLOVER_EN_MASK, FIFO_CONF_FIFO_ROLLOVER_EN_POS, 
        enable);
}

ret_code_t max30101_set_fifo_almost_full_threshold(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_FIFO_CONF, FIFO_CONF_FIFO_A_FULL_MASK, FIFO_CONF_FIFO_A_FULL_POS, value);
}

ret_code_t max30101_write_mode_configuration(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_MODE_CONF, data);
}

ret_code_t max30101_read_mode_configuration(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_MODE_CONF, user_cb, p_data);
}

ret_code_t max30101_reset(max30101_instance_t *p_instance) {
    ret_code_t err_code;
    err_code = update_reg8(p_instance, MAX30101_REG_MODE_CONF, MODE_RESET_MASK, MODE_RESET_POS, 1);
    NRF_LOG_INFO("max30101_reset - update_reg8: %u", err_code);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_instance);

    //wait for reset to complete
    uint8_t reg;
    do {
        err_code = max30101_read_mode_configuration(p_instance, NULL, &reg);
        NRF_LOG_INFO("max30101_reset - read conf: %u", err_code);
        if((err_code) != NRF_SUCCESS) {
            return err_code;
        }
        __WFE(); //wait for read op to complete
        sensor_spin_wait(p_instance);
    } while(reg != 0);

    return NRF_SUCCESS;
}

// down = 1 -> enter low power state
// down = 0 -> exit low power state
ret_code_t max30101_shutdown(max30101_instance_t *p_instance, bool down) {
    return update_reg8(p_instance, MAX30101_REG_MODE_CONF, MODE_SHDN_MASK, MODE_SHDN_POS, down);
}

ret_code_t max30101_set_operation_mode(max30101_instance_t *p_instance, uint8_t data) {
    p_instance->op_mode = data;
    return update_reg8(p_instance, MAX30101_REG_MODE_CONF, OPERATION_MODE_MASK, OPERATION_MODE_POS, data);
}

ret_code_t max30101_write_spo2_configuration(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_SPO2_CONF, data);
}

ret_code_t max30101_read_spo2_configuration(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_SPO2_CONF, user_cb, p_data);
}

ret_code_t max30101_set_spo2_adc_range(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_SPO2_CONF, SPO2_CONF_ADC_RANGE_MASK, SPO2_CONF_ADC_RANGE_POS, value);
}

ret_code_t max30101_set_spo2_sampling_rate(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_SPO2_CONF, SPO2_CONF_SR_MASK, SPO2_CONF_SR_POS, value);
}

ret_code_t max30101_set_spo2_pulse_width(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_SPO2_CONF, SPO2_CONF_LED_PW_MASK, SPO2_CONF_LED_PW_POS, value);
}

ret_code_t max30101_set_spo2_config(max30101_instance_t *p_instance, uint8_t adc_range, uint8_t spo2_sr, uint8_t led_pw) {
    ret_code_t err_code;
    uint8_t reg = 0;

    //TODO: scrub inputs.  There are mutually exclusive values for inputs.
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_CONF_ADC_RANGE_MASK, SPO2_CONF_ADC_RANGE_POS, adc_range);
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_CONF_SR_MASK, SPO2_CONF_SR_POS, adc_range);
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_CONF_LED_PW_MASK, SPO2_CONF_LED_PW_POS, adc_range);
    err_code = max30101_write_spo2_configuration(p_instance, reg);
    //if((err_code) == NRF_SUCCESS) {
    //    __WFE();
    //}
    return err_code;
}

// red
ret_code_t max30101_write_led1_pulse_amplitude(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE1, data);
}

// red
ret_code_t max30101_read_led1_pulse_amplitude(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE1, user_cb, p_data);
}

// IR
ret_code_t max30101_write_led2_pulse_amplitude(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE2, data);
}

// IR
ret_code_t max30101_read_led2_pulse_amplitude(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE2, user_cb, p_data);
}

// green
ret_code_t max30101_write_led3_pulse_amplitude(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE3, data);
}

// green
ret_code_t max30101_read_led3_pulse_amplitude(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE3, user_cb, p_data);
}

// another green
ret_code_t max30101_write_led4_pulse_amplitude(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE4, data);
}

// also green
ret_code_t max30101_read_led4_pulse_amplitude(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_PULSE_AMPLITUDE4, user_cb, p_data);
}

ret_code_t max30101_set_led_pulse_amplitudes(max30101_instance_t *p_instance, uint8_t red, uint8_t ir, uint8_t green1, uint8_t green2) {
    ret_code_t err_code;
    
    err_code = max30101_write_led1_pulse_amplitude(p_instance, red);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    err_code = max30101_write_led2_pulse_amplitude(p_instance, ir);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    err_code = max30101_write_led3_pulse_amplitude(p_instance, green1);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    err_code = max30101_write_led4_pulse_amplitude(p_instance, green2);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();

    return err_code;
} 

ret_code_t max30101_write_led_mode_control12(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL12, data);
}


ret_code_t max30101_read_led_mode_control12(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL12, user_cb, p_data);
}

ret_code_t max30101_set_led_mode_control_slot1(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL12, SPO2_LED_MODE_SLOT1_MASK, SPO2_LED_MODE_SLOT1_POS, value);
}

ret_code_t max30101_set_led_mode_control_slot2(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL12, SPO2_LED_MODE_SLOT2_MASK, SPO2_LED_MODE_SLOT2_POS, value);
}

ret_code_t max30101_write_led_mode_control34(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL34, data);
}

ret_code_t max30101_read_led_mode_control34(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL34, user_cb, p_data);
}

ret_code_t max30101_set_led_mode_control(max30101_instance_t *p_instance, uint8_t slot1, uint8_t slot2, 
    uint8_t slot3, uint8_t slot4) {
    uint8_t reg = 0;
    ret_code_t err_code;
    
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_LED_MODE_SLOT1_MASK, SPO2_LED_MODE_SLOT1_POS, slot1);
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_LED_MODE_SLOT2_MASK, SPO2_LED_MODE_SLOT2_POS, slot2);
    err_code = max30101_write_led_mode_control12(p_instance, reg);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();

    NRF_TWI_SENSOR_REG_SET(reg, SPO2_LED_MODE_SLOT3_MASK, SPO2_LED_MODE_SLOT3_POS, slot3);
    NRF_TWI_SENSOR_REG_SET(reg, SPO2_LED_MODE_SLOT4_MASK, SPO2_LED_MODE_SLOT4_POS, slot4);
    err_code = max30101_write_led_mode_control34(p_instance, reg);
    __WFE();

    return err_code;
} 

ret_code_t max30101_set_led_mode_control_slot3(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL34, SPO2_LED_MODE_SLOT3_MASK, SPO2_LED_MODE_SLOT3_POS, value);
}

ret_code_t max30101_set_led_mode_control_slot4(max30101_instance_t *p_instance, uint8_t value) {
    return update_reg8(p_instance, MAX30101_REG_LED_MODE_CTRL34, SPO2_LED_MODE_SLOT4_MASK, SPO2_LED_MODE_SLOT4_POS, value);
}

//read die temp
//
ret_code_t max30101_read_die_temperature_int(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_DIE_TEMP_INT, user_cb, p_data);
}

ret_code_t max30101_read_die_temperature_frac(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_DIE_TEMP_FRAC, user_cb, p_data);
}

ret_code_t max30101_write_die_temp_config(max30101_instance_t *p_instance, uint8_t data) {
   return max30101_write_reg8(p_instance, MAX30101_REG_DIE_TEMP_CONF, data);
}

ret_code_t max30101_read_die_temp_config(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data) {
    return max30101_read_reg8(p_instance, MAX30101_REG_DIE_TEMP_CONF, user_cb, p_data);
}

static void read_die_temp_cb(ret_code_t result, void * p_register_data) {
    uint8_t *pb_reg_data = (uint8_t *)p_register_data;
    NRF_LOG_INFO("die temp: %d, %p, %u", (int)(*pb_reg_data), p_register_data, result);
}

//temperature is in increments of 0.0625 C
ret_code_t max30101_get_die_temp(max30101_instance_t *p_instance, int16_t *p_temp) {
    ret_code_t err_code;
    static int16_t temp_frac = 0;

    // initiate temperature collection
    err_code = max30101_write_die_temp_config(p_instance, 0x01);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    

    //wait until temperature reading is ready
    uint8_t reg = 1;
    while((reg) != 0) {
        nrf_delay_ms(1);
        err_code = max30101_read_die_temp_config(p_instance, NULL, &reg);
        if((err_code) != NRF_SUCCESS) {
            return err_code;
        }
        __WFE();
        sensor_spin_wait(p_instance);
    }

    sensor_spin_wait(p_instance);
    
    static int16_t temp_int = 0;
    (err_code) = max30101_read_die_temperature_int(p_instance, NULL, (uint8_t *)&temp_int);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_instance);

    //NRF_LOG_INFO("int %d, %p", temp_int, &temp_int);

    err_code = max30101_read_die_temperature_frac(p_instance, NULL, (uint8_t *)&temp_frac);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_instance);
    
    //NRF_LOG_INFO("frac %d, %p", temp_frac, &temp_frac);
    *p_temp = (temp_int << 4) + temp_frac;

    return NRF_SUCCESS;
}

ret_code_t max30101_init(max30101_instance_t *p_instance, max30101_init_params_t *p_params) {
    ret_code_t err_code;

    *p_instance = (max30101_instance_t){
      .read_data.buff = {0},
      .p_sensor_data = p_params->p_sensor_data,
      .sensor_addr = p_params->sensor_addr,
      .detected = false,
      .initialized = false
    }; 
    //p_instance->p_sensor_data = NULL; //p_params->p_sensor_data;
    //p_instance->sensor_addr = p_params->sensor_addr;

    sensor_spin_wait(p_instance); //in case another twi operations is in progress 

    //put device into a known state
    err_code = max30101_reset(p_instance);
    NRF_LOG_INFO("max30101_reset: %u", err_code);
    if((err_code) != NRF_SUCCESS) {
        return err_code;
    }
    __WFE();
    sensor_spin_wait(p_instance);

    //check part id - fail if unexpected value
    uint8_t partid = 0;
    err_code = max30101_read_part_id(p_instance);
    NRF_LOG_INFO("max30101_read_part_id: %u", err_code);
    ret_code_t ret_code = err_code;
    __WFE();

    //read revision id
    if(NRF_SUCCESS == err_code) {
        err_code = max30101_read_revision_id(p_instance);
        NRF_LOG_INFO("max30101_read_revision: %u", err_code);
        ret_code = ((err_code) == NRF_SUCCESS) ? err_code : NRF_ERROR_NOT_FOUND;
    }

    sensor_spin_wait(p_instance);
    //wait for ops to complete
    //while(true) {
      //CRITICAL_REGION_ENTER();
   //   if((p_instance->initialized) == false) {
   //       break;
   //   }
      //CRITICAL_REGION_EXIT();
      //nrf_delay_ms(1);
    //}

    return ret_code;
}
