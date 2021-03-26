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
#ifndef __MAX30101_H__
#define __MAX30101_H__

typedef enum {
    //MAX30101_WRITE_ADDRESS = 0xAEu, //MAX30101 documentation
    MAX30101_WRITE_ADDRESS = 0x57u, //tinycircuits pulse ox sensor AST1041
    //MAX30101_READ_ADDRESS = 0xAFu //MAX30101 documentation
    MAX30101_READ_ADDRESS = 0x58u //tinycircuits pulse ox sensor AST1041
} max30101_address_t;

typedef enum {
    MAX30101_REG_INT_STATUS1 = 0X00, //R
    MAX30101_REG_INT_STATUS2 = 0X01, //R
    MAX30101_REG_INT_ENABLE1 = 0X02,  //RW
    MAX30101_REG_INT_ENABLE2 = 0X03,  //RW
    MAX30101_REG_FIFO_WR_PTR = 0X04,  //RW
    MAX30101_REG_OVF_COUNTER = 0X05,  //RW
    MAX30101_REG_FIFO_RD_PTR = 0X06,  //RW
    MAX30101_REG_FIFO_DATA = 0X07,  //RW
    MAX30101_REG_FIFO_CONF = 0X08,  //RW
    MAX30101_REG_MODE_CONF = 0X09,  //RW
    MAX30101_REG_SPO2_CONF = 0X0A,  //RW
    //0X0B RESERVED  RW
    MAX30101_REG_LED_PULSE_AMPLITUDE1 = 0X0C,  //RW
    MAX30101_REG_LED_PULSE_AMPLITUDE2 = 0X0D,  //RW
    MAX30101_REG_LED_PULSE_AMPLITUDE3 = 0X0E,  //RW
    MAX30101_REG_LED_PULSE_AMPLITUDE4 = 0X0F,  //RW
    //0X10??? not documentesd
    MAX30101_REG_LED_MODE_CTRL12 = 0X11,  //RW
    MAX30101_REG_LED_MODE_CTRL34 = 0X12,  //RW
    //0X13-0X17 RESERVED  RW
    //0X18-0X1E RESERVED  R
    MAX30101_REG_DIE_TEMP_INT = 0X1F,   //R
    MAX30101_REG_DIE_TEMP_FRAC = 0X20,  //R
    MAX30101_REG_DIE_TEMP_CONF = 0X21,   //RW
    //0X22-0X2F RESERVED //RW
    MAX30101_REG_REV_ID = 0XFE,  //R
    MAX30101_REG_PART_ID = 0XFF  //R
} max30101_reg_address_t;

#define MAX30101_PART_ID (0X15)

// register flags/fields
#define MAX30101_FIFO_ALMOST_FULL (0x80)
#define MAX30101_PPG_RDY          (0x40)
#define MAX30101_ALC_OF           (0x20)
#define MAX30101_PWR_RDY          (0x01)
#define MAX30101_DIE_TEMP_RDY     (0X02)
#define FIFO_CONF_SMP_AVE_MASK    (0xE0)
#define FIFO_CONF_SMP_AVE_POS     (5)
#define FIFO_CONF_FIFO_ROLLOVER_EN_MASK (0X10)
#define FIFO_CONF_FIFO_ROLLOVER_EN_POS  (4)
#define FIFO_CONF_FIFO_A_FULL_MASK (0X0F)
#define FIFO_CONF_FIFO_A_FULL_POS  (0)
#define MODE_SHDN_MASK            (0X80)
#define MODE_SHDN_POS             (8)
#define MODE_RESET_MASK           (0X40)
#define MODE_RESET_POS            (7)
#define OPERATION_MODE_MASK       (0X07)
#define OPERATION_MODE_POS        (0)
#define SPO2_CONF_ADC_RANGE_MASK  (0x60)
#define SPO2_CONF_ADC_RANGE_POS   (5)
#define SPO2_CONF_SR_MASK         (0X16)
#define SPO2_CONF_SR_POS          (2)
#define SPO2_CONF_LED_PW_MASK     (0X03)
#define SPO2_CONF_LED_PW_POS      (0)
#define SPO2_LED_MODE_SLOT1_MASK  (0x03)
#define SPO2_LED_MODE_SLOT1_POS   (0)
#define SPO2_LED_MODE_SLOT2_MASK  (0x30)
#define SPO2_LED_MODE_SLOT2_POS   (4)
#define SPO2_LED_MODE_SLOT3_MASK  (0x03)
#define SPO2_LED_MODE_SLOT3_POS   (0)
#define SPO2_LED_MODE_SLOT4_MASK  (0x30)
#define SPO2_LED_MODE_SLOT4_POS   (4)

#define MAX30101_OPERATING_MODE_HR      (0b010) //heart rate mode: red
#define MAX30101_OPERATING_MODE_SPO2    (0b011) //SpO2 mode: red+IR
#define MAX30101_OPERATING_MODE_MULTI   (0b111) //Multi: red+IR+green

#define MAX30101_LED_MODE_CONTROL_RED      (0b001)
#define MAX30101_LED_MODE_CONTROL_IR       (0b010)
#define MAX30101_LED_MODE_CONTROL_GREEN    (0b011)
#define MAX30101_LED_MODE_CONTROL_NONE     (0b100)

#define MAX30101_ADC_RANGE1                (0b00)  //LSB size 7.81 pA, full scale 2048 nA
#define MAX30101_ADC_RANGE2                (0b01)  //LSB size 7.81 pA, full scale 4096 nA
#define MAX30101_ADC_RANGE3                (0b10)  //LSB size 7.81 pA, full scale 8192 nA
#define MAX30101_ADC_RANGE4                (0b11)  //LSB size 7.81 pA, full scale 16384 nA

#define MAX30101_SAMPLING_RATE_50          (0b000)
#define MAX30101_SAMPLING_RATE_100         (0b001)
#define MAX30101_SAMPLING_RATE_200         (0b010)
#define MAX30101_SAMPLING_RATE_400         (0b011)
#define MAX30101_SAMPLING_RATE_800         (0b100)
#define MAX30101_SAMPLING_RATE_1000        (0b101)
#define MAX30101_SAMPLING_RATE_1600        (0b110)
#define MAX30101_SAMPLING_RATE_3200        (0b111)

#define MAX30101_LED_PULSE_WIDTH_69        (0b00) //69 uS, 15-bit ADC
#define MAX30101_LED_PULSE_WIDTH_118       (0b01) //118 uS, 16-bit ADC
#define MAX30101_LED_PULSE_WIDTH_215       (0b10) //215 uS, 17-bit ADC
#define MAX30101_LED_PULSE_WIDTH_411       (0b11) //411 uS, 18-bit ADC 

#define MAX30101_SMP_AVE_NONE              (0b000) //1 sample/no averaging
#define MAX30101_SMP_AVE_2                 (0b001)                    
#define MAX30101_SMP_AVE_4                 (0b010)
#define MAX30101_SMP_AVE_8                 (0b011)
#define MAX30101_SMP_AVE_16                (0b100)
#define MAX30101_SMP_AVE_32                (0b101)

typedef struct max30101_instance_s {
    union {
        uint8_t buff[16];
        uint8_t u8; 
    } read_data;
    nrf_twi_sensor_t * p_sensor_data;
    uint8_t            sensor_addr;
    bool detected;
    bool initialized;
    uint8_t rev_id;
    uint32_t op_mode;
    uint32_t red_sample;
    uint32_t ir_sample;
    uint32_t green_sample;
} max30101_instance_t;

typedef struct max30101_init_params_s {
    nrf_twi_sensor_t * p_sensor_data;
    uint8_t const            sensor_addr;
} max30101_init_params_t;

ret_code_t max30101_init(max30101_instance_t *p_instance, max30101_init_params_t *p_params);

ret_code_t max30101_reset(max30101_instance_t *p_instance);

// down = 1 -> enter low power state
// down = 0 -> exit low power state
ret_code_t max30101_shutdown(max30101_instance_t *p_instance, bool down);

ret_code_t max30101_set_led_pulse_amplitudes(max30101_instance_t *p_instance, uint8_t red, uint8_t ir, uint8_t green1, uint8_t green2);

ret_code_t max30101_set_fifo_sample_average(max30101_instance_t *p_instance, uint8_t data);

ret_code_t max30101_set_operation_mode(max30101_instance_t *p_instance, uint8_t data);

ret_code_t max30101_set_led_mode_control(max30101_instance_t *p_instance, uint8_t slot1, uint8_t slot2, 
    uint8_t slot3, uint8_t slot4);

ret_code_t max30101_set_spo2_config(max30101_instance_t *p_instance, uint8_t adc_range, uint8_t spo2_sr, uint8_t led_pw);

ret_code_t max30101_samples_available(max30101_instance_t *p_instance, uint32_t *num_samples_available);

ret_code_t max30101_read_fifo_data(max30101_instance_t *p_instance, 
    nrf_twi_sensor_reg_cb_t  user_cb, uint8_t *p_data);

#endif //__MAX30101_H__