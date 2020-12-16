#ifndef __HEART_RATE_SENSOR__
#define __HEART_RATE_SENSOR__

#include <stdbool.h>

typedef struct device_heart_rate_sensor_s {
    bool initialized;
    bool contact_detected;
    bool rr_interval_enabled;
    ret_code_t (*read_heart_rate)(struct device_heart_rate_sensor_s *p_dev, uint16_t *p_measurement);
    ret_code_t (*read_rr_interval)(struct device_heart_rate_sensor_s *p_dev, uint16_t *p_measurement);
} device_heart_rate_sensor_t;

#define DECL_DEV_HEART_RATE_SENSOR(x) extern device_heart_rate_sensor_t *dev_heart_rate_sensor ## x;
#define DEF_DEV_HEART_RATE_SENSOR(x) device_heart_rate_sensor_t *dev_heart_rate_sensor ## x;

#endif //__HEART_RATE_SENSOR__