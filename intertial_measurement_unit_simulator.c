#include "hw_config.h"
#include "sensorsim.h"
#include "intertial_measurement_unit_simulator.h"

typedef struct device_inertial_measurement_unit_sim_s {
    device_inertial_measurement_unit_t ext_iface;
    sensorsim_cfg_t   speed_mps_sim_cfg;                       
    sensorsim_state_t speed_mps_sim_state;                     
    sensorsim_cfg_t   cadence_rpm_sim_cfg;                     
    sensorsim_state_t cadence_rpm_sim_state;                   
    sensorsim_cfg_t   cadence_stl_sim_cfg;                     
    sensorsim_state_t cadence_stl_sim_state;                   
} device_inertial_measurement_unit_sim_t;


static ret_code_t imu_read_speed(device_inertial_measurement_unit_t *p_dev, uint16_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_inertial_measurement_unit_sim_t *p_sim = (device_inertial_measurement_unit_sim_t *)p_dev;
    *p_measurement = sensorsim_measure(&p_sim->speed_mps_sim_state,
                                      &p_sim->speed_mps_sim_cfg);
}

static ret_code_t imu_read_cadence(device_inertial_measurement_unit_t *p_dev, uint8_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_inertial_measurement_unit_sim_t *p_sim = (device_inertial_measurement_unit_sim_t *)p_dev;
    *p_measurement = sensorsim_measure(&p_sim->cadence_rpm_sim_state,
                                                    &p_sim->cadence_rpm_sim_cfg);
}

static ret_code_t imu_read_stride_length(device_inertial_measurement_unit_t *p_dev, uint16_t *p_measurement) {
    DEV_READ_CHECK_PARAMS(p_dev, p_measurement);
    device_inertial_measurement_unit_sim_t *p_sim = (device_inertial_measurement_unit_sim_t *)p_dev;
    *p_measurement = sensorsim_measure(&p_sim->cadence_stl_sim_state,
                                                          &p_sim->cadence_stl_sim_cfg);
}

ret_code_t device$inertial_measurement_unit_sim$open(uint32_t device_id, 
    device_inertial_measurement_unit_sim_params_t *params, device_inertial_measurement_unit_t **pp_dev) {
    static device_inertial_measurement_unit_sim_t sims[INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT];
    if(device_id >= INERTIAL_MEASUREMENT_UNIT_SIMULATOR_COUNT) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if((pp_dev) == NULL) {
        return NRF_ERROR_NULL;
    }

    // speed is in units of meters per second divided by 256
    device_inertial_measurement_unit_sim_t *p_sim = &sims[device_id];
 
    p_sim->speed_mps_sim_cfg.min          = (uint32_t)(MIN_SPEED_MPS * 256);
    p_sim->speed_mps_sim_cfg.max          = (uint32_t)(MAX_SPEED_MPS * 256);
    p_sim->speed_mps_sim_cfg.incr         = (uint32_t)(SPEED_MPS_INCREMENT * 256);
    p_sim->speed_mps_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->speed_mps_sim_state, &p_sim->speed_mps_sim_cfg);

    p_sim->cadence_rpm_sim_cfg.min          = MIN_CADENCE_RPM;
    p_sim->cadence_rpm_sim_cfg.max          = MAX_CADENCE_RPM;
    p_sim->cadence_rpm_sim_cfg.incr         = CADENCE_RPM_INCREMENT;
    p_sim->cadence_rpm_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->cadence_rpm_sim_state, &p_sim->cadence_rpm_sim_cfg);

    p_sim->cadence_stl_sim_cfg.min          = MIN_STRIDE_LENGTH;
    p_sim->cadence_stl_sim_cfg.max          = MAX_STRIDE_LENGTH;
    p_sim->cadence_stl_sim_cfg.incr         = STRIDE_LENGTH_INCREMENT;
    p_sim->cadence_stl_sim_cfg.start_at_max = false;

    sensorsim_init(&p_sim->cadence_stl_sim_state, &p_sim->cadence_stl_sim_cfg);
    p_sim->ext_iface.read_speed = imu_read_speed;
    p_sim->ext_iface.read_cadence = imu_read_cadence;
    p_sim->ext_iface.read_stride_length = imu_read_stride_length;
    p_sim->ext_iface.initialized = true;
    *pp_dev = (device_inertial_measurement_unit_t *)p_sim;

    return NRF_SUCCESS;
}
