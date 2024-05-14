/******************************************************************************
 * \file mtb_bmi270.c
 *
 * \brief
 *     This file contains the functions for interacting with the
 *     BMI270 motion sensor.
 *
 ********************************************************************************
 * \copyright
 * Copyright 2024 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "mtb_bmi270.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/******************************************************************************
* Macros
******************************************************************************/
#define _I2C_TIMEOUT_MS            (1U)
#define _READ_WRITE_LEN            (46U)
#define _SOFT_RESET_DELAY_US       300

/******************************************************************************
* Global variables
******************************************************************************/
static cyhal_i2c_t* _bmi270_i2c = NULL;
static uint8_t _dev_addr;

/*****************************************************************************
* Local function Prototypes
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(uint8_t reg_addr, uint8_t* reg_data,
                                            uint32_t len, void* intf_ptr);

static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(uint8_t reg_addr, const uint8_t* reg_data,
                                             uint32_t len, void* intf_ptr);

static void _bmi2_delay_us(uint32_t us, void* intf_ptr);

/******************************************************************************
* _mtb_bmi270_pins_equal
******************************************************************************/
static inline bool _mtb_bmi270_pins_equal(_mtb_bmi270_interrupt_pin_t ref_pin, cyhal_gpio_t pin)
{
    return (ref_pin.pin == pin);
}


/******************************************************************************
* mtb_bmi270_init_i2c
******************************************************************************/
cy_rslt_t mtb_bmi270_init_i2c(mtb_bmi270_t* dev, cyhal_i2c_t* i2c_instance,
                              mtb_bmi270_address_t address)
{
    cy_rslt_t rslt;
    _dev_addr = address;
    struct bmi2_sens_config config = {0};

    CY_ASSERT(NULL != i2c_instance);
    CY_ASSERT(NULL != dev);

    _bmi270_i2c = i2c_instance;

    dev->sensor.intf = BMI2_I2C_INTF;
    dev->sensor.read = _bmi2_i2c_read;
    dev->sensor.write = _bmi2_i2c_write;
    dev->sensor.delay_us = _bmi2_delay_us;
    dev->sensor.intf_ptr = &_dev_addr;
    dev->sensor.read_write_len = _READ_WRITE_LEN;
    dev->sensor.config_file_ptr = NULL;

    bmi270_init(&(dev->sensor));

    rslt = bmi270_get_sensor_config(&config, 1, &(dev->sensor));

    return (BMI2_OK == rslt)
            ? CY_RSLT_SUCCESS
            : rslt;
}


/******************************************************************************
* mtb_bmi270_config_default
******************************************************************************/
cy_rslt_t mtb_bmi270_config_default(mtb_bmi270_t* dev)
{
    cy_rslt_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    struct bmi2_sens_config config = {0};

    rslt = bmi2_get_sensor_config(&config, 1, &(dev->sensor));
    if (BMI2_OK != rslt)
    {
        return rslt;
    }
    config.type = BMI2_ACCEL;
    config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config.cfg.acc.range = BMI2_ACC_RANGE_2G;
    config.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    rslt = bmi2_set_sensor_config(&config, 1, &(dev->sensor));
    if (BMI2_OK != rslt)
    {
        return rslt;
    }

    rslt = bmi2_get_sensor_config(&config, 1, &(dev->sensor));
    if (BMI2_OK != rslt)
    {
        return rslt;
    }
    config.type = BMI2_GYRO;
    config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    rslt = bmi2_set_sensor_config(&config, 1, &(dev->sensor));
    if (BMI2_OK != rslt)
    {
        return rslt;
    }

    rslt = bmi2_get_sensor_config(&config, 1, &(dev->sensor));
    if (BMI2_OK != rslt)
    {
        return rslt;
    }

    rslt = bmi2_sensor_enable(sens_list, 2, &(dev->sensor));
    return (BMI2_OK == rslt)
            ? CY_RSLT_SUCCESS
            : rslt;
}


/******************************************************************************
* mtb_bmi270_read
******************************************************************************/
cy_rslt_t mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)
{
    cy_rslt_t rslt;
    rslt = bmi2_get_sensor_data(&(data->sensor_data), &(dev->sensor));
    return (BMI2_OK == rslt)
            ? CY_RSLT_SUCCESS
            : rslt;
}


/******************************************************************************
* mtb_bmi270_selftest
******************************************************************************/
cy_rslt_t mtb_bmi270_selftest(mtb_bmi270_t* dev)
{
    cy_rslt_t rslt = bmi2_perform_accel_self_test(&(dev->sensor));
    cyhal_system_delay_us(_SOFT_RESET_DELAY_US); //per datasheet, delay needed after reset to reboot

    if (BMI2_OK == rslt)
    {
        rslt =  bmi2_do_gyro_st(&(dev->sensor));
        cyhal_system_delay_us(_SOFT_RESET_DELAY_US); // delay needed after another reset
    }
    return (BMI2_OK == rslt)
            ? CY_RSLT_SUCCESS
            : rslt;
}


/******************************************************************************
* mtb_bmi270_free_pin
******************************************************************************/
void mtb_bmi270_free_pin(mtb_bmi270_t* dev)
{
    if (!_mtb_bmi270_pins_equal(dev->intpin1, NC))
    {
        cyhal_gpio_free((dev->intpin1).pin);
    }

    if (!_mtb_bmi270_pins_equal(dev->intpin2, NC))
    {
        cyhal_gpio_free((dev->intpin2).pin);
    }

    _bmi270_i2c = NULL;
}


/*****************************************************************************
* Function name: _bmi2_i2c_read
*****************************************************************************
* Summary:
* This internal function reads I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(uint8_t reg_addr, uint8_t* reg_data,
                                            uint32_t len, void* intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_read(_bmi270_i2c, device_addr,
                                                            reg_addr, 1,
                                                            reg_data, (uint16_t)len,
                                                            _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmi2_i2c_write
*****************************************************************************
* Summary:
* This internal function writes I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(uint8_t reg_addr, const uint8_t* reg_data,
                                             uint32_t len, void* intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_write(_bmi270_i2c, device_addr,
                                                             reg_addr, 1,
                                                             reg_data,
                                                             (uint16_t)len,
                                                             _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmi2_delay_us
*****************************************************************************
* Summary:
* This internal function maps delay function to host MCU
*
* Parameters:
*  us    The time period in microseconds
*  intf_ptr  Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  void
*
*****************************************************************************/
static void _bmi2_delay_us(uint32_t us, void* intf_ptr)
{
    (void)intf_ptr;

    cyhal_system_delay_us(us);
}


#if defined(__cplusplus)
}
#endif

/* [] END OF FILE */
