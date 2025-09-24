/*******************************************************************************
* \file mtb_bmi270.h
*
* \brief
*     This file is the public interface of the BMI270 motion sensor.
*
********************************************************************************
* \copyright
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company)
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

#pragma once

#include "bmi270.h"
#include "cy_result.h"

#include "mtb_hal.h"
#include "mtb_hal_gpio.h"
#include "mtb_hal_hw_types.h"
#include "mtb_hal_system.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Structure holding the IMU instance specific information.
 *
 * Application code should not rely on the specific content of this struct.
 * They are considered an implementation detail which is subject to change
 * between platforms and/or library releases.
 */
typedef struct
{
    struct bmi2_dev sensor;
} mtb_bmi270_t;

/** Structure holding the type of the sensor and its configurations. */
typedef struct
{
    struct bmi2_sens_config sensor_config;
} mtb_bmi270_sens_config_t;

/** Structure holding the accelerometer and gyroscope data read from the
 * device.
 */
typedef struct
{
    /* data */
    struct bmi2_sens_data sensor_data;
} mtb_bmi270_data_t;

/**
 * Enumeration used for selecting I2C address.
 */
typedef enum
{
    /* I2C address */
    MTB_BMI270_ADDRESS_DEFAULT  = BMI2_I2C_PRIM_ADDR,
    MTB_BMI270_ADDRESS_SEC = BMI2_I2C_SEC_ADDR
} mtb_bmi270_address_t;

/*******************************************************************************
* Function name: mtb_bmi270_init_i2c
********************************************************************************
* Summary:
*
* This function initializes the I2C instance, configures the BMI270, and sets
* platform-dependent function pointers.
*
* Parameters:
*
*  dev              Pointer to a BMI270 object. The caller must allocate the
*                   memory for this object but the init function will initialize
*                   its contents.
*  i2c_instance     I2C instance to use for communicating with the BMI270 sensor
*  dev_addr         BMI270 I2C address, set by hardware implementation
*
* Return:
*
*  cy_rslt_t        CY_RSLT_SUCCESS if properly initialized, else an error
*                   indicating what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_init_i2c(mtb_bmi270_t* dev, mtb_hal_i2c_t* i2c_instance,
                              mtb_bmi270_address_t address);

/*******************************************************************************
* Function name: mtb_bmi270_get_sensor_config
********************************************************************************
* Summary:
*
* This API gets the sensor/feature configuration.
* Parameters:
*  config           Pointer to mtb_bmi270_sens_config_t structure instance
*  sensor_count     Number of sensors selected
*  dev              Pointer to a BMI270 object. The caller must allocate the
*                   memory for this object but the init function will initialize
*                   its contents.
*
* Return:
*
*  cy_rslt_t        CY_RSLT_SUCCESS if properly initialized, else an error
*                   indicating what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_get_sensor_config(mtb_bmi270_sens_config_t* config,
                                       uint8_t sensor_count, mtb_bmi270_t* dev);

/*******************************************************************************
* Function name: mtb_bmi270_set_sensor_config
********************************************************************************
* Summary:
*
* This API sets the sensor/feature configuration.
* Parameters:
*  config           Pointer to mtb_bmi270_sens_config_t structure instance
*  sensor_count     Number of sensors selected
*  dev              Pointer to a BMI270 object. The caller must allocate the
*                   memory for this object but the init function will initialize
*                   its contents.
*
* Return:
*
*  cy_rslt_t        CY_RSLT_SUCCESS if properly initialized, else an error
*                   indicating what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_set_sensor_config(mtb_bmi270_sens_config_t* config,
                                       uint8_t sensor_count, mtb_bmi270_t* dev);

/*******************************************************************************
* Function name: mtb_bmi270_sensor_enable
********************************************************************************
* Summary:
*
* This API selects the sensors/features to be enabled
* Parameters:
*  sensor_list      Pointer to select the sensor/feature
*  sensor_count     Number of sensors selected
*  dev              Pointer to a BMI270 object. The caller must allocate the
*                   memory for this object but the init function will initialize
*                   its contents.
*
* Return:
*
*  cy_rslt_t        CY_RSLT_SUCCESS if properly initialized, else an error
*                   indicating what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_sensor_enable(const uint8_t* sensor_list,
                                   uint8_t sensor_count,
                                   mtb_bmi270_t* dev);

/*******************************************************************************
* Function name: mtb_bmi270_sensor_disable
********************************************************************************
* Summary:
*
* This API selects the sensors/features to be disabled
* Parameters:
*  sensor_list      Pointer to select the sensor/feature
*  sensor_count     Number of sensors selected
*  dev              Pointer to a BMI270 object. The caller must allocate the
*                   memory for this object but the init function will initialize
*                   its contents.
*
* Return:
*
*  cy_rslt_t        CY_RSLT_SUCCESS if properly initialized, else an error
*                   indicating what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_sensor_disable(const uint8_t* sensor_list,
                                    uint8_t sensor_count,
                                    mtb_bmi270_t* dev);

/*******************************************************************************
* Function name: mtb_bmi270_config_default
********************************************************************************
* Summary:
*
*   This function configures the motion sensor to a default mode with both
*   accelerometer and gyroscope enabled with a 100Hz output data rate.
*
* Parameters:
*
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*         for this object but the init function will initialize its contents.
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if properly configured, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_config_default(mtb_bmi270_t* dev);

/*******************************************************************************
* Function name: mtb_bmi270_read
********************************************************************************
* Summary:
*
* This function gets the sensor data for accelerometer and gyroscope.
*
* Parameters:
*
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*         for this object but the init function will initialize its contents.
*  data   The accelerometer & gyroscope data read from the motion sensor.
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if properly read, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data);

/*******************************************************************************
* Function name: mtb_bmi270_read_temp
********************************************************************************
* Summary:
*
*    Gets the raw temperature data from the sensor which can be further
*    converted into degree celsius
*
* Parameters:
*
*  dev          Pointer to a BMI270 object. The caller must allocate the memory
*               for this object but the init function will initialize its
*               contents.
*  temp_data    The raw temperature data read from the sensor
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if properly read, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_read_temp(mtb_bmi270_t* dev, uint16_t* temp_data);

/*******************************************************************************
* Function name: mtb_bmi270_selftest
********************************************************************************
* Summary:
*
* Performs both accelerometer and gyro self tests.
*
* Note: These tests cause a soft reset of the device and device should be
*       reconfigured after a test.
*
* Parameters:
*
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*         for this object but the init function will initialize its contents.
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if test passed, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_selftest(mtb_bmi270_t* dev);


#if defined(__cplusplus)
}
#endif

/* [] END OF FILE */
