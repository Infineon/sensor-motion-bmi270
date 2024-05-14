/******************************************************************************
 * \file mtb_bmi270.h
 *
 * \brief
 *     This file is the public interface of the BMI270 motion sensor.
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

#pragma once

#include "bmi270.h"
#include "cy_result.h"
#include "cyhal.h"
#include "cyhal_gpio.h"
#if defined(__cplusplus)
extern "C"
{
#endif

typedef cyhal_gpio_callback_data_t  _mtb_bmi270_interrupt_pin_t;

/**
 * Structure holding the IMU instance specific information.
 *
 * Application code should not rely on the specific content of this struct.
 * They are considered an implementation detail which is subject to change
 * between platforms and/or library releases.
 */
typedef struct
{
    struct bmi2_dev           sensor;
    _mtb_bmi270_interrupt_pin_t intpin1;
    _mtb_bmi270_interrupt_pin_t intpin2;
} mtb_bmi270_t;

/** Structure holding the accelerometer and gyroscope data read from the device. */
typedef struct
{
    /** data */
    struct bmi2_sens_data sensor_data;
} mtb_bmi270_data_t;

/**
 * Enumeration used for selecting I2C address.
 */
typedef enum
{
    /** I2C address */
    MTB_BMI270_ADDRESS_DEFAULT  = BMI2_I2C_PRIM_ADDR,
    MTB_BMI270_ADDRESS_SEC = BMI2_I2C_SEC_ADDR
} mtb_bmi270_address_t;

/*****************************************************************************
* Function name: mtb_bmi270_init_i2c
*****************************************************************************
* Summary:
* This function initializes the I2C instance, configures the BMI270, and sets
* platform-dependent function pointers.
*
* Parameters:
*  dev                    Pointer to a BMI270 object. The caller must allocate the memory
*  for this object but the init function will initialize its contents.
*  i2c_instance           I2C instance to use for communicating with the BMI270 sensor.
*  dev_addr               BMI270 I2C address, set by hardware implementation
* Return:
*  cy_rslt_t    CY_RSLT_SUCCESS if properly initialized, else an error indicating
*               what went wrong.
*
*****************************************************************************/
cy_rslt_t mtb_bmi270_init_i2c(mtb_bmi270_t* dev, cyhal_i2c_t* i2c_instance,
                              mtb_bmi270_address_t dev_addr);

/*****************************************************************************
* Function Name: mtb_bmi270_config_default
*****************************************************************************
* Summary:
*     This function configures the accelerometer and gyroscope with a 100 Hz
*     output data rate.
*
* Parameters:
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*         for this object but the init function will initialize its contents.
*
* Return:
*  cy_rslt_t    CY_RSLT_SUCCESS if properly configured, else an error indicating
*               what went wrong.
*
*****************************************************************************/
cy_rslt_t mtb_bmi270_config_default(mtb_bmi270_t* dev);

/*****************************************************************************
* Function Name: mtb_bmi270_read
*****************************************************************************
* Summary:
* This function gets the sensor data for accelerometer and gyroscope.
*
* Parameters:
*  dev           Pointer to a BMI270 object. The caller must allocate the memory
*  for this object but the init function will initialize its contents.
*  data          The accelerometer & gyroscope data read from the motion sensor.
*
* Return:
*  cy_rslt_t    CY_RSLT_SUCCESS if properly read, else an error indicating
*               what went wrong.
*
*****************************************************************************/
cy_rslt_t mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data);

/*****************************************************************************
* Function Name: mtb_bmi270_selftest
*****************************************************************************
* Summary:
* Performs both accelerometer and gyro self tests.
* Note: These tests cause a soft reset of the device and device should be reconfigured after a test.
*
* Parameters:
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*  for this object but the init function will initialize its contents.
*
* Return:
*  cy_rslt_t    CY_RSLT_SUCCESS if test passed, else an error indicating
*               what went wrong.
*
*****************************************************************************/
cy_rslt_t mtb_bmi270_selftest(mtb_bmi270_t* dev);

/*****************************************************************************
* Function Name: mtb_bmi270_free_pin
*****************************************************************************
* Summary:
* Frees up any resources allocated by the motion_sensor as part of \ref mtb_bmi270_init_i2c().
*
* Parameters:
*  dev    Pointer to a BMI270 object. The caller must allocate the memory
*  for this object but the init function will initialize its contents.
*****************************************************************************/
void mtb_bmi270_free_pin(mtb_bmi270_t* dev);

#if defined(__cplusplus)
}
#endif

/* [] END OF FILE */
