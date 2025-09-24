/*******************************************************************************
* \file mtb_bmi270.c
*
* \brief
*     This file contains the functions for interacting with the
*     BMI270 motion sensor.
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

#include "mtb_bmi270.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*******************************************************************************
* Macros
*******************************************************************************/
#define _I2C_TIMEOUT_MS            (1U)
#define _READ_WRITE_LEN            (46U)
#define _SOFT_RESET_DELAY_US       (300U)

/*******************************************************************************
* Global variables
*******************************************************************************/
static mtb_hal_i2c_t* _bmi270_i2c = NULL;
static uint8_t _dev_addr;

/*******************************************************************************
* Local function Prototypes
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(uint8_t reg_addr,
                                            uint8_t* reg_data,
                                            uint32_t len,
                                            void* intf_ptr);

static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(uint8_t reg_addr,
                                             const uint8_t* reg_data,
                                             uint32_t len,
                                             void* intf_ptr);

static void _bmi2_delay_us(uint32_t us, void* intf_ptr);

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
cy_rslt_t mtb_bmi270_init_i2c(mtb_bmi270_t* dev,
                              mtb_hal_i2c_t* i2c_instance,
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

    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
                                       uint8_t sensor_count, mtb_bmi270_t* dev)
{
    cy_rslt_t rslt;
    rslt = bmi2_get_sensor_config(&(config->sensor_config), sensor_count,
                                  &(dev->sensor));

    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
                                       uint8_t sensor_count, mtb_bmi270_t* dev)
{
    cy_rslt_t rslt;
    rslt = bmi2_set_sensor_config(&(config->sensor_config), sensor_count,
                                  &(dev->sensor));

    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
                                   mtb_bmi270_t* dev)
{
    cy_rslt_t rslt;
    rslt = bmi2_sensor_enable(sensor_list, sensor_count, &(dev->sensor));

    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
                                    mtb_bmi270_t* dev)
{
    cy_rslt_t rslt;
    rslt = bmi2_sensor_disable(sensor_list, sensor_count, &(dev->sensor));

    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


/*******************************************************************************
* Function name: mtb_bmi270_read
********************************************************************************
* Summary:
*
* This function gets the sensor data for accelerometer and gyroscope.
*
* Parameters:
*
*  dev    Pointer to a BMI270 object. The caller must allocate the
*         memory for this object but the init function will initialize
*         its contents.
*  data   The accelerometer & gyroscope data read from the motion sensor.
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if properly read, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)
{
    cy_rslt_t rslt;
    rslt = bmi2_get_sensor_data(&(data->sensor_data), &(dev->sensor));
    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
*  dev          Pointer to a BMI270 object. The caller must allocate the
*               memory for this object but the init function will initialize
*               its contents.
*  temp_data    The raw temperature data read from the sensor
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if properly read, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_read_temp(mtb_bmi270_t* dev, uint16_t* temp_data)
{
    cy_rslt_t rslt;
    rslt = bmi2_get_temperature_data(temp_data, &(dev->sensor));
    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


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
*  dev    Pointer to a BMI270 object. The caller must allocate the
*         memory for this object but the init function will initialize
*         its contents.
*
* Return:
*
*  cy_rslt_t    CY_RSLT_SUCCESS if test passed, else an error indicating
*               what went wrong.
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_selftest(mtb_bmi270_t* dev)
{
    cy_rslt_t rslt = bmi2_perform_accel_self_test(&(dev->sensor));

    /*per datasheet, delay needed after reset to reboot*/
    mtb_hal_system_delay_us(_SOFT_RESET_DELAY_US);

    if (BMI2_OK == rslt)
    {
        rslt =  bmi2_do_gyro_st(&(dev->sensor));

        /*delay needed after another reset*/
        mtb_hal_system_delay_us(_SOFT_RESET_DELAY_US);
    }
    return (BMI2_OK == rslt) ? CY_RSLT_SUCCESS : rslt;
}


/*******************************************************************************
* Function name: _bmi2_i2c_read
********************************************************************************
* Summary:
*
* This internal function reads I2C function map to host MCU
*
* Parameters:
*
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callbacks
*
* Return:
*
*  BMI2_INTF_RETURN_TYPE     Status of execution
*
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(uint8_t reg_addr,
                                            uint8_t* reg_data,
                                            uint32_t len,
                                            void* intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    cy_rslt_t status = mtb_hal_i2c_controller_write(_bmi270_i2c,
                                                    device_addr,
                                                    &reg_addr,
                                                    1,
                                                    _I2C_TIMEOUT_MS,
                                                    false);
    if (CY_RSLT_SUCCESS == status)
    {
        status = mtb_hal_i2c_controller_read(_bmi270_i2c,
                                             device_addr,
                                             reg_data,
                                             len,
                                             _I2C_TIMEOUT_MS,
                                             true);
    }
    return (BMI2_INTF_RETURN_TYPE)status;
}


/*******************************************************************************
* Function name: _bmi2_i2c_write
********************************************************************************
* Summary:
*
* This internal function writes I2C function map to host MCU
*
* Parameters:
*
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callbacks
*
* Return:
*
*  BMI2_INTF_RETURN_TYPE     Status of execution
*
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(uint8_t reg_addr,
                                             const uint8_t* reg_data,
                                             uint32_t len,
                                             void* intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    cy_rslt_t status = mtb_hal_i2c_controller_write(_bmi270_i2c,
                                                    device_addr,
                                                    &reg_addr,
                                                    1,
                                                    _I2C_TIMEOUT_MS,
                                                    false);

    if (CY_RSLT_SUCCESS == status)
    {
        while (len > 0)
        {
            status = Cy_SCB_I2C_MasterWriteByte(_bmi270_i2c->base,
                                                *reg_data,
                                                _I2C_TIMEOUT_MS,
                                                _bmi270_i2c->context);

            if (CY_SCB_I2C_SUCCESS != status)
            {
                break;
            }
            --len;
            ++reg_data;
        }
        /* SCB in I2C mode is very time sensitive. In practice we have to */
        /* request STOP after each block, otherwise it may break the */
        /* transmission */
        Cy_SCB_I2C_MasterSendStop(_bmi270_i2c->base,
                                  _I2C_TIMEOUT_MS,
                                  _bmi270_i2c->context);
    }

    return (BMI2_INTF_RETURN_TYPE)status;
}


/*******************************************************************************
* Function name: _bmi2_delay_us
********************************************************************************
* Summary:
*
* This internal function maps delay function to host MCU
*
* Parameters:
*
*  us        The time period in microseconds
*  intf_ptr  Void pointer that can enable the linking of descriptors for
*            interface related callbacks
*
* Return:
*
*  void
*
*******************************************************************************/
static void _bmi2_delay_us(uint32_t us, void* intf_ptr)
{
    (void)intf_ptr;

    mtb_hal_system_delay_us(us);
}


#if defined(__cplusplus)
}
#endif

/* [] END OF FILE */
