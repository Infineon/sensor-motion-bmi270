# BMI270 inertial measurement unit (IMU - motion sensor)

## Overview

This library provides functions for interfacing with the BMI270 I2C 16-bit inertial measurement unit (IMU) with 3-axis accelerometer and 3-axis gyroscope used on the PSOC&trade; Edge E84 Evaluation Kit.

[BMI270 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)

[BMI270_SensorAPI GitHub](https://github.com/boschsensortec/BMI270_SensorAPI)

## Quick start

Follow these steps to create a simple application, which outputs the motion sensor data from the sensor to the UART.

1. Create a new application in the ModusToolbox&trade; and select the appropriate board support package (BSP)
2. Select the [PSOC&trade; Edge MCU: Hello world](https://github.com/Infineon/mtb-example-psoc-edge-hello-world) application template and create it
3. Add sensor-motion-bmi270 library to the application using the library manager
4. Place following code in the *main.c* file of the non-secure application of your project (*proj_cm33_ns*)

```
#include "cy_syslib.h"
#include "cybsp.h"
#include "mtb_bmi270.h"
#include "mtb_hal.h"

static mtb_hal_i2c_t CYBSP_I2C_CONTROLLER_hal_obj;
cy_stc_scb_i2c_context_t CYBSP_I2C_CONTROLLER_context;

cy_en_scb_i2c_status_t initStatus;
cy_rslt_t result;

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    mtb_bmi270_data_t bmi270_data;
    mtb_bmi270_t bmi270;
    initStatus = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW,
                                 &CYBSP_I2C_CONTROLLER_config,
                                 &CYBSP_I2C_CONTROLLER_context);

    /* I2C initialization failed. Stop program execution. */
    if (CY_SCB_I2C_SUCCESS != initStatus)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    result = mtb_hal_i2c_setup(&CYBSP_I2C_CONTROLLER_hal_obj,
                               &CYBSP_I2C_CONTROLLER_hal_config,
                               &CYBSP_I2C_CONTROLLER_context,
                               NULL);

    /* HAL I2C setup failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Initialize can configure platform-dependent function pointers. */
    result = mtb_bmi270_init_i2c(&bmi270,
                                 &CYBSP_I2C_CONTROLLER_hal_obj,
                                 MTB_BMI270_ADDRESS_DEFAULT);

    /* BMI270 sensor initialization failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Configure accelerometer and gyroscope. */
    result = mtb_bmi270_config_default(&bmi270);

    /* BMI270 sensor configuration failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    for (;;)
    {
        /* Get accelerometer and gyroscope sensor data. */
        result = mtb_bmi270_read(&bmi270, &bmi270_data);

        /* BMI270 sensor read failed. Stop program execution. */
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }
        /* Performs application-specific task. */
    }
}
```
5. Builds the application and program the kit


## More information

For more information, see the following documents:

* [API reference guide](./api_reference.md)
* [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox)
* [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge MCU on ModusToolbox&trade; application note
* [Infineon Technologies AG](https://www.infineon.com)

-----
© 2024-2025, Cypress Semiconductor Corporation (an Infineon company)
