# BMI270 API Reference Guide

## General Description

Basic set of API for interacting with the BMI270 motion sensor.

This provides basic initialization and access to the basic accelerometer and gyroscope data. It also provides access to the base BMI270 driver for full control. For more information about the sensor, see: https://github.com/boschsensortec/BMI270_SensorAPI/tree/v2.86.1

## Code Snippets

### Snippet 1: Simple initialization with I2C
The following snippet initializes an I2C instance and the BMI270, then does configuration of BMI270.

```
/* Initialize I2C and BMI270 */
result = cyhal_i2c_init(&i2c_instance, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
result = mtb_bmi270_init_i2c(&imu, &i2c_instance, MTB_BMI270_ADDRESS_DEFAULT);
/* Default configuration for BMI270 */
result = mtb_bmi270_config_default(&imu);
```


### Snippet 2: Read accelerometer and gyroscope data
The following snippet demonstrates how to read accelerometer and gyroscope from BMI270.

```
/* Read data from BMI270 */
mtb_bmi270_data_t data;
result = mtb_bmi270_read(&imu, &data);
```

## Data Structure
```
struct mtb_bmi270_t         Structure holding the IMU instance specific information
struct mtb_bmi270_data_t    Structure holding the accelerometer and gyroscope data read from the device
```

## Enumerations
Enumeration used for selecting I2C address.

 ```
typedef enum
{
    MTB_BMI270_ADDRESS_DEFAULT  = BMI2_I2C_PRIM_ADDR,
    MTB_BMX270_ADDRESS_SEC      = BMI2_I2C_SEC_ADDR
} mtb_bmi270_address_t;
```

## Functions
cy_rslt_t `mtb_bmi270_init_i2c(mtb_bmi270_t* dev, cyhal_i2c_t* i2c_instance, mtb_bmi270_address_t dev_addr)`
>Initializes the sensor with I2C interface.

cy_rslt_t `mtb_bmi270_config_default(mtb_bmi270_t* dev)`
>Configures the motion sensor to a default mode with both accelerometer and gyroscope enabled with a nominal output data rate.

cy_rslt_t `mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)`
>Reads the current accelerometer and gyroscope data from the motion sensor.

cy_rslt_t `mtb_bmi270_selftest(mtb_bmi270_t* dev)`
>Performs both accelerometer and gyro self tests.

cy_rslt_t `mtb_bmi270_config_int(mtb_bmi270_t* dev, struct bmi2_int_pin_config* intsettings, cyhal_gpio_t pin, uint8_t intr_priority, cyhal_gpio_event_t event, cyhal_gpio_event_callback_t callback, void* callback_arg)`
>This configures the pin as an interrupt, and calls the BMI270 interrupt configuration API with the application supplied settings structure.

void `mtb_bmi270_free_pin(mtb_bmi270_t* dev)`
>Frees up any resources allocated by the sensor as part of mtb_bmi270_init_i2c().

## Data Structure Documentation
- mtb_bmi270_t

 Data Fields                      |           object       |    Description
 :-------                       |  :------------         |  :------------
 bmi2_dev                       | sensor                 |  Structure to define BMI270 sensor configurations
 _mtb_bmi270_interrupt_pin_t    | intpin1                |  Internal structure containing callback data for pins
 _mtb_bmi270_interrupt_pin_t    | intpin2                |  Internal structure containing callback data for pins

- mtb_bmi270_data_t

 Data Fields          |           object       |    Description
 :-------           |  :------------         |  :------------
 bmi2_sens_data     | sensor_data            |     Accelerometer and Gyroscope data

## Function Documentation
#### mtb_bmi270_init_i2c
- cy_rslt_t `mtb_bmi270_init_i2c(mtb_bmi270_t* dev, cyhal_i2c_t* i2c_instance, mtb_bmi270_address_t dev_addr);`

> **Summary:** This function initializes the I2C instance, configures the BMI270, and sets platform-dependent function pointers.
>
> **Parameter:**
>  Parameters            |  Description
>  :-------              |  :------------
>  dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.
>  i2c_instance          |  I2C instance to use for communicating with the BMI270 sensor.
>  dev_addr              |  BMI270 I2C address, set by hardware implementation.
>
> Return:
>  - cy_rslt_t           :  CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.

#### mtb_bmi270_config_default
- cy_rslt_t `mtb_bmi270_config_default(mtb_bmi270_t* dev)`

>**Summary:** Configures the motion sensor to a default mode with both accelerometer and gyroscope enabled with a nominal output data rate.
>
>The default values used are from the example in the BMI270 driver repository.
>
> **Parameter:**
>  Parameters            |  Description
>  :-------              |  :------------
>  dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.
>
> Return:
>  - cy_rslt_t           :  CY_RSLT_SUCCESS if properly configured, else an error indicating what went wrong.

#### mtb_bmi270_read
- cy_rslt_t `mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)`

>**Summary:** Reads the current accelerometer and gyroscope data from the motion sensor.
>
> **Parameter:**
>  Parameters           |  Description
>  :-------             |  :------------
>  dev                  |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.
>  data                 |  The accelerometer and gyroscope data read from the motion sensor
>
> Return:
>  - cy_rslt_t          :  CY_RSLT_SUCCESS if properly read, else an error indicating what went wrong.

#### mtb_bmi270_selftest
- cy_rslt_t `mtb_bmi270_selftest(mtb_bmi270_t* dev)`

>**Summary:** Performs both accelerometer and gyro self tests.
>
> Note these tests cause a soft reset of the device and device should be reconfigured after a test.
>
> **Parameter:**
>  Parameters            |  Description
>  :-------              |  :------------
>  dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.
>
> Return:
>  - cy_rslt_t           :  CY_RSLT_SUCCESS if test passed, else an error indicating what went wrong.

#### mtb_bmi270_config_int
- cy_rslt_t `mtb_bmi270_config_int(mtb_bmi270_t* dev, struct bmi2_int_pin_config* intsettings, cyhal_gpio_t pin, uint8_t intr_priority, cyhal_gpio_event_t event, cyhal_gpio_event_callback_t callback, void* callback_arg)`

>**Summary:** This configures the pin as an interrupt, and calls the BMI270 interrupt configuration API with the application supplied settings structure.
>
>**Note:** To get the current interrupt configuration, `bmi2_get_int_pin_config` should be called in the application.
>
> **Parameter:**
>  Parameters           |  Description
>  :-------             |  :------------
>  dev                  |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.
>  intsettings          |  Pointer to a BMI270 interrupt settings structure.
>  pin                  |  Which pin to configure as interrupt.
>  intr_priority        |  The priority for NVIC interrupt events.
>  event                |  The type of interrupt event.
>  callback             |  The function to call when the specified event happens. Pass NULL to unregister the handler.
>  callback_arg         |  Generic argument that will be provided to the callback when called, can be NULL.
>
> Return:
>  - cy_rslt_t          :  CY_RSLT_SUCCESS if properly configured, else an error indicating what went wrong.

#### mtb_bmi270_free_pin
- void `mtb_bmi270_free_pin(mtb_bmi270_t* dev)`

>**Summary:** Frees up any resources allocated by the motion sensor as part of mtb_bmi270_init_i2c().
>
> **Parameter:**
>  Parameters            |  Description
>  :-------              |  :------------
>  dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents.

---
© 2024, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
