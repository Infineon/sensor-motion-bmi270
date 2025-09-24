# BMI270 API reference guide

## General description

Basic set of APIs for interacting with the BMI270 motion sensor.

This provides basic initialization and access to the basic accelerometer and gyroscope data. Additionally, provides access to the base BMI270 driver for full control. For more information about the sensor, see [BMI270_SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI/tree/v2.86.1).

## Code snippets

### Snippet 1: Simple initialization with I2C

The following snippet initializes an I2C instance and the BMI270, and then does the configuration of the BMI270.

```
/* Initializes I2C. */
initStatus = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, 
                             &CYBSP_I2C_CONTROLLER_config, 
                             &CYBSP_I2C_CONTROLLER_context);

Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_0_HW);

result = mtb_hal_i2c_setup(&CYBSP_I2C_CONTROLLER_hal_obj,
                           &CYBSP_I2C_CONTROLLER_hal_config, 
                           &CYBSP_I2C_CONTROLLER_context,
                           NULL);

/* Initializes BMI270. */
result = mtb_bmi270_init_i2c(&bmi270, &CYBSP_I2C_CONTROLLER_hal_obj, 
                             MTB_BMI270_ADDRESS_DEFAULT);

/* Default configuration for BMI270. */
result = mtb_bmi270_config_default(&bmi270);
```


### Snippet 2: Read accelerometer and gyroscope data

The following snippet demonstrates how to read the accelerometer and gyroscope from BMI270.

```
/* Reads data from BMI270. */
mtb_bmi270_data_t bmi270_data;
result = mtb_bmi270_read(&bmi270, &bmi270_data);
```

## Data structure

```
struct mtb_bmi270_t               Structure holding the IMU instance-specific information
struct mtb_bmi270_sens_config_t   Structure holding the type of the sensor and its configurations
struct mtb_bmi270_data_t          Structure holding the accelerometer and gyroscope data read from the device
```

## Enumerations

Enumeration used for selecting an I2C address.

 ```
typedef enum
{
    MTB_BMI270_ADDRESS_DEFAULT  = BMI2_I2C_PRIM_ADDR,
    MTB_BMX270_ADDRESS_SEC      = BMI2_I2C_SEC_ADDR
} mtb_bmi270_address_t;
```

## Functions

cy_rslt_t `mtb_bmi270_init_i2c(mtb_bmi270_t* dev, mtb_hal_i2c_t* i2c_instance, mtb_bmi270_address_t address)`
- Initializes the sensor with an I2C interface

cy_rslt_t `mtb_bmi270_get_sensor_config(mtb_bmi270_sens_config_t* config, uint8_t sensor_count, mtb_bmi270_t* dev)`
- This API gets the sensor/feature configuration

cy_rslt_t `mtb_bmi270_set_sensor_config(mtb_bmi270_sens_config_t* config, uint8_t sensor_count, mtb_bmi270_t* dev)`
- This API sets the sensor/feature configuration

cy_rslt_t `mtb_bmi270_sensor_enable(const uint8_t* sensor_list, uint8_t sensor_count, mtb_bmi270_t* dev)`
- This API selects the sensors/features to be enabled

cy_rslt_t `mtb_bmi270_sensor_disable(const uint8_t* sensor_list, uint8_t sensor_count, mtb_bmi270_t* dev)`
- This API selects the sensors/features to be disabled

cy_rslt_t `mtb_bmi270_config_default(mtb_bmi270_t* dev)`
- Configures the motion sensor to a default mode with both accelerometer and gyroscope enabled with a nominal output data rate

cy_rslt_t `mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)`
- Reads the current accelerometer and gyroscope data from the motion sensor

cy_rslt_t `mtb_bmi270_read_temp(mtb_bmi270_t* dev, uint16_t *temp_data)`
- Reads the current raw temperature data from the sensor

cy_rslt_t `mtb_bmi270_selftest(mtb_bmi270_t* dev)`
- Performs both accelerometer and gyro self-tests


## Data structure documentation

- mtb_bmi270_t

    Data fields                    |           object       |    Description
    :-------                       |  :------------         |  :------------
    bmi2_dev                       | sensor                 |  Structure to define BMI270 sensor configurations
    mtb_hal_gpio_t                 | intpin1                |  Internal structure containing callback data for pins
    mtb_hal_gpio_t                 | intpin2                |  Internal structure containing callback data for pins

<br>

- mtb_bmi270_sens_config_t

    Data fields          |  object                  |    Description
    :-------             |  :------------           |  :------------
    bmi2_sens_config     | sensor_config            |  Sensor configurations

<br>

- mtb_bmi270_data_t

    Data fields        |           object       |    Description
    :-------           |  :------------         |  :------------
    bmi2_sens_data     | sensor_data            |     Accelerometer and gyroscope data

<br>

## Function documentation

#### mtb_bmi270_init_i2c

- cy_rslt_t `mtb_bmi270_init_i2c(mtb_bmi270_t* dev, mtb_hal_i2c_t* i2c_instance, mtb_bmi270_address_t address)`

    **Summary:** This function initializes the I2C instance, configures the BMI270, and sets platform-dependent function pointers.

   **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents
    i2c_instance          |  I2C instance to use for communicating with the BMI270 sensor
    dev_addr              |  BMI270 I2C address, set by hardware implementation

    <br>

    **Return**
    
    - cy_rslt_t: CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong


#### mtb_bmi270_get_sensor_config

- cy_rslt_t `mtb_bmi270_get_sensor_config(mtb_bmi270_sens_config_t* config, uint8_t sensor_count, mtb_bmi270_t* dev)`

    **Summary:** This API gets the sensor/feature configuration.

   **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    config                |  Pointer to mtb_bmi270_sens_config_t structure instance
    sensor_count          |  Number of sensors selected   
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    <br>

    **Return**
    
    - cy_rslt_t: CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong


#### mtb_bmi270_set_sensor_config

- cy_rslt_t `mtb_bmi270_set_sensor_config(mtb_bmi270_sens_config_t* config, uint8_t sensor_count, mtb_bmi270_t* dev)`

    **Summary:** This API sets the sensor/feature configuration.

   **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    config                |  Pointer to mtb_bmi270_sens_config_t structure instance
    sensor_count          |  Number of sensors selected   
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    <br>

    **Return**
    
    - cy_rslt_t: CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong


#### mtb_bmi270_sensor_enable

- cy_rslt_t `mtb_bmi270_sensor_enable(const uint8_t* sensor_list, uint8_t sensor_count, mtb_bmi270_t* dev)`

    **Summary:** This API selects the sensors/features to be enabled.

   **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    sensor_list           |  Pointer to select the sensor/feature
    sensor_count          |  Number of sensors selected   
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    <br>

    **Return**
    
    - cy_rslt_t: CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong


#### mtb_bmi270_sensor_disable

- cy_rslt_t `mtb_bmi270_sensor_disable(const uint8_t* sensor_list, uint8_t sensor_count, mtb_bmi270_t* dev)`

    **Summary:** This API selects the sensors/features to be disabled.

   **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    sensor_list           |  Pointer to select the sensor/feature
    sensor_count          |  Number of sensors selected   
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    <br>

    **Return**
    
    - cy_rslt_t: CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong


#### mtb_bmi270_config_default

- cy_rslt_t `mtb_bmi270_config_default(mtb_bmi270_t* dev)`

    **Summary:** Configures the motion sensor to a default mode with both accelerometer and gyroscope enabled with a nominal output data rate.

    The default values used are from the example in the BMI270 driver repository.

    **Parameter**

    Parameters            |  Description
    :-------              |  :------------
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    **Return**

    - cy_rslt_t: CY_RSLT_SUCCESS if properly configured, else an error indicating what went wrong


#### mtb_bmi270_read

- cy_rslt_t `mtb_bmi270_read(mtb_bmi270_t* dev, mtb_bmi270_data_t* data)`

    **Summary:** Reads the current accelerometer and gyroscope data from the motion sensor.

    **Parameter**

    Parameters           |  Description
    :-------             |  :------------
    dev                  |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents
    data                 |  The accelerometer and gyroscope data read from the motion sensor

    <br>

    **Return**

    - cy_rslt_t          :  CY_RSLT_SUCCESS if properly read, else an error indicating what went wrong


#### mtb_bmi270_read_temp

- cy_rslt_t `mtb_bmi270_read_temp(mtb_bmi270_t* dev, uint16_t *temp_data)`

    **Summary:** Gets the raw temperature data from the sensor which can be further converted into degree Celsius.

    **Parameter**

    Parameters           |  Description
    :-------             |  :------------
    dev                  |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents
    data                 |  The raw temperature data read from the sensor

    <br>

    **Return**

    - cy_rslt_t:  CY_RSLT_SUCCESS if properly read, else an error indicating what went wrong


#### mtb_bmi270_selftest

- cy_rslt_t `mtb_bmi270_selftest(mtb_bmi270_t* dev)`

    **Summary:** Performs both accelerometer and gyro self-tests.

    Note that these tests cause a soft reset of the device and reconfigure the device after a test.

    **Parameter**
    
    Parameters            |  Description
    :-------              |  :------------
    dev                   |  Pointer to a BMI270 object. The caller must allocate the memory for this object but the init function will initialize its contents

    <br>

    **Return**

    - cy_rslt_t           :  CY_RSLT_SUCCESS if test passed, else an error indicating what went wrong



---
© 2024-2025, Cypress Semiconductor Corporation (an Infineon company)
