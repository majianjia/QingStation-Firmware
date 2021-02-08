# BMM150 sensor API
## Introduction
This package contains the Bosch Sensortec's BMM150 Magnetometer sensor driver (sensor API)

The sensor driver package includes bmm150.h, bmm150.c and bmm150_defs.h files

bmm150 sensor driver supports the following Bosch MEMS sensors
* BMM050
* BMM150
* BMC150 - Combination of bma2x2 + bmm150 APIs
* BMC056 - Combination of bma2x2 + bmm150 APIs
* BMX055 - Combination of bma2x2 + bmg160 + bmm150 APIs
    
## Version
File          | Version | Date
--------------|---------|-------------
bmm150.c      |  1.0.0  | 12 Sep 2017
bmm150.h      |  1.0.0  | 12 Sep 2017
bmm150_defs.h |  1.0.0  | 12 Sep 2017

## Integration details
* Integrate bmm150.h, bmm150_defs.h and bmm150.c file in to your project.
* Include the bmm150.h file in your code like below.
``` c
#include "bmm150.h"
```

## File information
* bmm150_defs.h : This header file has the constants, macros and datatype declarations.
* bmm150.h      : This header file contains the declarations of the sensor driver APIs.
* bmm150.c      : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire
* I2C

## Usage guide
### Initializing the sensor
To initialize the sensor, you will first need to create a device structure. You 
can do this by creating an instance of the structure bmm150_dev. Then go on to 
fill in the various parameters as shown below.


#### Example for SPI 4-Wire
``` c
struct bmm150_dev dev;
int8_t rslt = BMM150_OK;

/* Sensor interface over SPI with native chip select line */
dev.dev_id = 0;
dev.intf = BMM150_SPI_INTF;
dev.read = user_spi_read;
dev.write = user_spi_write;
dev.delay_ms = user_delay_ms;

rslt = bmm150_init(&dev);
```
#### Example for I2C
``` c
struct bmm150_dev dev;
int8_t rslt = BMM150_OK;

/* Sensor interface over I2C */
dev.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
dev.intf = BMM150_I2C_INTF;
dev.read = user_i2c_read;
dev.write = user_i2c_write;
dev.delay_ms = user_delay_ms;

rslt = bmm150_init(&dev);
```
### Sensor Configuration settings
#### Setting Normal operation mode (power mode) and preset mode.
##### Example for configuring the sensor in normal power mode and Low power preset mode
``` c
int8_t set_sensor_settings(struct bmm150_dev *dev)
{
	int8_t rslt;

	/* Setting the power mode as normal */
	dev->settings.pwr_mode = BMM150_NORMAL_MODE;
	rslt = bmm150_set_op_mode(dev);
	
	/* Setting the preset mode as Low power mode 
	i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
	dev->settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
	rslt = bmm150_set_presetmode(dev);
	
	return rslt;	
}
```
### Reading sensor data 
 * Sensor data should be read after setting the desired power mode and preset mode.
#### Example for reading sensor data - Fixed point version
> Ensure that the macro "BMM150_USE_FLOATING_POINT" in the bmm150_defs.h file is commented out
``` c
int8_t read_sensor_data(struct bmm150_dev *dev)
{
	int8_t rslt;

	/* Mag data for X,Y,Z axis are stored inside the
	bmm150_dev structure in int16_t format */
	rslt = bmm150_read_mag_data(dev);

	/* Print the Mag data */
	printf("\n Magnetometer data \n");
	printf("MAG X : %d \t MAG Y : %d \t MAG Z : %d \n"
		,dev->data.x, dev->data.y, dev->data.z);
	
	return rslt;
}
```
#### Example for reading sensor data - Floating point version
> Enable the macro "BMM150_USE_FLOATING_POINT" in the bmm150_defs.h file (or)
> enable "BMM150_USE_FLOATING_POINT" in the compiler option of your project
``` c
int8_t read_sensor_data(struct bmm150_dev *dev)
{
	int8_t rslt;

	/* Mag data for X,Y,Z axis are stored inside the
	bmm150_dev structure in float format */
	rslt = bmm150_read_mag_data(dev);

	/* Print the Mag data */
	printf("\n Magnetometer data \n");
	printf("MAG X : %0.2f \t MAG Y : %0.2f \t MAG Z : %0.2f \n"
		,dev->data.x, dev->data.y, dev->data.z);
	
	return rslt;
}
```
### Self test - Normal self test and Advanced self test
#### Example for performing Normal self test and Advanced self test
``` c
int8_t perform_self_tests(struct bmm150_dev *dev)
{
	int8_t rslt;

	/* Perform Normal Self test */
	rslt = bmm150_perform_self_test(BMM150_NORMAL_SELF_TEST, dev);
	printf("\n NORMAL SELF TEST RESULT :  %d",rslt);
	
	/* Validate normal self test result */
	if (rslt == BMM150_OK) {
		printf("\n Normal Self test passed ");
	} else {
		printf("\n Normal Self test failed ");
	}
	/* Perform Advanced Self test */
	rslt |= bmm150_perform_self_test(BMM150_ADVANCED_SELF_TEST, dev);
	printf("\n ADVANCED SELF TEST RESULT : %d",rslt);

	/* Validate Advanced self test result */
	if (rslt == BMM150_OK) {
		printf("\n Advanced Self test passed ");
	} else {
		printf("\n Advanced Self test failed ");
	}
	
	return rslt;
}
```
### Interrupt settings
#### Example for using Data ready interrupt to read data
> Configuring the interrupt
``` c
int8_t drdy_interrupt_configure(struct bmm150_dev *dev)
{
	int8_t rslt;
	uint16_t desired_settings;

	/* Set the macros to enable DRDY pin */
	desired_settings = BMM150_DRDY_PIN_EN_SEL | BMM150_DRDY_POLARITY_SEL;
	/* Set the drdy_pin_en to enable the drdy interrupt  */
	dev->settings.int_settings.drdy_pin_en = BMM150_INT_ENABLE;
	/* Set the polarity as active high on the DRDY pin */
	dev->settings.int_settings.drdy_polarity = BMM150_ACTIVE_HIGH_POLARITY; 
	
	/* Set the configurations in the sensor */
	rslt = bmm150_set_sensor_settings(desired_settings, dev);
	
	return rslt;
}
```

> Read the interrupt status continuously to check for interrupt assertion (Polling method)
``` c
int8_t drdy_interrupt_handling(struct bmm150_dev *dev)
{
	int8_t rslt;
	
	/* Read the interrupt status */
	rslt = bmm150_get_interrupt_status(dev);
	if (rslt == BMM150_OK) {
		if (dev->int_status & BMM150_DATA_READY_INT) {
			/* Interrupt asserted - Read mag data */
			rslt = bmm150_read_mag_data(dev);
			printf("\n MAG DATA ");
			printf("\n MAG X : %d MAG Y : %d MAG Z : %d "
				,dev->data.x,dev->data.y,dev->data.z);
		} else {
			/*Interrupt not asserted */
			printf("\n Data is not ready yet");
		}
	}
	
	return rslt;
}