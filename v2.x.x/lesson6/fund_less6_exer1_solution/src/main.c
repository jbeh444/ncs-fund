/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Note: Tested on nRF52833 DK
 */
//#include "arduino_bma456.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "bma456.h"
#include "bma4_defs.h"
#include "bma4.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* STEP 6 - Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysensor)

void main(void)
{
	struct bma4_dev bma = { 0 };
	uint8_t ret;
	uint8_t chip_ID;
	uint8_t status_features = 0x2A;

	
	ret = bma456_init(&bma);

/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return;
	}		
	printk("Chip ID is %x \n", chip_ID);
/* STEP 9 - Setup the sensor by writing the value 0x8C to the Configuration register */
	//uint8_t config[2] = {STTS751_CONFIG_REG,0x8C};
	printk("I2C bus %s is ready!\n\r",dev_i2c.bus->name);
	ret = i2c_write_dt(&dev_i2c, 0x00, sizeof(0x00));//set pointer to 0x00 in BMA456 to read chip ID		
	ret = i2c_read_dt(&dev_i2c, &chip_ID, sizeof(chip_ID));	
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg. %x n", dev_i2c.addr,chip_ID);
	}
	printk("Chip ID is %x \n", chip_ID);
	if (chip_ID == 0x16){
		bma.chip_id = chip_ID; //set chip ID in bma4_dev struct
		printk("Chip ID is correct \n");
	}
	else{
		printk("Chip ID is incorrect \n");
	}
	uint8_t disable_pwr[2] = {0x7C,0x00};
	ret = i2c_write_dt(&dev_i2c, disable_pwr, sizeof(disable_pwr));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,disable_pwr[0]);
		return;
	}
	printk("Power down mode disabled \n");		
	k_msleep(SLEEP_TIME_MS);
	uint8_t prepare_features[2] = {0x59,0x00};
	ret = i2c_write_dt(&dev_i2c, prepare_features, sizeof(prepare_features));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,prepare_features[0]);
		return;
	}
	printk("Features prepared \n");
	ret = bma456_write_config_file(&bma);
	ret = i2c_burst_write_dt(&dev_i2c, 0x5E, *bma.config_file_ptr, sizeof(*bma.config_file_ptr));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. \n", dev_i2c.addr);
		return;
	}
	printk("Config file written \n");

	uint8_t enable_features[2] = {0x59,0x01};
	ret = i2c_write_dt(&dev_i2c, enable_features, sizeof(enable_features));	
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,enable_features[0]);
		return;
	}
	printk("Features enabled \n");
	k_msleep(140);
	
	printk("Status Features %x \n", status_features);	
	ret = i2c_read_dt(&dev_i2c, &status_features, sizeof(status_features));
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg. %x \n", dev_i2c.addr,status_features);
		return;
	}
	printk("Status Features %x \n", status_features);

	printk("BMA456 is ready!\n\r");
	printk("\n\r");

	//ret = bma4_set_accel_enable(BMA4_ENABLE, &bma);
	//ret = bma456_feature_enable(BMA456_STEP_CNTR, BMA4_ENABLE, &bma);
	//ret = bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_STEP_CNTR_INT, BMA4_ENABLE, &bma);
// 	//ret = i2c_write_dt(&dev_i2c, 0x5E ,sizeof(0x5E)); 
	//i2c_burst_write_dt(&dev_i2c, 0x5E, init_array , sizeof(init_array));
// 	ret = i2c_write_dt(&dev_i2c,bma456_config_file,sizeof(bma456_config_file));
// 	while (1) {
// /* STEP 10 - Read the temperature from the sensor */
// 		uint8_t temp_reading[2]= {0};
// 		uint8_t sensor_regs[2] ={STTS751_TEMP_LOW_REG,STTS751_TEMP_HIGH_REG};
// 		ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],1,&temp_reading[0],1);
// 		if(ret != 0){
// 			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[0]);
// 		}
// 		ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[1],1,&temp_reading[1],1);
// 		if(ret != 0){
// 			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[1]);
// 		}

// /* STEP 11 - Convert the two bytes to a 12-bits */
// 		int temp = ((int)temp_reading[1] * 256 + ((int)temp_reading[0] & 0xF0)) / 16;
// 		if(temp > 2047)
// 		{
// 			temp -= 4096;
// 		}

// 		// Convert to engineering units 
// 		double cTemp = temp * 0.0625;
// 		double fTemp = cTemp * 1.8 + 32;

// 		//Print reading to console  
// 		printk("Temperature in Celsius : %.2f C \n", cTemp);
// 		printk("Temperature in Fahrenheit : %.2f F \n", fTemp);
// 		k_msleep(SLEEP_TIME_MS);
// 	}
}
