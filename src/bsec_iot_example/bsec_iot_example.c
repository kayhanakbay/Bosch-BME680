/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.c
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME68x sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


#include "../bme68x/bme68x.h"
#include "../inc//bsec_interface.h"
#include "bsec_integration.h"
#include "../config/Selectivity_Config.h"


int8_t user_i2c_read(int fd, uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    /*
    * Data on the bus should be like
    * |------------+---------------------|
    * | I2C action | Data                |
    * |------------+---------------------|
    * | Start      | -                   |
    * | Write      | (reg_addr)          |
    * | Stop       | -                   |
    * | Start      | -                   |
    * | Read       | (reg_data[0])       |
    * | Read       | (....)              |
    * | Read       | (reg_data[len - 1]) |
    * | Stop       | -                   |
    * |------------+---------------------|
    */


//    char i2cDevicePath[20]; // Adjust the size as needed
//    snprintf(i2cDevicePath, sizeof(i2cDevicePath), "/dev/i2c-%d", fd);
//
//    int i2c = open(i2cDevicePath, O_RDWR);
//    if (i2c < 0) {
//        perror("Unable to open I2C device");
//        return 1; // Return failure
//    }
//
//    printf("r1 \n");
//    if (ioctl(i2c, I2C_SLAVE, dev_id) < 0) {
//        printf("Unable to set I2C address \n");
//        return 1; // Return failure
//    }

   // printf("Read : i2c bus =  %d -> dev_id = %d -> reg_addr = %d -> len = %d ", fd, dev_id, reg_addr, len);

    char i2cDevicePath[20]; // Adjust the size as needed
    snprintf(i2cDevicePath, sizeof(i2cDevicePath), "/dev/i2c-%d", fd);

    int i2c = open(i2cDevicePath, O_RDWR);
    if (i2c < 0) {
        printf("Unable to open I2C device \n");
        return 1; // Return failure
    }

    if (ioctl(i2c, I2C_SLAVE, dev_id) < 0) {
        printf("Error setting I2C slave address \n");
        return 1;
    }

    if (write(i2c, &reg_addr, 1) != 1) {
        printf("Error writing register address");
        close(i2c);
        return -1;
    }

    if (read(i2c, reg_data, len) != len) {
        printf("Error reading data \n");
        return -1;
    }

//    printf("Read data:");
//    for (uint16_t i = 0; i < len; i++) {
//        printf(" %02X", reg_data[i]);
//    }
//    printf("\n");

    close(i2c);
    return 0;// Return success
}


int8_t user_i2c_write(int fd, uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint16_t len) {
    /*
  * Data on the bus should be like
  * |------------+---------------------|
  * | I2C action | Data                |
  * |------------+---------------------|
  * | Start      | -                   |
  * | Write      | (user_i2c_writereg_addr)          |
  * | Write      | (reg_data[0])       |
  * | Write      | (....)              |
  * | Write      | (reg_data[len - 1]) |
  * | Stop       | -                   |
  * |------------+---------------------|
  */
   // printf("Write : i2c bus id = %d -> dev_id = %d -> reg_addr = %d \n", fd, dev_id, reg_addr);

    char i2cDevicePath[20];
    snprintf(i2cDevicePath, sizeof(i2cDevicePath), "/dev/i2c-%d", fd);

    int i2c = open(i2cDevicePath, O_RDWR);
    if (i2c < 0) {
        printf("Unable to open I2C device");
        return 1; // Return failure
    }

    if (ioctl(i2c, I2C_SLAVE, dev_id) < 0) {
        printf("Error setting I2C slave address");
        close(i2c);
        return 1; // Return failure
    }

    // Prepare data buffer
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    for (uint16_t i = 0; i < len; ++i) {
        buffer[i + 1] = reg_data[i];
    }

    // Write data to I2C device
    ssize_t bytes_written = write(i2c, buffer, len + 1);
    if (bytes_written != len + 1) {
        printf("Error writing to I2C");
        close(i2c);
        return 1; // Return failure
    }

    close(i2c);
    return 0; // Return success
}


/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

typedef struct String String;

/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 * param[in]        intf_ptr        interface pointer
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr) {
    // ...
    // Please insert system specific function to write to the bus where BME68x is connected
    // ...
    return (user_i2c_write((int) intf_ptr, BME68X_I2C_ADDR_LOW, reg_addr, reg_data_ptr, data_len));
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 * param[in]        intf_ptr        interface pointer
 * 
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr) {
    //printf("bus read..\n");
    // ...
    // Please insert system specific function to read from bus where BME68x is connected
    // ...
    return (user_i2c_read((int) intf_ptr, BME68X_I2C_ADDR_LOW, reg_addr, reg_data_ptr, data_len));
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_us     Time in microseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
void sleep_n(uint32_t t_us, void *intf_ptr) {


    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...

    // Convert microseconds to milliseconds for usleep
    useconds_t t_ms = t_us / 1000;

    //printf("\niot sleep func start sleep time ( microseconds ) = %d\n",t_us);

    // Sleep for the specified time
    usleep(t_us);


   // printf("\niot sleep func finish sleep\n");
}

uint32_t overflowCounter;
uint32_t lastTimeUS;

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us() {
    //    int64_t system_current_time = 0;
//    // ...
//    // Please insert system specific function to retrieve a timestamp (in microseconds)
//    // ...
//    return system_current_time;
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    uint64_t microseconds = (current_time.tv_sec * UINT64_C(1000000)) + (current_time.tv_nsec / 1000);

    uint64_t timeUs = microseconds;

    if (lastTimeUS > timeUs) /* An overflow occurred */
    {
        overflowCounter++;
    }
    lastTimeUS = timeUs;

    return timeUs + (overflowCounter * UINT64_C(0xFFFFFFFFFFFFFFFF));
}

void output_ready(int64_t timestamp, float iaq, float static_iaq,
                  float temperature,
                  float raw_temperature, float humidity, float raw_humidity,
                  float pressure, float gas, float gas_percentage,
                  float co2_equivalent,
                  float breath_voc_equivalent, bsec_library_return_t bsec_status,
                  float stabStatus, float runInStatus) {


    printf("\n*************** OUTPUT_READY START ******************* \n\n");

    printf("TIMESTAMP = %d \n", timestamp);
    printf("IAQ ( STATIC ) = %f ( %f ) \n", iaq, static_iaq);
    printf("TEMPERATURE (RAW TEMPERATURE) = %f ( %f )\n", temperature, raw_temperature);
    printf("PRESSURE = %f\n", pressure);
    printf("HUMIDITY (RAW HUMIDITY) = %f ( %f )\n", humidity, raw_humidity);
    printf("GAS ( PERCENTAGE ) = %f ( %f )\n", gas, gas_percentage);
    printf("CO2 EQUIVALENT = %f\n", co2_equivalent);
    printf("BREATH VOC EQUIVALENT = %f\n", breath_voc_equivalent);
    printf("STATUS BSEC | STAB | RUN = %d | %f | %f \n", bsec_status, stabStatus, runInStatus);

    printf("\n*************** OUTPUT_READY END *******************\n");

    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer) {
    printf("state_load functions \n");
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length) {
    printf("state_save \n");
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer) {
    printf("config_load function \n");
    printf("config_load buffer1 %d\n", (int)config_buffer);
    printf("config_load buffer2 %d\n", n_buffer);

    memcpy(config_buffer, Selectivity_config, n_buffer);
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    return n_buffer;
}


/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */




int main() {

    printf("Initializing BME68x starting.. \n");
    return_values_init ret;
    struct bme68x_dev bme_dev;
    memset(&bme_dev, 0, sizeof(bme_dev));

    bme68x_delay_us_fptr_t delayUsFptr = sleep_n;

    bme_dev.chip_id = BME68X_CHIP_ID;
    bme_dev.intf = BME68X_I2C_INTF;
    bme_dev.intf_ptr = (void *) 1; // Pass I2C bus number as a pointer
    bme_dev.read = bus_read;
    bme_dev.write = bus_write;
    bme_dev.delay_us = delayUsFptr;
    bme_dev.amb_temp = 25;


    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep_n, state_load, config_load, bme_dev);
    if (ret.bme68x_status) {
        /* Could not initialize BME68x */
        printf("Error while initializing BME68x: %d \n", ret.bme68x_status);
        return (int) ret.bme68x_status;
    } else if (ret.bsec_status) {
        printf("Error while initializing BSEC library: %d \n", ret.bsec_status);
        /* Could not initialize BSEC library */
        return (int) ret.bsec_status;
    }

    printf("Initializing BSEC status library : %d : %d \n", ret.bme68x_status, ret.bsec_status);

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep_n, get_timestamp_us, output_ready, state_save, 10000);

    return 0;
}

/*! @}*/

