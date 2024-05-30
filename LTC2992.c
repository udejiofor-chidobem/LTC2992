// Chidobem Udejiofor 5/30/2024
// LTC 2992 Basic Functionality Library

#include <smbus.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "LTC2992.h"

float *ltc2992_read_gpio(int ic_addr, const int Pin)
{
    float* out = (float*) malloc(3 * sizeof(float));
    uint16_t sense_data;
    int status = 1;

    int sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, gpio_regs[Pin]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    float sense_voltage = sense_data * GPIO_LSB_12BIT; // Volts
    out[0] = sense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, gpio_max_regs[Pin]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    sense_voltage = sense_data * GPIO_LSB_12BIT; // Volts
    out[1] = sense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, gpio_min_regs[Pin]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    sense_voltage = sense_data * GPIO_LSB_12BIT; // Volts
    out[2] = sense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, gpio_state_regs[0]);
    uint8_t status_data = ((uint8_t) (sense_dat & 0x00FF));
    sense_voltage = (status_data & (1<<(3-Pin)));
    out[3] = sense_voltage ? 1 : 0;

    return out;
}

float *ltc2992_read_sense(uint8_t ic_addr, const int senseNum)
{
    float* out = (float*) malloc(3 * sizeof(float));
    uint16_t sense_data;
    int status = 1;

    int sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, sense_regs[senseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    float sense_voltage = sense_data * SENSE_LSB_12BIT; // Volts
    out[0] = sense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, sense_max_regs[senseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    sense_voltage = sense_data * SENSE_LSB_12BIT; // Volts
    out[1] = sense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, sense_min_regs[senseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    sense_voltage = sense_data * SENSE_LSB_12BIT; // Volts
    out[2] = sense_voltage;
    return out;
}

// To Get Current Draw Divide By Sampling Resistor Value
float *ltc2992_read_dsense(int ic_addr, const int dsenseNum)
{
    float* out = (float*) malloc(3 * sizeof(float));
    uint16_t sense_data;
    int status = 1;

    int sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, dsense_regs[dsenseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    float dsense_voltage = sense_data * DSENSE_LSB_12BIT; // Volts
    out[0] = dsense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, dsense_max_regs[dsenseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    dsense_voltage = sense_data * DSENSE_LSB_12BIT; // Volts
    out[1] = dsense_voltage;

    sense_dat = readSMBUsWord(DEFAULT_PORT, ic_addr, dsense_min_regs[dsenseNum]);
    sense_data = 0x0000;
    sense_data |= ((uint16_t) ((sense_dat & 0x00FF) << 8));
    sense_data |= ((uint16_t) ((sense_dat & 0xFF00) >> 8));
    sense_data >>= 4;
    dsense_voltage = sense_data * DSENSE_LSB_12BIT; // Volts
    out[2] = dsense_voltage;
    return out;
}

// To Get Power Draw Divide By Sampling Resistor Value
// TODO - Replace existing I2C calls with SMBUS based method
float *ltc2992_read_power(int ic_addr, const int powerNum)
{
    float *out = (float *) malloc(3 * sizeof(float));
    uint16_t power_data = 0x00;

    uint16_t address = power_regs[powerNum];
    uint8_t* addr_buffer = (uint8_t *) malloc(2 * sizeof(uint8_t));
    addr_buffer[0] = (uint8_t) ((address & 0xFF00) >> 8);
    addr_buffer[1] = (uint8_t) (address & 0x00FF);
    uint8_t* power_dat = (uint8_t *) malloc(2 * sizeof(uint8_t));
    if (i2c_readMultiByteAddr(DEFAULT_PORT, ic_addr, addr_buffer, 2, power_dat, 3))
    {
        // I2C Failure Handling
        //return -1;
    }
    power_data = 0x00;
    power_data |= ((uint16_t) power_dat[0] << 8);
    power_data |= ((uint16_t) power_dat[1] << 0);
    power_data >>= 8;
    float power_voltage = power_data * POWER_LSB_12BIT; // Volts
    out[0] = power_voltage;

    address = power_max_regs[powerNum];
    addr_buffer [2 * sizeof(uint8_t)];
    addr_buffer[0] = (uint8_t) ((address & 0xFF00) >> 8);
    addr_buffer[1] = (uint8_t) (address & 0x00FF);
    if (i2c_readMultiByteAddr(DEFAULT_PORT, ic_addr, addr_buffer, 2, power_dat, 3))
    {
        // I2C Failure Handling
        //return -1;
    }
    power_data = 0x00;
    power_data |= ((uint16_t) power_dat[0] << 8);
    power_data |= ((uint16_t) power_dat[1] << 0);
    power_data >>= 8;
    power_voltage = power_data * POWER_LSB_12BIT; // Volts
    out[1] = power_voltage;

    address = power_min_regs[powerNum];
    addr_buffer [2 * sizeof(uint8_t)];
    addr_buffer[0] = (uint8_t) ((address & 0xFF00) >> 8);
    addr_buffer[1] = (uint8_t) (address & 0x00FF);
    if (i2c_readMultiByteAddr(DEFAULT_PORT, ic_addr, addr_buffer, 2, power_dat, 3))
    {
        // I2C Failure Handling
        //return -1;
    }
    power_data = 0x00;
    power_data |= ((uint16_t) power_dat[0] << 8);
    power_data |= ((uint16_t) power_dat[1] << 0);
    power_data >>= 8;
    power_voltage = power_data * POWER_LSB_12BIT; // Volts
    out[2] = power_voltage;

    free(addr_buffer);
    free(power_dat);

    return out;
}

// TODO
int8_t ltc2992_write_confg(int ic_addr, uint16_t c_reg, int c_reg_Num)
{
    if(1)
    {
        // I2C Failure Handling
        return -1;
    } 

    return 0;
}

int readSMBUsWord(int i2cbus, int address, int daddress)
{
	char *end;
	int res, size, file;
	char filename[20];
	int pec = 0;
	int flags = 0;
	int force = 0, yes = 0;

    size = I2C_SMBUS_WORD_DATA;

	snprintf(filename, size, "/dev/i2c/%d", i2cbus);
	filename[size - 1] = '\0';

	file = open(filename, O_RDWR);

	if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
		sprintf(filename, "/dev/i2c-%d", i2cbus);
		file = open(filename, O_RDWR);
	}

	if (file < 0) {
		if (errno == ENOENT) {
			fprintf(stderr, "Error: Could not open file "
				"`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
				i2cbus, i2cbus, strerror(ENOENT));
		} else {
			fprintf(stderr, "Error: Could not open file "
				"`%s': %s\n", filename, strerror(errno));
			if (errno == EACCES)
				fprintf(stderr, "Run as root?\n");
		}
	}

    unsigned long funcs;

	if (file < 0
	 || (ioctl(file, I2C_FUNCS, &funcs) < 0) || (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA))
	 || (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0)){
		exit(1);
    }

	if (pec && ioctl(file, I2C_PEC, 1) < 0) {
		close(file);
		exit(1);
	}

    res = i2c_smbus_read_word_data(file, daddress);
	
	close(file);

	if (res < 0) {
		fprintf(stderr, "Error: Read failed\n");
		exit(2);
	}

	//printf("0x%0*x\n", size == I2C_SMBUS_WORD_DATA ? 4 : 2, res);

	return res;
}