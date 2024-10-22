#include "functionalities.h"

LOG_MODULE_REGISTER(functionalities,LOG_LEVEL_DBG);

/************ sensor: LSM303AHTR ************/
/*
 * lsm_read_reg
 * ---------------------
 * read values from register of lsm303ahtr
 * 
 * reg: register address
 * data: pointer to place to store the read data
 * size: amount of bits to be read
 * 
 * returns: 0 | error code for transceive dt
 */
static int lsm_read_reg(uint8_t reg, uint8_t *data, uint8_t size){
        uint8_t tx_buffer = reg;
	struct spi_buf tx_spi_buf			= {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs 			= {.buf = data, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

	int err = spi_transceive_dt(&lsm303_spispec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	return 0;
}

static int lsm_write_reg(uint8_t reg, uint8_t value){
	uint8_t tx_buf[] = {(reg & 0x7F), value};	
	struct spi_buf 		tx_spi_buf 		= {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set 	tx_spi_buf_set	= {.buffers = &tx_spi_buf, .count = 1};

	int err = spi_write_dt(&lsm303_spispec, &tx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_write_dt() failed, err %d", err);
		return err;
	}
	return 0;
}

void lsm_init(void){
    uint8_t regaddr;
    int err;

    LOG_INF("-------------------------------------------------------------");
    LOG_INF("lsm_init: initializing sensor by writing to the config registers.");

    // config register CTRL1_A
    regaddr = 0x20;
    err = lsm_write_reg(regaddr,CTRL1_A);
    if(err != 0){
        LOG_ERR("could not write to config register CTRL1_A");
    }
    k_msleep(DELAY_PARAM);

    // config register CTRL2_A
    regaddr = 0x21;
    err = lsm_write_reg(regaddr,CTRL2_A);
    if(err != 0){
        LOG_ERR("could not write to config register CTRL2_A");
    }
    k_msleep(DELAY_PARAM);

    // config register CFG_REG_A_M
    regaddr = 0x60;
    err = lsm_write_reg(regaddr,CFG_REG_A_M);
    if(err != 0){
        LOG_ERR("could not write to config register CFG_REG_A_M");
    }
    k_msleep(DELAY_PARAM);

    // config register CFG_REG_B_M
    regaddr = 0x61;
    err = lsm_write_reg(regaddr,CFG_REG_B_M);
    if(err != 0){
        LOG_ERR("could not write to config register CFG_REG_B_M");
    }
    k_msleep(DELAY_PARAM);

    // config register CFG_REG_C_M
    regaddr = 0x62;
    err = lsm_write_reg(regaddr,CFG_REG_C_M);
    if(err != 0){
        LOG_ERR("could not write to config register CFG_REG_C_M");
    }
    k_msleep(DELAY_PARAM);
    LOG_INF("-------------------------------------------------------------");

    return;
}

int lsm_read_outputs(struct lsm_data lsmdata){
	int err;

    int16_t acc_x=0,acc_y=0,acc_z=0,mag_x=0,mag_y=0,mag_z=0;
    int8_t temp=0;

    // registers to read
    uint8_t regs_buffer[14] = {0x26, 0x28, 0x29, 0x2A, 0x2B, 0x2C, \
        0x2D, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D}; // 0xFF dummy
    
    uint8_t readbuf[sizeof(regs_buffer)];

	struct spi_buf 		tx_spi_buf 			= {.buf = (void *)&regs_buffer, .len = sizeof(regs_buffer)};
	struct spi_buf_set 	tx_spi_buf_set		= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf 		rx_spi_bufs			= {.buf = readbuf, .len = sizeof(regs_buffer)};
	struct spi_buf_set 	rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

	err = spi_transceive_dt(&lsm303_spispec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

    temp = readbuf[1];

    acc_x = (readbuf[2] & 0x0F) | (readbuf[3] << 8);
    acc_y = (readbuf[4] & 0x0F) | (readbuf[5] << 8);
    acc_z = (readbuf[6] & 0x0F) | (readbuf[7] << 8);

    mag_x = (readbuf[8] & 0x0F) | (readbuf[9] << 8);
    mag_y = (readbuf[10] & 0x0F) | (readbuf[11] << 8);
    mag_z = (readbuf[12] & 0x0F) | (readbuf[13] << 8);

    lsmdata.out_t_a = temp;
    lsmdata.out_x_a = acc_x;
    lsmdata.out_y_a = acc_y;
    lsmdata.out_z_a = acc_z;
    lsmdata.outx_m = mag_x;
    lsmdata.outy_m = mag_y;
    lsmdata.outz_m = mag_z;

    LOG_INF("\tTemperature: \t %d", temp);
    LOG_INF("\tAccelerometer: \t X:%d, \t Y:%d, \t Z:%d",acc_x,acc_y,acc_z);
    LOG_INF("\tMegnetometer: \t X:%d, \t Y:%d, \t Z:%d",mag_x,mag_y,mag_z);

    return 0;
}


/************ sensor: ISM330DHCXTR ************/

/*
 * ism_read_reg
 * ---------------------
 * read values from register of ism330dhcx
 * 
 * reg: register address
 * data: pointer to place to store the read data
 * size: amount of bits to be read
 * 
 * returns: 0 | error code for transceive dt
 */
static int ism_read_reg(uint8_t reg, uint8_t *data, uint8_t size){
        uint8_t tx_buffer = reg;
	struct spi_buf tx_spi_buf			= {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs 			= {.buf = data, .len = size};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

	int err = spi_transceive_dt(&ism330_spispec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}
	return 0;
}

static int ism_write_reg(uint8_t reg, uint8_t value){
	uint8_t tx_buf[] = {(reg & 0x7F), value};	
	struct spi_buf 		tx_spi_buf 		= {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set 	tx_spi_buf_set	= {.buffers = &tx_spi_buf, .count = 1};

	int err = spi_write_dt(&ism330_spispec, &tx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_write_dt() failed, err %d", err);
		return err;
	}
	return 0;
}

void ism_init(void){
    uint8_t regaddr;
    int err;

    LOG_INF("-------------------------------------------------------------");
    LOG_INF("ism_init: initializing sensor by writing to the config registers.");

    // config register CTRL1_XL
    regaddr = 0x10;
    err = ism_write_reg(regaddr,CTRL1_XL);
    if(err != 0){
        LOG_ERR("could not write to config register CTRL1_XL");
    }
    k_msleep(DELAY_PARAM_ISM);

    // config register CTRL2_G
    regaddr = 0x11;
    err = ism_write_reg(regaddr,CTRL2_G);
    if(err != 0){
        LOG_ERR("could not write to config register CTRL2_G");
    }
    k_msleep(DELAY_PARAM_ISM);
    LOG_INF("-------------------------------------------------------------");

    return;
}

int ism_read_outputs(struct ism_data ismdata){
	int err;

    int16_t acc_x=0,acc_y=0,acc_z=0,gyr_x=0,gyr_y=0,gyr_z=0,temp=0;

    // registers to read
    uint8_t regs_buffer[15] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, \
        0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D}; // 0xFF dummy
    
    uint8_t readbuf[sizeof(regs_buffer)];

	struct spi_buf 		tx_spi_buf 			= {.buf = (void *)&regs_buffer, .len = sizeof(regs_buffer)};
	struct spi_buf_set 	tx_spi_buf_set		= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf 		rx_spi_bufs			= {.buf = readbuf, .len = sizeof(regs_buffer)};
	struct spi_buf_set 	rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

	err = spi_transceive_dt(&ism330_spispec, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
		return err;
	}

    // readbuf[0] is chunk
    temp = (readbuf[1] & 0x0F) | (readbuf[2] << 8);

    acc_x = (readbuf[3] & 0x0F) | (readbuf[4] << 8);
    acc_y = (readbuf[5] & 0x0F) | (readbuf[6] << 8);
    acc_z = (readbuf[7] & 0x0F) | (readbuf[8] << 8);

    gyr_x = (readbuf[9] & 0x0F) | (readbuf[10] << 8);
    gyr_y = (readbuf[11] & 0x0F) | (readbuf[13] << 8);
    gyr_z = (readbuf[13] & 0x0F) | (readbuf[14] << 8);

    ismdata.out_temp = temp;
    ismdata.out_x_a = acc_x;
    ismdata.out_y_a = acc_y;
    ismdata.out_z_a = acc_z;
    ismdata.outx_g = gyr_x;
    ismdata.outy_g = gyr_y;
    ismdata.outz_g = gyr_z;

    LOG_INF("\tTemperature: \t %d", temp);
    LOG_INF("\tAccelerometer: \t X:%d, \t Y:%d, \t Z:%d",acc_x,acc_y,acc_z);
    LOG_INF("\tGyroscope: \t X:%d, \t Y:%d, \t Z:%d",gyr_x,gyr_y,gyr_z);

    return 0;
}