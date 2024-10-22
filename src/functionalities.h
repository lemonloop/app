#include "project.h"

#ifndef FUNCTIONALITIES_H
#define FUNCTIONALITIES_H

/************ sensor: LSM303AHTR ************/
// configuration values TODO:change
#define CTRL1_A         0xF0
#define CTRL2_A         0x07
#define CFG_REG_A_M     0x01
#define CFG_REG_B_M     0x04
#define CFG_REG_C_M     0x20

#define DELAY_PARAM     500 //TODO
#define DELAY_REG       100 //TODO

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static struct spi_dt_spec lsm303_spispec = SPI_DT_SPEC_GET(DT_NODELABEL(lsm303agr), SPIOP, 0);

struct lsm_config { // struct to store the sensor configurations
        uint8_t ctrl1_A;
        uint8_t ctrl2_A;
        uint8_t cfg_reg_a_m;
        uint8_t cfg_reg_b_m;
        uint8_t cfg_reg_c_m;
};

struct lsm_data {
        uint8_t out_t_a;
        uint16_t out_x_a;
        uint16_t out_y_a;
        uint16_t out_z_a;
        uint16_t outx_m;
        uint16_t outy_m;
        uint16_t outz_m;
};

static int lsm_read_reg(uint8_t reg, uint8_t *data, uint8_t size);
static int lsm_write_reg(uint8_t reg, uint8_t value);
void lsm_init(void);
int lsm_read_outputs(struct lsm_data lsmdata);

/************ sensor: ISM330DHCXTR ************/
// configuration values TODO:change
#define CTRL1_XL        0x40
#define CTRL2_G         0x50

#define DELAY_PARAM_ISM     500 //TODO
#define DELAY_REG_ISM       100 //TODO

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static struct spi_dt_spec ism330_spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), SPIOP, 0);

struct ism_config { // struct to store the sensor configurations
        uint8_t ctrl1_XL;
        uint8_t ctrl2_G;
};

struct ism_data {
        uint16_t out_temp;
        uint16_t out_x_a;
        uint16_t out_y_a;
        uint16_t out_z_a;
        uint16_t outx_g;
        uint16_t outy_g;
        uint16_t outz_g;
};

static int ism_read_reg(uint8_t reg, uint8_t *data, uint8_t size);
static int ism_write_reg(uint8_t reg, uint8_t value);
void ism_init(void);
int ism_read_outputs(struct ism_data ismdata);

#endif