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
        int8_t out_t_a;
        int16_t out_x_a;
        int16_t out_y_a;
        int16_t out_z_a;
        int16_t outx_m;
        int16_t outy_m;
        int16_t outz_m;
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
        int16_t out_temp;
        int16_t out_x_a;
        int16_t out_y_a;
        int16_t out_z_a;
        int16_t outx_g;
        int16_t outy_g;
        int16_t outz_g;
};

static int ism_read_reg(uint8_t reg, uint8_t *data, uint8_t size);
static int ism_write_reg(uint8_t reg, uint8_t value);
void ism_init(void);
int ism_read_outputs(struct ism_data ismdata);

/************ U Blox: ZED-F9P-02B ************/
// copy from christian:

#define GPS_RX_BUF_SIZE 128
#define GPS_TX_BUF_SIZE 64
#define UBX_MAX_MSG_IN_BUFFER 3
#define UBX_CLASS_NAV 0x01
#define UBX_MSGID_POSLLH 0x02
#define UBX_MSGID_TIMEUTC 0x21
#define UBX_CLASS_CFG 0x06
#define UBX_MSGID_VALSET 0x8a

static struct device *gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

// use only one buffer
static char gps_rx_uart_buffer[GPS_RX_BUF_SIZE];
static int gps_rx_buf_pos;

// TODO: change attributes
struct gps_data {
   int32_t latitude;
   int32_t longitude;
   int32_t time;
   int16_t horizontal_accuracy;
   uint16_t time_until_lock;
   int32_t sample_count;
};	

void gps_reset_data(struct gps_data *gps);
void gps_serial_cb(const struct device *dev, void *user_data);
void gps_parse_rxbuffer(struct gps_data *gps, char rx_buf[], int32_t buffer_len);
void gps_initialize_module();
void gps_init();
void gps_poll(struct gps_data *gps);

//
#define LORA_PREAMBLE   0b1010110011110000 //2 bytes / 8 bit

// TODO: add information on interrupt
struct lora_payload {
        uint16_t preamble;
        uint64_t origin_device_id;
        struct gps_data ublox_gps_data;
};

static const struct device* lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora_dev));

int is_lora_busy();
int lora_init();
int lora_send_data(struct gps_data *gps);
int lora_receive_data(struct gps_data *gps_slave,uint64_t *origin, int16_t *rssi, int8_t *snr);
int lora_wakeup();

#endif