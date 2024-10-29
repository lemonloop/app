#include "functionalities.h"

LOG_MODULE_REGISTER(functionalities,LOG_LEVEL_DBG);

/************ sensor: LSM303AHTR ************/

/*
 * lsm_read_reg
 * -------------
 * read values from register of lsm303ahtr
 * 
 * reg: register address
 * data: pointer to place to store the read data
 * size: amount of bits to be read
 * 
 * returns: 0 / error code of spi_transceive_dt()
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

/************ U Blox: ZED-F9P-02B ************/

#define GPS_RX_BUF_SIZE 128
#define GPS_TX_BUF_SIZE 64
#define UBX_MAX_MSG_IN_BUFFER 3
#define UBX_CLASS_NAV 0x01
#define UBX_MSGID_POSLLH 0x02
#define UBX_MSGID_TIMEUTC 0x21
#define UBX_CLASS_CFG 0x06
#define UBX_MSGID_VALSET 0x8a

const struct device *gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

// ***** from Christian *****
// use only one buffer
static char gps_rx_uart_buffer[GPS_RX_BUF_SIZE];
static int gps_rx_buf_pos;

// reset the gps data
void gps_reset_data(struct gps_data *gps) {
	gps->time = 0;
	gps->longitude = 0;
	gps->latitude = 0;
	gps->horizontal_accuracy = 0;
	gps->sample_count = 0;
	gps->time_until_lock = 0;
}

//serial callback from uart rx interrupt, user_data contains the pointer to the gps struct
void gps_serial_cb(const struct device *dev, void *user_data) {
	uint8_t c;
	
	if (!uart_irq_update(gps_uart)) {
		return;
	}

	while (uart_irq_rx_ready(gps_uart)) {
		uart_fifo_read(gps_uart, &c, 1);

		if (gps_rx_buf_pos < (GPS_RX_BUF_SIZE - 1)) {
			gps_rx_uart_buffer[gps_rx_buf_pos++] = c;
		} else {
			gps_rx_buf_pos = 0;

			LOG_HEXDUMP_INF(gps_rx_uart_buffer, GPS_RX_BUF_SIZE, "rx_buf");
			gps_parse_rxbuffer(user_data, gps_rx_uart_buffer, GPS_RX_BUF_SIZE); //parse if buffer is full
		}
	}
}

//parsing recorded rx buffer and write to gps struct if valid data, needs pointer to buffer to read
void gps_parse_rxbuffer(struct gps_data *gps, char rx_buf[], int32_t buffer_len) {
	char ubx_nav_response[GPS_RX_BUF_SIZE];
	const char *next_ubx_msg_position;
	int32_t decoded_byte_count;
	int32_t decoded_messageid;
	int32_t decoded_classid;
	int32_t temp_horz_accuracy;
	int32_t k;
	
	//first message in the buffer, set pointer correctly
	next_ubx_msg_position = rx_buf;
	
	//decode the ubx protocol message. (ret positive if decoded with number of bytes of response) 
	for (k = 0; k < UBX_MAX_MSG_IN_BUFFER; k++) {        
		decoded_classid = 0;
		decoded_messageid = 0;  	
		decoded_byte_count = uUbxProtocolDecode(next_ubx_msg_position, buffer_len, &decoded_classid, &decoded_messageid, ubx_nav_response, GPS_RX_BUF_SIZE, &next_ubx_msg_position);

        // for debugging
        LOG_DBG("decoded byte count: %d, decoded_classid: %d, decoded_messageid: %d",decoded_byte_count,decoded_classid,decoded_messageid);
        LOG_HEXDUMP_INF(ubx_nav_response, GPS_RX_BUF_SIZE, "ubx_nav_response");

		if (decoded_byte_count > 0) {
			//sane NAV_POSLLH response from module
			if (decoded_byte_count == 28 && decoded_classid == UBX_CLASS_NAV && decoded_messageid == UBX_MSGID_POSLLH) {
				gps->longitude = (ubx_nav_response[7] << 24) | (ubx_nav_response[6] << 16) | (ubx_nav_response[5] << 8) | (ubx_nav_response[4]);
				gps->latitude = (ubx_nav_response[11] << 24) | (ubx_nav_response[10] << 16) | (ubx_nav_response[9] << 8) | (ubx_nav_response[8]);
				gps->horizontal_accuracy = 0;
				temp_horz_accuracy = 0;
				temp_horz_accuracy = (ubx_nav_response[23] << 24) | (ubx_nav_response[22] << 16) | (ubx_nav_response[21] << 8) | (ubx_nav_response[20]);
				
				//change to meter and limit to reasonable values
				if (temp_horz_accuracy > 0) {
					temp_horz_accuracy /= 1000; //now unit meter
					if (temp_horz_accuracy > 0x7FFF) { 
						gps->horizontal_accuracy = 0x7FFF;
					} else {
						gps->horizontal_accuracy =  (int16_t)(temp_horz_accuracy);
					}
				} else {
					gps->horizontal_accuracy = (int16_t) temp_horz_accuracy;
				}

				//LOG_INF("got lon %i, lat %i", gps.longitude, gps.latitude);
				if ((gps->longitude != 0) && (gps->latitude != 0)) {
					gps->sample_count++;
				}
				
			}
			//sane NAV_TIMEUTC from module
			if (decoded_byte_count == 20 && decoded_classid == UBX_CLASS_NAV && decoded_messageid == UBX_MSGID_TIMEUTC) {
				gps->time = ubx_nav_response[16]*10000 + ubx_nav_response[17]*100 + ubx_nav_response[18];
			}
		} else {
			break; //nothing in this buffer
		}
	}
}
// ***** end of Christians part *******

void gps_init(){
    uint32_t bytes_written_to_buffer;

    // already the default values, uncomment and send if the module is no longer in default config.
    // uint8_t ubx_request[GPS_TX_BUF_SIZE] = {0x00,0x07,0x00,0x00, // first 4 bytes of message
    //                 0x40,0x52,0x00,0x01,0x00,0x00,0x96,0x00, // key and value for: bauderate of 38400
    //                 0x20,0x52,0x00,0x02,0x01, // key and value for: stop bits
    //                 0x20,0x52,0x00,0x03,0x00, // key and value for: 8 data bits
    //                 0x20,0x52,0x00,0x04,0x00, // key and value for: 0 parity bits
    //                 0x10,0x52,0x00,0x05,0x01, // key and value for: 
    //                 0x10,0x73,0x00,0x01,0x01, // key and value for: input UBX protocol
    //                 0x10,0x74,0x00,0x01,0x01}; // key and value for: output UBX protocol
    // char tx_buf[GPS_TX_BUF_SIZE];
    // bytes_written_to_buffer = uUbxProtocolEncode(UBX_CLASS_CFG, UBX_MSGID_VALSET, ubx_request, 42, tx_buf);
    // for(uint32_t poll_k = 0; poll_k < bytes_written_to_buffer; poll_k++) {
	// 	uart_poll_out(gps_uart,tx_buf[poll_k]);
	// }
    
    // payload for UBX_CFG_VALSET for CFG_MSGOUT:
    uint8_t ubx_request_poll[GPS_TX_BUF_SIZE] = 
                    {0x00,0x07,0x00,0x00, //save the configuration in all three layers of memory storage
                    0x2a,0x00,0x91,0x20,0x01, // key and value for: enable nav posllh at uart 1
                    0x5c,0x00,0x91,0x20,0x01, // key and value for: enable nav utc at uart 1
                    0xb1,0x00,0x91,0x20,0x00, // key and value for: disable nmea vtg at uart 1
                    0xac,0x00,0x91,0x20,0x00, // key and value for: disable nmea rmc at uart 1
                    0xc5,0x00,0x91,0x20,0x00, // key and value for: disable nmea gsv at uart 1
                    0xc0,0x00,0x91,0x20,0x00, // key and value for: disable nmea gsa at uart 1
                    0xca,0x00,0x91,0x20,0x00, // key and value for: disable nmea gll at uart 1
                    0xbb,0x00,0x91,0x20,0x00}; // key and value for: disable nmea gga at uart 1

	
    char tx_buf_poll[GPS_TX_BUF_SIZE];
    
    // encode the payload as a whole ubx message
    bytes_written_to_buffer = uUbxProtocolEncode(UBX_CLASS_CFG,UBX_MSGID_VALSET,ubx_request_poll,44,tx_buf_poll);
    // send message over uart
    for(uint32_t poll_k = 0; poll_k < 52; poll_k++) {
		uart_poll_out(gps_uart,tx_buf_poll[poll_k]);
	}
    // could possibly at a wait for UBX_ACK_ACK (length 2, Class 0x05, ID 0x01)
    return;
}

void gps_poll(struct gps_data *gps){
	char buf_tx[GPS_TX_BUF_SIZE];
	int32_t encoded_byte_count;
    
    // encode polling messages once, then use multiple times
	// naviation request message (we read the one of the last request, but only few seconds old so no issue)
	encoded_byte_count = uUbxProtocolEncode(UBX_CLASS_NAV, UBX_MSGID_POSLLH, NULL, 0, buf_tx);
	//encoded_byte_count += 16;
	// time request message and append in buffer
	//encoded_byte_count += uUbxProtocolEncode(UBX_CLASS_NAV, UBX_MSGID_TIMEUTC, NULL, 0, buf_tx+encoded_byte_count);

    for (uint32_t j = 0; j < 240; j++) {
		k_sleep(K_SECONDS(1));

		// send polling requests via uart
		for(uint32_t poll_k = 0; poll_k < encoded_byte_count; poll_k++) {
			uart_poll_out(gps_uart, buf_tx[poll_k]);
		}
		
		//current state, if time stays 0 we do not get any serial messages...
		LOG_INF("lon; %i; lat; %i; time; %i; horAcc; %i", gps->longitude, gps->latitude, gps->time, gps->horizontal_accuracy);
				
		// track how long the locking takes. Currently not in use...
		gps->time_until_lock++;
 	}

    // LOG_INF("lon; %i; lat; %i; time; %i; horAcc; %i", gps->longitude, gps->latitude, gps->time, gps->horizontal_accuracy);

    // Check if response from last time is in buffer
	gps_parse_rxbuffer(&gps, gps_rx_uart_buffer, GPS_RX_BUF_SIZE); 

    
    return;
}



/************ LED-Ring and Power manager: nPM1300-QEAA ************/
//bootup: turkies ring

//when received an interrupt: red blinking 3times

//point to slave if possible