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
// copy from christian:

#define GPS_RX_BUF_SIZE 128
#define GPS_TX_BUF_SIZE 64
#define UBX_MAX_MSG_IN_BUFFER 3
#define UBX_CLASS_NAV 0x01
#define UBX_MSGID_POSLLH 0x02
#define UBX_MSGID_TIMEUTC 0x21
#define UBX_CLASS_CFG 0x06
#define UBX_MSGID_VALSET 0x8a
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct device *gps_uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

// const struct gpio_dt_spec gps_gpio_reset = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, gpsreset_gpios);
// const struct gpio_dt_spec gps_gpio_int = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, gpsint_gpios);


// have this large buffers only once
static char gps_rx_uart_buffer[GPS_RX_BUF_SIZE];
static int gps_rx_buf_pos;

// init a gps struct
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
	
	if (!uart_irq_update(gps_uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(gps_uart_dev)) {
		uart_fifo_read(gps_uart_dev, &c, 1);

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

//setup the module after cold start correctly
void gps_initialize_module() {
	uint32_t bytes_written_to_buffer;
	uint8_t ubx_request[GPS_TX_BUF_SIZE];
	char tx_buf[GPS_TX_BUF_SIZE];

	//set to ubx mode
	ubx_request[0] = 1; //port ID for UART
	ubx_request[1] = 0; //reserved
	ubx_request[2] = 0; //txReady
	ubx_request[3] = 0;
	ubx_request[4] = 0xC0; //mode 8bit no parity
	ubx_request[5] = 0x08;
	ubx_request[6] = 0;
	ubx_request[7] = 0;
	ubx_request[8] = 0x80; //baudrate 9600
	ubx_request[9] = 0x25; 
	ubx_request[10] = 0; 
	ubx_request[11] = 0; 
	ubx_request[12] = 7; //inProtoMask to ubx
	ubx_request[13] = 0;
	ubx_request[14] = 1; //outProtoMask to ubx
	ubx_request[15] = 0;
	ubx_request[16] = 0; //flags
	ubx_request[17] = 0;
	ubx_request[18] = 0; //reserved
	ubx_request[19] = 0; //reserved
	bytes_written_to_buffer = uUbxProtocolEncode(UBX_CLASS_CFG, UBX_MSGID_VALSET, ubx_request, 20, tx_buf);
	bytes_written_to_buffer += 10;
	//bytes_written_to_buffer = uUbxProtocolEncode(UBX_CLASS_CFG, UBX_MSGID_PRT, NULL, 0, tx_buf);
	
	// //set nav output to uart ubx port
	// ubx_request[0] = UBX_CLASS_NAV; //msgClass
	// ubx_request[1] = UBX_MSGID_POSLLH; //msgID
	// ubx_request[2] = 1; //rate port to 1 (every time generate also publish it)
	// bytes_written_to_buffer += uUbxProtocolEncode(UBX_CLASS_CFG, UBX_MSGID_MSG, ubx_request, UBX_CFG_MSG_MSG_SIZE, tx_buf+bytes_written_to_buffer);	 
	
	for(uint32_t poll_k = 0; poll_k < bytes_written_to_buffer; poll_k++) {
		uart_poll_out(gps_uart_dev,tx_buf[poll_k]);
	}
}

// void gps_task(struct k_msgq *gps_queue, struct k_event *work_trigger) {
// 	uint32_t events;
// 	uint32_t j, poll_k;
// 	char buf_tx[GPS_TX_BUF_SIZE];
// 	int32_t encoded_byte_count;
// 	struct gps_data gps;
	
// 	LOG_INF("Starting gps thread");
	
// 	// Power gating switches
// 	if (!device_is_ready(gps_reg)) {
// 		LOG_INF("GPS power switch initalization failed, not ready");
// 	}

// 	// UART for GPS recording
// 	if (!device_is_ready(gps_uart_dev)) {
// 		LOG_INF("uart device not found!");
// 	}

// 	// Callback function
// 	gps_reset_data(&gps); //init gps stuct
// 	uart_irq_callback_user_data_set(gps_uart_dev, gps_serial_cb, &gps);
// 	uart_irq_rx_enable(gps_uart_dev);
// 	gps_rx_buf_pos = 0;
	
// 	// Set INT and RESET GPIOs correctly
// 	gpio_pin_configure_dt(&gps_gpio_reset, GPIO_OUTPUT);
// 	gpio_pin_configure_dt(&gps_gpio_int, GPIO_OUTPUT);
// 	gpio_pin_set_dt(&gps_gpio_int, 0);
// 	gpio_pin_set_dt(&gps_gpio_reset, 0);

// 	while (1) {
// 		// Wait for an event
// 		events = k_event_wait(work_trigger, EVENT_MEASUREMENT_TRIGGER, true, K_FOREVER);

// 		if (events & EVENT_MEASUREMENT_TRIGGER) {
			
// 			// Clear GPS data
// 			gps_reset_data(&gps);
			
// 			// Enable GPS module
// 			regulator_enable(gps_reg, NULL);

// 			// Reset GPS module
// 			gpio_pin_set_dt(&gps_gpio_reset, 1);
// 			k_sleep(K_MSEC(100));
// 			gpio_pin_set_dt(&gps_gpio_reset, 0);

// 			k_sleep(K_SECONDS(2)); //do not decrease this value, is needed for the ublox module to boot and accept commands!
			
// 			// Configure and initialize GPS module
// 			gps_initialize_module();
			
// 			// encode polling messages once, then use multiple times
// 			// naviation request message (we read the one of the last request, but only few seconds old so no issue)
// 			encoded_byte_count = uUbxProtocolEncode(UBX_CLASS_NAV, UBX_MSGID_POSLLH, NULL, 0, buf_tx);
// 			encoded_byte_count += 16;
// 			// time request message and append in buffer
// 			encoded_byte_count += uUbxProtocolEncode(UBX_CLASS_NAV, UBX_MSGID_TIMEUTC, NULL, 0, buf_tx+encoded_byte_count);	 
			
// 			for (j = 0; j < GPS_SEARCH_SIGNAL_SECONDS; j++) {
// 				k_sleep(K_SECONDS(1));

// 				// send polling requests via uart
// 				for(poll_k = 0; poll_k < encoded_byte_count; poll_k++) {
// 					uart_poll_out(gps_uart_dev, buf_tx[poll_k]);
// 				}
				
// 				//current state, if time stays 0 we do not get any serial messages...
// 				// LOG_INF("lon; %i; lat; %i; time; %i; horAcc; %i", gps.longitude, gps.latitude, gps.time, gps.horizontal_accuracy);
				
// 				// track how long the locking takes. Currently not in use...
// 				gps.time_until_lock++;
// 			}
			
// 			// Check if response from last time is in buffer
// 			gps_parse_rxbuffer(&gps, gps_rx_uart_buffer, GPS_RX_BUF_SIZE); 
			
// 			// Disable GPS receiver
// 			regulator_disable(gps_reg);
				
// 			// Write GPS result to queue, drop if not working..
// 			k_msgq_put(gps_queue, &gps, K_NO_WAIT);
		    	
// 		}
// 	}
	
// }

void gps_init(){
    uint32_t bytes_written_to_buffer;
	// body of the ubx valset message: key id + value (4 bytes + n)
    uint8_t ubx_request[GPS_TX_BUF_SIZE] = {0x40,0x52,0x00,0x01,0x00,0x00,0x96,0x00, // key and value for: bauderate of 38400
                    0x20,0x52,0x00,0x02,0x01, // key and value for: stop bits
                    0x20,0x52,0x00,0x03,0x00, // key and value for: 8 data bits
                    0x20,0x52,0x00,0x04,0x00, // key and value for: 0 parity bits
                    0x10,0x52,0x00,0x05,0x01, // key and value for: 
                    0x10,0x73,0x00,0x01,0x01, // key and value for: input UBX protocol
                    0x10,0x74,0x00,0x01,0x01}; // key and value for: output UBX protocol
	char tx_buf[GPS_TX_BUF_SIZE];

    bytes_written_to_buffer = uUbxProtocolEncode(UBX_CLASS_CFG, UBX_MSGID_VALSET, ubx_request, 38, tx_buf);

    for(uint32_t poll_k = 0; poll_k < bytes_written_to_buffer; poll_k++) {
		uart_poll_out(gps_uart_dev,tx_buf[poll_k]);
	}

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
			uart_poll_out(gps_uart_dev, buf_tx[poll_k]);
		}
		
		//current state, if time stays 0 we do not get any serial messages...
		LOG_INF("lon; %i; lat; %i; time; %i; horAcc; %i", gps->longitude, gps->latitude, gps->time, gps->horizontal_accuracy);
				
		// track how long the locking takes. Currently not in use...
		gps->time_until_lock++;
 	}

    // Check if response from last time is in buffer
	gps_parse_rxbuffer(&gps, gps_rx_uart_buffer, GPS_RX_BUF_SIZE); 

    
    return;
}



/************ LED-Ring and Power manager: nPM1300-QEAA ************/
