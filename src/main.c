#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main,LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// device pointers
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
struct spi_dt_spec ism330_spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), SPIOP, 0);
struct spi_dt_spec lsm303_spispec = SPI_DT_SPEC_GET(DT_NODELABEL(lsm303agr), SPIOP, 0);
const struct gpio_dt_spec uart_ctrl = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, uartctrl_gpios);

// sensors
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

//read from a register
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

void lsm_calibrationdata(void){
        return;
}

// uart
static uint8_t rx_buf[10] = {0};
static uint8_t tx_buf[] = {"hello"};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
		
	case UART_TX_DONE:
                LOG_DBG("uart send buffer");
                break;
        
        case UART_RX_RDY:
		// do something
                int size = evt->data.rx.len;
                int offset = evt->data.rx.offset;
                for (int i=0; i<size; i++){
                        rx_buf[i] = evt->data.rx.buf[offset + i];
                }
		break;
		
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
		break;
		
	default:
		break;
	}
}

int main(void)
{
        // if(!device_is_ready(&uart_ctrl)){
        //         LOG_ERR("uart control pin not ready");
        //         return 1;
        // }
        
        int err = gpio_pin_configure_dt(&uart_ctrl, GPIO_OUTPUT_LOW);
        
        if (!device_is_ready(uart)){
                LOG_ERR("uart not ready");
        }
        
        err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		LOG_ERR("uart callback not set");
                //return err;
	}

        uart_rx_enable(uart, rx_buf, sizeof(rx_buf), 100);

        while(1){
                err = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
                if(err){
                        LOG_ERR("uart could not send, err = %d",err);
                        return err;
                }
                k_sleep(K_MSEC(1000));
        }
        
        return 0;
}
