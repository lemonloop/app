#include "project.h"
#include "functionalities.h"

LOG_MODULE_REGISTER(main,LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// device pointers
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct gpio_dt_spec uart_ctrl = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, uartctrl_gpios);

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
        
        // int err = gpio_pin_configure_dt(&uart_ctrl, GPIO_OUTPUT_LOW);
        
        // if (!device_is_ready(uart)){
        //         LOG_ERR("uart not ready");
        // }
        
        // err = uart_callback_set(uart, uart_cb, NULL);
	// if (err) {
	// 	LOG_ERR("uart callback not set");
        //         //return err;
	// }

        // uart_rx_enable(uart, rx_buf, sizeof(rx_buf), 100);

        // while(1){
        //         err = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
        //         if(err){
        //                 LOG_ERR("uart could not send, err = %d",err);
        //                 return err;
        //         }
        //         k_sleep(K_MSEC(1000));
        // }

        int err;

        err = spi_is_ready_dt(&ism330_spispec);
        if(!err){
                LOG_ERR("Error: SPI device is not ready, err:%d",err);
                return 0;
        }

        ism_init();
        struct ism_data ismdata;
        int i = 100;

        while(i>0){
                err = ism_read_outputs(ismdata);
                if(err != 0){
                        LOG_ERR("Error: reading sensor data from LSM303 failed, err: %d",err);
                        return 0;
                }
                i--;
        }
        
        return 0;
}
