#include "project.h"
#include "functionalities.h"

LOG_MODULE_REGISTER(main,LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// device pointers
//const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct gpio_dt_spec uart_ctrl = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, uartctrl_gpios);
const struct gpio_dt_spec en_led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE,en_ledring_gpios);

// // uart
// static uint8_t rx_buf[128] = {0};
// static uint8_t tx_buf[64] = {"hello"};

// static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
// {
// 	switch (evt->type) {
		
// 	case UART_TX_DONE:
//                 LOG_DBG("uart send buffer");
//                 break;
        
//         case UART_RX_RDY:
// 		// do something
//                 int size = evt->data.rx.len;
//                 int offset = evt->data.rx.offset;
//                 for (int i=0; i<size; i++){
//                         rx_buf[i] = evt->data.rx.buf[offset + i];
//                 }
// 		break;
		
// 	case UART_RX_DISABLED:
// 		uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
// 		break;
		
// 	default:
// 		break;
// 	}
// }

int main(void)
{
        struct gps_data gps;
#if MASTER
        struct gps_data slave_gps;
        uint64_t gps_origin;
        int16_t slave_rssi;
        int8_t slave_snr;
#endif
        // if(!device_is_ready(&uart_ctrl)){
        //         LOG_ERR("uart control pin not ready");
        //         return 1;
        // }
        
        int err = gpio_pin_configure_dt(&uart_ctrl, GPIO_OUTPUT_HIGH);
        
        if (!device_is_ready(gps_uart)){
                LOG_ERR("uart not ready");
        }

        k_msleep(1000);
        err = npm1300_init();
        //activate the led drivers
        err = gpio_pin_configure_dt(&en_led,GPIO_OUTPUT_HIGH);
        err = lp5012_init();
        
        // asynch
        // err = uart_callback_set(uart, gps_serial_cb, &gps);
	// if (err) {
	// 	LOG_ERR("uart callback not set");
        //         //return err;
	// }

        // uart_rx_enable(uart, rx_buf, sizeof(rx_buf), 100);


        // interupt driven
	uart_irq_callback_user_data_set(gps_uart, gps_serial_cb, &gps);
	uart_irq_rx_enable(gps_uart);
        k_msleep(100);
        gps_init();
        gps_reset_data(&gps);
        gps_poll(&gps);

        //lora
        k_msleep(500);
        lora_init();
        k_msleep(500);
        

        while(1){
                // err = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
                // if(err){
                //         LOG_ERR("uart could not send, err = %d",err);
                //         return err;
                // }

                // gps_poll(&gps);
                // k_sleep(K_MSEC(1000));
                // gps_reset_data(&gps);
                // k_msleep(100);{
#if MASTER
                lora_receive_data(&slave_gps,&gps_origin,&slave_rssi,&slave_snr);
                k_msleep(100);
#else
                lora_send_data(&gps);
                k_msleep(1000);
#endif

        }

        // int err;

        // err = spi_is_ready_dt(&ism330_spispec);
        // if(!err){
        //         LOG_ERR("Error: SPI device is not ready, err:%d",err);
        //         return 0;
        // }

        // err = spi_is_ready_dt(&lsm303_spispec);
        // if(!err){
        //         LOG_ERR("Error: SPI device is not ready, err:%d",err);
        //         return 0;
        // }

        // //ism_init();
        // k_msleep(100);
        // lsm_init();
        // //k_msleep(100);
        // //struct ism_data ismdata;
        // struct lsm_data lsmdata;
        // int i = 10;

        // while(i>0){
        //         // err = ism_read_outputs(ismdata);
        //         // if(err != 0){
        //         //         LOG_ERR("Error: reading sensor data from ISM330 failed, err: %d",err);
        //         //         return 0;
        //         // }
        //         // k_msleep(100);
        //         err = lsm_read_outputs(lsmdata);
        //         if(err != 0){
        //                 LOG_ERR("Error: reading sensor data from LSM303 failed, err: %d",err);
        //                 return 0;
        //         }
        //         k_msleep(100);
        //         i--;
        // }
        
        return 0;
}
