#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led/lp50xx.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/logging/log.h>
#include "u_ubx_protocol.h"

#ifndef PROJECT_H
#define PROJECT_H

#define MASTER 0

/**
 * @brief 64 bit "unique" (random) identifier of the nRF52832
*/
#define UNIQUE_CHIP_ID ((((uint64_t)NRF_FICR->DEVICEADDR[1]) << 32) | ((uint64_t)NRF_FICR->DEVICEADDR[0]))

#endif