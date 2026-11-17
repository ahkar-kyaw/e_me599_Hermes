#ifndef BOARD_ESP32_NODEMCU_V1_H
#define BOARD_ESP32_NODEMCU_V1_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

/* ISM330DHCX IMU SPI bus. */
#define BOARD_IMU_SPI_HOST               SPI2_HOST
#define BOARD_IMU_SPI_SCLK_GPIO          GPIO_NUM_18
#define BOARD_IMU_SPI_MISO_GPIO          GPIO_NUM_19
#define BOARD_IMU_SPI_MOSI_GPIO          GPIO_NUM_23
#define BOARD_IMU_SPI_CS_GPIO            GPIO_NUM_27
#define BOARD_IMU_SPI_CLOCK_HZ           5000000
#define BOARD_IMU_SPI_MODE               3

/* IMU interrupt pins. Not in use yet. */
#define BOARD_IMU_INT1_GPIO              GPIO_NUM_34
#define BOARD_IMU_INT2_GPIO              GPIO_NUM_35

/* Waveshare ST7789 display on a dedicated SPI bus. */
#define BOARD_DISPLAY_SPI_HOST           SPI3_HOST
#define BOARD_DISPLAY_SPI_SCLK_GPIO      GPIO_NUM_14
#define BOARD_DISPLAY_SPI_MOSI_GPIO      GPIO_NUM_13
#define BOARD_DISPLAY_SPI_CS_GPIO        GPIO_NUM_25
#define BOARD_DISPLAY_DC_GPIO            GPIO_NUM_21
#define BOARD_DISPLAY_RESET_GPIO         GPIO_NUM_22
#define BOARD_DISPLAY_SPI_CLOCK_HZ       20000000
#define BOARD_DISPLAY_SPI_MODE           0

/* ExpressLRS receiver CRSF UART. */
#define BOARD_CRSF_UART_PORT             UART_NUM_2
#define BOARD_CRSF_UART_RX_GPIO          GPIO_NUM_16
#define BOARD_CRSF_UART_TX_GPIO          GPIO_NUM_17
#define BOARD_CRSF_UART_BAUDRATE         420000u
#define BOARD_CRSF_UART_RX_BUFFER_BYTES  512u

/* CubeMars actuator CAN bus through the external CAN transceiver. */
#define BOARD_CAN_TX_GPIO                GPIO_NUM_26
#define BOARD_CAN_RX_GPIO                GPIO_NUM_32

#endif
