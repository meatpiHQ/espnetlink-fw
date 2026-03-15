#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

// GPIO Pin Definitions
#define LED_GPIO               GPIO_NUM_2
#define USB_SEL_1              GPIO_NUM_12
#define USB_SEL_2              GPIO_NUM_10
#define WAKE_GPIO              GPIO_NUM_1

// GPS
#define GPS_UART_PORT          UART_NUM_2
#define GPS_UART_BAUD_RATE     115200
#define GPS_UART_TX_PIN        GPIO_NUM_42
#define GPS_UART_RX_PIN        GPIO_NUM_8
#define GPS_PWR_EN_PIN         GPIO_NUM_40

// LTE Module
#define LTE_UART_PORT          UART_NUM_1
#define LTE_UART_BAUD_RATE     3000000
#define LTE_UART_TX_PIN        GPIO_NUM_17
#define LTE_UART_RX_PIN        GPIO_NUM_18
#define LTE_DTR_PIN            GPIO_NUM_5
#define LTE_RTS_PIN            GPIO_NUM_7
#define LTE_CTS_PIN            GPIO_NUM_6
#define LTE_DCD_PIN            GPIO_NUM_4
#define LTE_RI_PIN             GPIO_NUM_9
#define LTE_PWR_EN_PIN         GPIO_NUM_39  //Do not use this pin
#define LTE_PWR_KEY_PIN        GPIO_NUM_13
#define LTE_STATUS_PIN         GPIO_NUM_16  //Low when module is ON
#define LTE_RESET_PIN          -1 //GPIO_NUM_14

// LED Configuration
#define LED_RED_PIN           GPIO_NUM_41

// I2C Configuration
#define I2C_MASTER_SCL_IO     GPIO_NUM_47
#define I2C_MASTER_SDA_IO     GPIO_NUM_48

#define IMU_INT_PIN           GPIO_NUM_21
#endif // HW_CONFIG_H
