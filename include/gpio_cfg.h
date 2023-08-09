#pragma once

#define M1_PWM_A 0
#define M1_PWM_B 1
#define M2_PWM_A 2
#define M2_PWM_B 3
#define WIRE0_SDA_ 4
#define WIRE0_SCL_ 5
#define M1_ENC_B 6
#define M1_ENC_A 7
#define M2_ENC_B 8
#define M2_ENC_A 9
#define POWER_BTN 10
#define SHUTDOWN 11
#define UROS_SERIAL_TX_ 12
#define UROS_SERIAL_RX_ 13
#define RPI_RUN 14
#define RPI_G_EN 15
#define VACUUM_EN 16

struct PinModeInfo
{
  uint8_t gpio;
  uint8_t mode;
};

struct PinFncInfo
{
  uint8_t gpio;
  gpio_function fnc;
};

const PinModeInfo pin_map_gpio[] = {
  {LED_BUILTIN, OUTPUT},
  {M1_PWM_A, OUTPUT},
  {M1_PWM_B, OUTPUT},
  {M2_PWM_A, OUTPUT},
  {M2_PWM_B, OUTPUT},
  {M1_ENC_A, INPUT},
  {M1_ENC_B, INPUT},
  {M2_ENC_A, INPUT},
  {M2_ENC_B, INPUT},
  {SHUTDOWN, OUTPUT},
  {RPI_RUN, INPUT},
  {RPI_G_EN, OUTPUT},
  {VACUUM_EN, OUTPUT},
  // it doesn't work with pinMode and interrupt attached, but POWER_BTN pin is pulled up anyway
  // {POWER_BTN, INPUT_PULLUP},  
};

const PinFncInfo pin_map_fnc_gpio[] = { 
  {WIRE0_SDA_, GPIO_FUNC_I2C},
  {WIRE0_SCL_, GPIO_FUNC_I2C},
  {UROS_SERIAL_TX_, GPIO_FUNC_UART},
  {UROS_SERIAL_RX_, GPIO_FUNC_UART},
};
