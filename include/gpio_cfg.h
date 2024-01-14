#pragma once

#define M1_ENC_A 7
#define M1_ENC_B 6
#define M2_ENC_A 2
#define M2_ENC_B 3
#define M1_PWM_A 12
#define M1_PWM_B 13
#define M2_PWM_A 14
#define M2_PWM_B 15
#define WIRE0_SDA_ 4
#define WIRE0_SCL_ 5
#define UROS_SERIAL_TX_ 8
#define UROS_SERIAL_RX_ 9

// Unsupported pins (yet)
#define LED_D 1
#define VACUUM_EN 18
#define SHUTDOWN 19
#define POWER_BTN 20
#define RPI_RUN 22
#define RPI_SHUTDOWN 21
#define ENDSPOT_1 16
#define ENDSPOT_2 17
#define ENDSPOT_3 10
#define ENDSPOT_4 11
#define ADC0 26
#define ADC1 27
#define ADC2 28

struct GPIOPinDirection
{
  pin_size_t pin_number;
  int direction;
};

struct GPIOPinFunction
{
  pin_size_t pin_number;
  gpio_function function;
};

const GPIOPinDirection pin_directions[] = {
  {LED_BUILTIN, OUTPUT}, {M1_PWM_A, OUTPUT}, {M1_PWM_B, OUTPUT}, {M2_PWM_A, OUTPUT},
  {M2_PWM_B, OUTPUT},    {M1_ENC_A, INPUT},  {M1_ENC_B, INPUT},  {M2_ENC_A, INPUT},
  {M2_ENC_B, INPUT},     {SHUTDOWN, OUTPUT}, {RPI_RUN, INPUT},   {RPI_SHUTDOWN, OUTPUT},
  {VACUUM_EN, OUTPUT},
  // it doesn't work with pinMode and interrupt attached, but POWER_BTN pin is pulled up anyway
  // {POWER_BTN, INPUT_PULLUP},
};

const GPIOPinFunction pin_functions[] = {
  {WIRE0_SDA_, GPIO_FUNC_I2C},
  {WIRE0_SCL_, GPIO_FUNC_I2C},
  {UROS_SERIAL_TX_, GPIO_FUNC_UART},
  {UROS_SERIAL_RX_, GPIO_FUNC_UART},
};