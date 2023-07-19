#include <Arduino.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#define micro_rollover_useconds 4294967295

int clock_gettime(clockid_t unused, struct timespec* tp)
{
  (void)unused;
  static uint32_t rollover = 0;
  static uint32_t last_measure = 0;

  uint32_t m = micros();
  rollover += (m < last_measure) ? 1 : 0;

  uint64_t real_us = (uint64_t)(m + rollover * micro_rollover_useconds);
  tp->tv_sec = real_us / 1000000;
  tp->tv_nsec = (real_us % 1000000) * 1000;
  last_measure = m;

  return 0;
}

bool pico_transport_open(struct uxrCustomTransport* transport)
{
  // gpio_set_function(16, GPIO_FUNC_UART);
  // gpio_set_function(17, GPIO_FUNC_UART);

  Serial1.begin(576000);
  delay(800);
  return true;
}

bool pico_transport_close(struct uxrCustomTransport* transport)
{
  Serial1.end();
  return true;
}

size_t pico_transport_write(
  struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode)
{
  (void)errcode;
  size_t sent = Serial1.write(buf, len);

  // busy_wait_us(50);

  return sent;
}

size_t pico_transport_read(
  struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
  (void)errcode;
  Serial1.setTimeout(timeout);

  busy_wait_us(50);

  return Serial1.readBytes((char*)buf, len);
}