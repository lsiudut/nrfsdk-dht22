#ifndef __DHT22_H__
#define __DHT22_H__

#include <stdbool.h>
#include <string.h>

#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

#include "nrf_delay.h"
#include "nrf_timer.h"

#ifndef CC_CHANNEL
#define CC_CHANNEL NRF_TIMER_CC_CHANNEL0
#endif

/* DHT returns 40 bytes, leaving some emergency space */
#define MAX_BUFFER_SIZE 42
#define DHT_TIMEOUT_TIME 20
#define DHT_LOGIC_1 25

enum {
    DHT_ERROR_NONE,
    DHT_ERROR_CONFIG,
    DHT_ERROR_TIMEOUT,
    DHT_ERROR_CHECKSUM,
    DHT_ERROR_NO_TIMER
};

typedef uint8_t dht_mode;
typedef uint8_t dht_error;
typedef uint8_t dht_buffer[42];

typedef struct dht_config {
    volatile bool in_progress;
    dht_error error;

    nrf_drv_timer_t *timer;
    uint8_t measure_mode;
    uint8_t pin;

    uint8_t t[2];
    uint8_t h[2];
    uint8_t chk;
} dht_config;

void dht_read(dht_config *instance);
void dht_read_wait(dht_config *instance);
bool dht_is_ok(dht_config *instance);
float dht_temperature(dht_config *instance);
float dht_humidity(dht_config *instance);

#endif
