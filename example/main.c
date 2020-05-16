#include <assert.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"
#include "boards.h"

#include "nrf_timer.h"

#include "dht22.h"

#define TIMER_INSTANCE 2
#define PIN_DHT 2

static nrf_drv_timer_t timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE);

int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);

    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    dht_config dht22_timer;
    memset(&dht22_timer, 0, sizeof(dht22_timer));
    dht22_timer.timer = &timer;
    dht22_timer.pin = PIN_DHT;

    dht_config dht22_delay;
    memset(&dht22_delay, 0, sizeof(dht22_delay));
    dht22_delay.pin = PIN_DHT;

    NRF_LOG_INFO("Starting measurement...");

    while (true)
    {
        dht_read_wait(&dht22_timer);
        NRF_LOG_INFO("TIMER T: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_temperature(&dht22_timer)));
        NRF_LOG_INFO("TIMER H: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_humidity(&dht22_timer)));
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);

        dht_read_wait(&dht22_delay);
        NRF_LOG_INFO("DELAY T: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_temperature(&dht22_delay)));
        NRF_LOG_INFO("DELAY H: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_humidity(&dht22_delay)));
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);
    }
}
