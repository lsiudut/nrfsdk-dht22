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

    dht_config dht22;
    memset(&dht22, 0, sizeof(dht22));
    dht22.timer = &timer;
    dht22.pin = PIN_DHT;

    NRF_LOG_INFO("Starting measurement...");

    while (true)
    {
        while(NRF_LOG_PROCESS() == true);
        dht_read_wait(&dht22);
        NRF_LOG_INFO("T: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_temperature(&dht22)));
        NRF_LOG_INFO("H: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(dht_humidity(&dht22)));
        nrf_delay_ms(1000);
    }
}
