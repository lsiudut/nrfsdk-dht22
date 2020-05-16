#include "dht22.h"

#define NRF_LOG_MODULE_NAME DHT22

#if DHT22_LOG_ENABLED
#define NRF_LOG_LEVEL DHT22_LOG_ENABLED
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*
Assigned to DHT instance which is currently running measurement using the TIMER.
Even though there are multiple timers only one measurement at a time is allowed
as multiple measuremets could impact the latency.
*/
static dht_config *active_instance;
static dht_buffer buffer;
static uint8_t buffer_pos;
static uint32_t last_hitolo;

static void dht_timer_stop(dht_config *instance);
static void dht_calculate_from_probes(dht_config *instance, dht_buffer buffer);
static void dht_read_timer_finish();
static void dht_timer_timeout_handler(nrf_timer_event_t event_type, void *p_context);
static void dht_timer_init(dht_config *instance);
static void dht_timer_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void dht_read_timer(dht_config *instance);
static void dht_read_delay(dht_config *instance);

/**
 * @brief Disables timer instance associated with DHT instance
 */
static void dht_timer_stop(dht_config *instance)
{
    nrf_drv_timer_disable(instance->timer);
    nrf_drv_timer_uninit(instance->timer);
}

/**
 * @brief Translate the probes to logical values for given instance
 */
static void dht_calculate_from_probes(dht_config *instance, dht_buffer buffer)
{
    uint8_t *buffptr = buffer + 1;
    uint8_t chk;
    uint8_t cnt;

    for(cnt=0; cnt<8; cnt++)
    {
        instance->h[0] <<= 1;
        instance->h[1] <<= 1;
        instance->t[0] <<= 1;
        instance->t[1] <<= 1;
        instance->chk <<= 1;

        if(*buffptr > DHT_LOGIC_1)
            instance->h[0] |= 1;
        if(*(buffptr+8) > DHT_LOGIC_1)
            instance->h[1] |= 1;
        if(*(buffptr+16) > DHT_LOGIC_1)
            instance->t[0] |= 1;
        if(*(buffptr+24) > DHT_LOGIC_1)
            instance->t[1] |= 1;
        if(*(buffptr+32) > DHT_LOGIC_1)
            instance->chk |= 1;
        buffptr++;
    }

    chk = instance->h[0] + instance->h[1] + instance->t[0] + instance->t[1];

    if(instance->chk != chk)
        instance->error = DHT_ERROR_CHECKSUM;
}

/**
 * @brief Disables timer, unregisters active instance, set in_progress to false
 */
static void dht_read_timer_finish()
{
    if(!active_instance)
        return;

    dht_timer_stop(active_instance);

    nrf_drv_gpiote_in_event_enable(active_instance->pin, false);
    nrf_drv_gpiote_in_uninit(active_instance->pin);

    if(active_instance->error == DHT_ERROR_NONE)
        dht_calculate_from_probes(active_instance, buffer);

    active_instance->in_progress = false;

    active_instance = NULL;
}

/**
 * @brief Handles timeout when using asynchronous measurement method. Triggered by the timer
 * event.
 */
static void dht_timer_timeout_handler(nrf_timer_event_t event_type, void *p_context)
{
    if(!active_instance)
        return;

    NRF_LOG_ERROR("DHT timeout!");

    active_instance->error = DHT_ERROR_TIMEOUT;
    dht_read_timer_finish();
}

/**
 * @brief Configure and enable timer instance associated with DHT instance
 */
static void dht_timer_init(dht_config *instance)
{
    nrfx_timer_config_t timer_config;

    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_16;
    timer_config.frequency = NRF_TIMER_FREQ_250kHz;
    timer_config.interrupt_priority = 3;
    timer_config.mode = NRF_TIMER_MODE_TIMER;

    if(nrf_drv_timer_init(instance->timer, &timer_config, dht_timer_timeout_handler) != NRFX_SUCCESS)
    {
        instance->in_progress = false;
        instance->error = DHT_ERROR_NO_TIMER;
        return;
    }

    nrf_drv_timer_extended_compare(
        instance->timer, CC_CHANNEL, nrf_drv_timer_ms_to_ticks(instance->timer, DHT_TIMEOUT_TIME), NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true
    );

    nrf_drv_timer_enable(instance->timer);
}

/* Yea we should probably disable interrupts here to avoid races */
static void dht_timer_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t timer_cap = nrfx_timer_capture(active_instance->timer, 0);
    buffer[buffer_pos] = timer_cap - last_hitolo;
    last_hitolo = timer_cap;

    NRF_LOG_DEBUG("int %u -> %u", buffer_pos, buffer[buffer_pos]);

    if(buffer_pos++ >= 40)
        dht_read_timer_finish();
}

/**
 * @brief Trigger asynchronous reading using the timer and GPIOTE events.
 */
static void dht_read_timer(dht_config *instance)
{
    nrf_drv_gpiote_in_config_t pin_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(instance->pin, &pin_config, dht_timer_event_handler));
    nrf_drv_gpiote_in_event_enable(instance->pin, true);

    last_hitolo = nrfx_timer_capture(instance->timer, 0);
}

/**
 * @brief Probes DHT using nrf_delay function. It's fully symmetric measurement. Asymetric measuremet
 * is possible if timer is provided to the driver config.
 */
static void dht_read_delay(dht_config *instance)
{
    dht_buffer local_buffer;
    uint8_t local_buff_pos = 0;
    uint32_t cnt1 = 0, cnt;

    nrf_gpio_cfg_input(instance->pin, NRF_GPIO_PIN_NOPULL);

    for(cnt=0; cnt<10000; cnt++)
    {
        if(nrf_gpio_pin_read(instance->pin))
        {
            cnt1++;
        }
        else if(cnt1 > 0)
        {
            NRF_LOG_DEBUG("dly %u -> %u", local_buff_pos, cnt1);
            local_buffer[local_buff_pos++] = cnt1;
            cnt1 = 0;
        }

        if(local_buff_pos >= 40)
            break;

        nrf_delay_us(1);
    }

    if(local_buff_pos < 40)
    {
        instance->error = DHT_ERROR_TIMEOUT;
        instance->in_progress = false;
    }

    dht_calculate_from_probes(instance, local_buffer);

    instance->in_progress = false;

    NRF_LOG_DEBUG("%u samples", local_buff_pos);
}

/**
 * @brief Initializes DHT readout using provided subsystem
 */
void dht_read(dht_config *instance)
{
    instance->in_progress = true;

    /* we want to waste the time for timer initialization after DHT transmission is
       initialized */
    if(instance->timer)
    {
        memset(buffer, 0, sizeof(buffer));
        buffer_pos = 0;
        dht_timer_init(instance);
        active_instance = instance;
    }

    /* initialize readout - low for 1ms, high for ~30-40us */
    nrf_gpio_cfg_output(instance->pin);

    nrf_gpio_pin_set(instance->pin);
    nrf_delay_ms(10);

    nrf_gpio_pin_clear(instance->pin);
    nrf_delay_ms(1);

    nrf_gpio_pin_set(instance->pin);
    nrf_delay_us(30);

    if(instance->timer)
        dht_read_timer(instance);
    else
        dht_read_delay(instance);
}

/**
 * @brief Triggers measurements and waits in the loop for them to finish. Actionable only for asynchronous
 * measurements.
 */
void dht_read_wait(dht_config *instance)
{
    dht_read(instance);
    while(true) {
        if(!instance->in_progress)
            break;
    }
}

/**
 * @brief Returns true if the measurement was successful.
 */
bool dht_is_ok(dht_config *instance)
{
    return instance->error == DHT_ERROR_NONE;
}

/**
 * @brief Reads a temperature from the sensor instance.
 */
float dht_temperature(dht_config *instance)
{
    return (((instance->t[0] & 0x7f) << 8) | instance->t[1]) * 0.1;
}

/**
 * @brief Reads a humidity from the sensor instance.
 */
float dht_humidity(dht_config *instance)
{
    return ((instance->h[0] << 8) | instance->h[1]) * 0.1;
}
