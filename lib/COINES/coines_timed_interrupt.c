/*!
 * @brief  implemention of the COINES timed interrupt feature using PPI
 * The implementation is kept seperate from the rest of Zephyr COINES implementation,
 * as it uses the Nordic nrfx GPIOTE library, which is not fully compatible with the 
 * Zephyr GPIO library, which is used in the rest of the ZCOINES implementation.
 * It is not expected that this API fuction will be used on the same pins as the other
 * COINES GPIO API functions, in which case it should not cause a problem.
 * NOTE: In case of problems, a pure Zephyr implementation for this API can also be done
 * which will be only marginally less accurate than a PPI/GPIOTE implementation
 */
#include <coines.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include "nrfx_gpiote.h"
#include "common_services.h"


#define TIMER_PRESCALAR                0
#define TIMER_FREQUENCY                16 /*mhz*/
#define TIMER_COUNTER_BITS             32
#define TIMER_TICKS_PER_SECOND         (TIMER_FREQUENCY / (1 + TIMER_PRESCALAR))
#define TIMER_TICKS_TO_USEC(t)         (((uint64_t)t * UINT64_C(1)) / TIMER_TICKS_PER_SECOND)
#define TIMER_TICKS_TO_NSEC(t)         (((uint64_t)t * UINT64_C(1000)) / TIMER_TICKS_PER_SECOND)

/*Timer instance 3 in nrf52 is used by default in Kconfig.coines as it has more 
capture channels (6) than the others. Six is presumed to be the max array size in 
below mappings. This may need to be changed for future architectures*/

static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(CONFIG_COINES_TIMED_INTR_TIMER); 
#define TIMER_IRQ      NRFX_CONCAT_3(TIMER,CONFIG_COINES_TIMED_INTR_TIMER,_IRQn)
#define TIMER_IRQ_HDLR NRFX_CONCAT_3(nrfx_timer_, CONFIG_COINES_TIMED_INTR_TIMER, _irq_handler )

#define MAX_TIMER_CC_CHANNELS NRF_TIMER_CC_CHANNEL_COUNT(CONFIG_COINES_TIMED_INTR_TIMER)
/* Timer cc task lookup table */
static nrf_timer_task_t timer_cc_tasks[] =
{ NRF_TIMER_TASK_CAPTURE0, NRF_TIMER_TASK_CAPTURE1, NRF_TIMER_TASK_CAPTURE2, NRF_TIMER_TASK_CAPTURE3,
  NRF_TIMER_TASK_CAPTURE4, NRF_TIMER_TASK_CAPTURE5 };
static nrf_timer_cc_channel_t timer_cc_channels[] =
{ NRF_TIMER_CC_CHANNEL0,NRF_TIMER_CC_CHANNEL1,NRF_TIMER_CC_CHANNEL2,NRF_TIMER_CC_CHANNEL3,
  NRF_TIMER_CC_CHANNEL4,NRF_TIMER_CC_CHANNEL5 };

/* Timer cc channel allocation array */
/* timer_cc_channel_no -> 0 used for compare event(overflow) */

static bool timer_cc_channel_slots[MAX_TIMER_CC_CHANNELS] = {false};

/* Holds timer overflow count */
static volatile uint32_t timer_overflow_count = 0;

typedef void (*timed_interrupt_cb)(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
struct coines_timed_interrupt_config
{
    nrf_timer_cc_channel_t timer_cc_channel;
    uint8_t ppi_channel;
    timed_interrupt_cb cb;
};

struct coines_timed_interrupt_config timed_interrupt_config[COINES_SHUTTLE_PIN_MAX];

static nrfx_gpiote_in_config_t gpio_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
#define GPIO_PORT(pin)    DT_PROP_BY_PHANDLE_IDX(DT_NODELABEL(pin), gpios, 0, port)
#define GPIO_PIN(pin)     DT_GPIO_PIN_BY_IDX(DT_NODELABEL(pin), gpios, 0)
#define GPIO_ABS_PIN(pin) NRF_GPIO_PIN_MAP(GPIO_PORT(pin), GPIO_PIN(pin))

static uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX] = {
                             [COINES_MINI_SHUTTLE_PIN_1_4] = GPIO_ABS_PIN(shuttle_pin_1_4),
                             [COINES_MINI_SHUTTLE_PIN_1_5] = GPIO_ABS_PIN(shuttle_pin_1_5),
                             [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_ABS_PIN(shuttle_pin_1_6),
                             [COINES_MINI_SHUTTLE_PIN_1_7] = GPIO_ABS_PIN(shuttle_pin_1_7),
                             [COINES_MINI_SHUTTLE_PIN_2_5] = GPIO_ABS_PIN(shuttle_pin_2_5),
                             [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_ABS_PIN(shuttle_pin_2_6),
                             [COINES_MINI_SHUTTLE_PIN_2_1] = GPIO_ABS_PIN(shuttle_pin_2_1),
                             [COINES_MINI_SHUTTLE_PIN_2_3] = GPIO_ABS_PIN(shuttle_pin_2_3),
                             [COINES_APP30_LED_R] = GPIO_ABS_PIN(led_red),
                             [COINES_APP30_LED_G] = GPIO_ABS_PIN(led_green),
                             [COINES_APP30_LED_B] = GPIO_ABS_PIN(led_blue),
                             [COINES_APP30_BUTTON_1] = GPIO_ABS_PIN(button_t1),
                             [COINES_APP30_BUTTON_2] = GPIO_ABS_PIN(button_t2),
                             [COINES_MINI_SHUTTLE_PIN_2_7] = GPIO_ABS_PIN(shuttle_pin_2_7),
                             [COINES_MINI_SHUTTLE_PIN_2_8] = GPIO_ABS_PIN(shuttle_pin_2_8),
                             /*Additionally map AB2.0 pin enums to AB3.0 pins, so that code written using AB2.0
                             pin enums can run on AB3.0*/
                             [COINES_SHUTTLE_PIN_7] = GPIO_ABS_PIN(shuttle_cs),     /*< AB2 CS  = AB3 CS*/
                             [COINES_SHUTTLE_PIN_8] = GPIO_ABS_PIN(shuttle_gpio0),  /*< AB2 Multi-IO 5 =AB3 GPIO0*/
                             [COINES_SHUTTLE_PIN_9] = GPIO_ABS_PIN(shuttle_gpio1),  /*< AB2 Multi-IO 0 = AB3 GPIO1*/
                             [COINES_SHUTTLE_PIN_14] = GPIO_ABS_PIN(shuttle_gpio4_ocsb),/*<AB2  Multi-IO 1 = AB3 GPIO4*/
                             [COINES_SHUTTLE_PIN_15] = GPIO_ABS_PIN(shuttle_gpio5_ascx),/*<AB2  Multi-IO 2 = AB3 GPIO5*/
                             //[COINES_SHUTTLE_PIN_16] = GPIO_ABS_PIN(shuttle_sdi_sda), /*<AB2  Multi-IO 3 = AB3 SDI/SDA TBD????*/
                             [COINES_SHUTTLE_PIN_19] = GPIO_ABS_PIN(shuttle_gpio6_osdo),/*<AB2  Multi-IO 8 = AB3 GPIO6*/
                             [COINES_SHUTTLE_PIN_20] = GPIO_ABS_PIN(shuttle_gpio2_int1),/*<AB2  Multi-IO 6 = AB3 GPIO2/INT1*/
                             [COINES_SHUTTLE_PIN_21] = GPIO_ABS_PIN(shuttle_gpio3_int2),/*<AB2  Multi-IO 7 = GPIO3/INT2*/
                             [COINES_SHUTTLE_PIN_22] = GPIO_ABS_PIN(shuttle_gpio7_asdx),/*<AB2  Multi-IO 4 = AB3 GPIO7*/
                             [COINES_SHUTTLE_PIN_SDO] = GPIO_ABS_PIN(shuttle_sdo),
};

/* Array to map corresponding COINES polarity states to nRF polarity states*/
uint8_t map_nrfpol_to_coinespol[COINES_PIN_INTERRUPT_MODE_MAXIMUM] = {
    0, [NRF_GPIOTE_POLARITY_LOTOHI] = COINES_PIN_INT_POLARITY_LOW_TO_HIGH,
    [NRF_GPIOTE_POLARITY_HITOLO] = COINES_PIN_INT_POLARITY_HIGH_TO_LOW,
    [NRF_GPIOTE_POLARITY_TOGGLE] = COINES_PIN_INT_POLARITY_TOGGLE
};

bool int_pin_usage_native_emulated[COINES_SHUTTLE_PIN_MAX] = { false };
/*!
 * @brief Read COINES multiio index from the NRF pin_number
 */
static enum coines_multi_io_pin get_multiio_pin(uint32_t pin_number)
{
    uint8_t i;

    if (pin_number != 0xff)
    {
        for (i = 0; i < COINES_SHUTTLE_PIN_MAX; i++)
        {
            if ((pin_number == multi_io_map[i]) && int_pin_usage_native_emulated[i])
            {
                return (enum coines_multi_io_pin)i;
            }
        }
    }

    return COINES_SHUTTLE_PIN_MAX;
}
/*!
 * @brief This API is hardware timer interrupt handler which will be used to handle the interrupt events
 */
static void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    (void)p_context;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            timer_overflow_count++;
            break;

        default:
            /*Do nothing. */
            break;
    }
}
/*!
 * @brief  initialization code for timed interrupt feature to be called from coines_open_comm_intf
 */
static bool timed_interrupt_init_done =false;
int timed_interrupt_init()
{
    if(timed_interrupt_init_done)
       return COINES_SUCCESS;
	    /* Timer configuration */
    nrfx_timer_config_t timer_config = {
        .frequency = NRF_TIMER_FREQ_16MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 6, .p_context = NULL
    };
	
	/* Configure hardware timer for capturing timestamp */
    if (NRFX_SUCCESS == nrfx_timer_init(&timer_instance, &timer_config, (nrfx_timer_event_handler_t)timer_handler))
    {

        nrfx_timer_extended_compare(&timer_instance,
                                    NRF_TIMER_CC_CHANNEL0,
                                    0xFFFFFFFF,
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    (bool)true);
        IRQ_DIRECT_CONNECT(TIMER_IRQ, 0, TIMER_IRQ_HDLR, 0);
        irq_enable(TIMER_IRQ);
        nrfx_timer_disable(&timer_instance);
        nrfx_timer_clear(&timer_instance);
        nrfx_timer_enable(&timer_instance);
        timer_cc_channel_slots[0] = true; /*permanently occupy the 0 slot*/
        timed_interrupt_init_done = true;
        nrfx_gpiote_init(7);/*set prio of gpiote lower than timer, so that overflow processing in timer handler
                            is guaranteed to happen before time value is read in the gpiote handler*/
    }
    else
    {
        return COINES_E_TIMER_INIT_FAILED;
    }
	return COINES_SUCCESS;
}

/*!
 * @brief GPIOTE event handler for timed interrupt
 */
static void attach_timed_interrupt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
        uint64_t total_ticks;
        uint32_t latest_ticks_count;
        nrf_timer_cc_channel_t cc_chan;
        enum coines_multi_io_pin pin_number;

        pin_number = get_multiio_pin((uint32_t)pin);

        if (pin_number == COINES_SHUTTLE_PIN_MAX)
        {
            return;
        }
        cc_chan = timed_interrupt_config[pin_number].timer_cc_channel;
        /* Get the captured ticks from the timer */
        nrfx_timer_capture(&timer_instance,cc_chan);
        latest_ticks_count = nrfx_timer_capture_get(&timer_instance,cc_chan);

        total_ticks = ((uint64_t)timer_overflow_count << 32) | latest_ticks_count;

        /* Callback with timestamp */
        timed_interrupt_config[pin_number].cb(TIMER_TICKS_TO_NSEC(total_ticks),
                                            (uint32_t)pin_number,
                                            (uint32_t)map_nrfpol_to_coinespol[action]);

}

/*!
 * @brief Attaches a timed interrupt to a Multi-IO pin
 */
int16_t coines_attach_timed_interrupt(enum coines_multi_io_pin pin_number,
                                      timed_interrupt_cb interrupt_cb,
                                      enum coines_pin_interrupt_mode int_mode)
{
    uint32_t pin_num = multi_io_map[pin_number];
    uint32_t timer_cc_channel_no;
    nrf_ppi_channel_t ppi_channel;
    nrf_timer_cc_channel_t timer_cc_channel=0;

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_INVALID_PIN_NUMBER;
    }
    for(timer_cc_channel_no=1;timer_cc_channel_no < MAX_TIMER_CC_CHANNELS;timer_cc_channel_no++)
    {
        if(!timer_cc_channel_slots[timer_cc_channel_no])
        break;
    }
    if (timer_cc_channel_no >= MAX_TIMER_CC_CHANNELS) /*no more channels left to allocate*/
    {
        return COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE;
    }
    timer_cc_channel = timer_cc_channels[timer_cc_channel_no];
    switch(int_mode)
    {
        case  COINES_PIN_INTERRUPT_CHANGE:   
            gpio_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
            break;
        case COINES_PIN_INTERRUPT_RISING_EDGE:
            gpio_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
            break;
        case COINES_PIN_INTERRUPT_FALLING_EDGE:
        default:
            gpio_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
            break;
    }

    timed_interrupt_init(); /*initialize timer, if not already done previously*/
    if(NRFX_SUCCESS != nrfx_gpiote_in_init(pin_num, &gpio_config, attach_timed_interrupt_handler))
    {
        return COINES_E_CHANNEL_ALLOCATION_FAILED;
    }
    
    nrfx_gpiote_trigger_enable(pin_num,true);
    if (NRFX_SUCCESS != nrfx_ppi_channel_alloc(&ppi_channel))
    {
        return COINES_E_CHANNEL_ALLOCATION_FAILED;
    }
    
    if (NRFX_SUCCESS !=
        nrfx_ppi_channel_assign(ppi_channel, 
            nrfx_gpiote_in_event_addr_get(pin_num),
            nrfx_timer_task_address_get(&timer_instance,
                timer_cc_tasks[timer_cc_channel_no])))
    {
        return COINES_E_CHANNEL_ASSIGN_FAILED;
    }
    if (NRFX_SUCCESS != nrfx_ppi_channel_enable(ppi_channel))
    {
        return COINES_E_CHANNEL_ENABLE_FAILED;
    }

    /* Allocate one cc channel for each timed interrupt config */
    timer_cc_channel_slots[timer_cc_channel_no] = true;
    timed_interrupt_config[pin_number].timer_cc_channel = timer_cc_channel;
    timed_interrupt_config[pin_number].ppi_channel = ppi_channel;
    timed_interrupt_config[pin_number].cb = interrupt_cb;
    int_pin_usage_native_emulated[pin_number] = true;
    return COINES_SUCCESS;

}

/*!
 * @brief Detaches a timed interrupt from a Multi-IO pin
 */
int16_t coines_detach_timed_interrupt(enum coines_multi_io_pin pin_number)
{
    uint8_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_INVALID_PIN_NUMBER;
    }
    uint32_t timer_cc_channel_no = timed_interrupt_config[pin_number].timer_cc_channel;
    /* Cleanup */
    nrfx_gpiote_in_uninit(pin_num);
    nrfx_gpiote_trigger_disable(pin_num);
    if (NRFX_SUCCESS != nrfx_ppi_channel_free(timed_interrupt_config[pin_number].ppi_channel))
        return COINES_E_CHANNEL_DEALLOCATION_FAILED;
    //nrfx_ppi_channel_disable(ppi_channel) not called as free does it automatically
    //free up slot for reallocation
    memset(&timed_interrupt_config[pin_number], 0, sizeof(timed_interrupt_config[pin_number]));
    int_pin_usage_native_emulated[pin_number] = false;
    timer_cc_channel_slots[timer_cc_channel_no] = false;
    return COINES_SUCCESS;

}