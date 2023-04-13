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
/*Timer instance 3 is used for timestamp as we need more number of capture channels*/
static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(3);

#define MAX_TIMER_CC_CHANNELS 6
/* Timer cc channel lookup table */
static uint32_t timer_cc_channel[MAX_TIMER_CC_CHANNELS] =
{ NRF_TIMER_TASK_CAPTURE0, NRF_TIMER_TASK_CAPTURE1, NRF_TIMER_TASK_CAPTURE2, NRF_TIMER_TASK_CAPTURE3,
  NRF_TIMER_TASK_CAPTURE4, NRF_TIMER_TASK_CAPTURE5 };

/* Timer cc channel allocation array */
/* timer_cc_channel_no -> 0 used for compare event(overflow) */

static bool timer_cc_channel_slots[MAX_TIMER_CC_CHANNELS] = {false};

/* Holds timer overflow count */
static volatile uint32_t timer_overflow_count = 0;

typedef void (*timed_interrupt_cb)(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
struct coines_timed_interrupt_config
{
    uint8_t timer_cc_channel;
    uint8_t ppi_channel;
    timed_interrupt_cb cb;
};

struct coines_timed_interrupt_config timed_interrupt_config[COINES_SHUTTLE_PIN_MAX];

static nrfx_gpiote_in_config_t gpio_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);

#define  GPIO_0                       NRF_GPIO_PIN_MAP(0, 14) /* SB1_4 - P0.14 (I2C1_SCL) */
#define  GPIO_1                       NRF_GPIO_PIN_MAP(0, 13) /* SB1_5 - P0.13 (I2C1_SDA) */
#define  GPIO_2                       NRF_GPIO_PIN_MAP(1, 1) /* INT1 - SB1_6 - P1.01 */
#define  GPIO_3                       NRF_GPIO_PIN_MAP(1, 8) /* INT2 - SB1_7 - P1.08 */
#define  GPIO_CS                      NRF_GPIO_PIN_MAP(0, 24) /* SB2_1 - P0.24 */
#define  GPIO_SDO                     NRF_GPIO_PIN_MAP(0, 15) /* SB2_3 - P0.15*/
#define  GPIO_4                       NRF_GPIO_PIN_MAP(1, 3) /* SB2_5 - P1.03 */
#define  GPIO_5                       NRF_GPIO_PIN_MAP(1, 2) /* SB2_6 - P1.02 */
#define  GPIO_6                       NRF_GPIO_PIN_MAP(1, 11) /* SB2_7 - P1.11 */
#define  GPIO_7                       NRF_GPIO_PIN_MAP(1, 10) /* SB2_8 - P1.10 */
#define  GPIO_SDI                     NRF_GPIO_PIN_MAP(0, 6) /* SB2_4 - P0.6*/
#define  GPIO_SCK                     NRF_GPIO_PIN_MAP(0, 16) /* SB2_2 - P0.16*/
#define MCU_LED_R                     NRF_GPIO_PIN_MAP(0, 7)
#define MCU_LED_G                     NRF_GPIO_PIN_MAP(0, 11)
#define MCU_LED_B                     NRF_GPIO_PIN_MAP(0, 12)
#define SWITCH1                       NRF_GPIO_PIN_MAP(1, 9)
#define SWITCH2                       NRF_GPIO_PIN_MAP(0, 25)
uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX] = {
    [COINES_SHUTTLE_PIN_9] = GPIO_1, [COINES_SHUTTLE_PIN_14] = GPIO_4, [COINES_SHUTTLE_PIN_15] = GPIO_5,
    [COINES_SHUTTLE_PIN_16] = GPIO_SDI, [COINES_SHUTTLE_PIN_22] = GPIO_7, [COINES_SHUTTLE_PIN_8] = GPIO_0,
    [COINES_SHUTTLE_PIN_20] = GPIO_2, [COINES_SHUTTLE_PIN_21] = GPIO_3, [COINES_SHUTTLE_PIN_19] = GPIO_6,
    [COINES_SHUTTLE_PIN_7] = GPIO_CS, 0, 0, 0, 0, 0, 0,

    /* Native APP3.0 pins */
    [COINES_MINI_SHUTTLE_PIN_1_4] = GPIO_0, [COINES_MINI_SHUTTLE_PIN_1_5] = GPIO_1,
    [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_2, [COINES_MINI_SHUTTLE_PIN_1_7] = GPIO_3,
    [COINES_MINI_SHUTTLE_PIN_2_5] = GPIO_4, [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_5,
    [COINES_MINI_SHUTTLE_PIN_2_1] = GPIO_CS, [COINES_MINI_SHUTTLE_PIN_2_3] = GPIO_SDO, [COINES_APP30_LED_R] = MCU_LED_R,
    [COINES_APP30_LED_G] = MCU_LED_G, [COINES_APP30_LED_B] = MCU_LED_B, [COINES_APP30_BUTTON_1] = SWITCH1,
    [COINES_APP30_BUTTON_2] = SWITCH2, [COINES_MINI_SHUTTLE_PIN_2_7] = GPIO_6, [COINES_MINI_SHUTTLE_PIN_2_8] = GPIO_7,
    [COINES_SHUTTLE_PIN_SDO] = GPIO_SDO
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
int timed_interrupt_init()
{
	    /* Timer configuration */
    nrfx_timer_config_t timer_config = {
        .frequency = NRF_TIMER_FREQ_16MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, .p_context = NULL
    };
	
	/* Configure hardware timer for capturing timestamp */
    if (NRFX_SUCCESS == nrfx_timer_init(&timer_instance, &timer_config, (nrfx_timer_event_handler_t)timer_handler))
    {

        nrfx_timer_extended_compare(&timer_instance,
                                    NRF_TIMER_CC_CHANNEL0,
                                    0xFFFFFFFF,
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    (bool)true);

        nrfx_timer_enable(&timer_instance);
        timer_cc_channel_slots[0] = true; //permanently occupy the 0 slot
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

        enum coines_multi_io_pin pin_number;

        pin_number = get_multiio_pin((uint32_t)pin);

        if (pin_number == COINES_SHUTTLE_PIN_MAX)
        {
            return;
        }

        /* Get the captured ticks from the timer */
        latest_ticks_count =
            nrfx_timer_capture_get(&timer_instance,
                                (nrf_timer_cc_channel_t)timed_interrupt_config[pin_number].timer_cc_channel);

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

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_INVALID_PIN_NUMBER;
    }
    for(timer_cc_channel_no=1;timer_cc_channel_no < MAX_TIMER_CC_CHANNELS;timer_cc_channel_no++)
    {
        if(!timer_cc_channel_slots[timer_cc_channel_no])
        break;
    }
    if (timer_cc_channel_no >= MAX_TIMER_CC_CHANNELS) //no more channels left to allocate
    {
        return COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE;
    }

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

    if(NRFX_SUCCESS != nrfx_gpiote_in_init(pin_num, &gpio_config, attach_timed_interrupt_handler))
    {
        return COINES_E_CHANNEL_ALLOCATION_FAILED;
    }
    nrfx_gpiote_in_event_enable(pin_num, true);

    if (NRFX_SUCCESS != nrfx_ppi_channel_alloc(&ppi_channel))
    {
        return COINES_E_CHANNEL_ALLOCATION_FAILED;
    }
    
    if (NRFX_SUCCESS !=
        nrfx_ppi_channel_assign(ppi_channel, 
            nrfx_gpiote_in_event_addr_get(pin_num),
            nrfx_timer_task_address_get(&timer_instance,
            (nrf_timer_task_t)timer_cc_channel[timer_cc_channel_no])))
    {
        return COINES_E_CHANNEL_ASSIGN_FAILED;
    }
    if (NRFX_SUCCESS != nrfx_ppi_channel_enable(ppi_channel))
    {
        return COINES_E_CHANNEL_ENABLE_FAILED;
    }

    /* Allocate one cc channel for each timed interrupt config */
    timer_cc_channel_slots[timer_cc_channel_no] = true;
    timed_interrupt_config[pin_number].timer_cc_channel = timer_cc_channel_no;
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
    nrfx_gpiote_in_event_disable(pin_num);
    if (NRFX_SUCCESS != nrfx_ppi_channel_free(timed_interrupt_config[pin_number].ppi_channel))
        return COINES_E_CHANNEL_DEALLOCATION_FAILED;
    //nrfx_ppi_channel_disable(ppi_channel) not called as free does it automatically
    //free up slot for reallocation
    memset(&timed_interrupt_config[pin_number], 0, sizeof(timed_interrupt_config[pin_number]));
    int_pin_usage_native_emulated[pin_number] = false;
    timer_cc_channel_slots[timer_cc_channel_no] = false;
    return COINES_SUCCESS;

}