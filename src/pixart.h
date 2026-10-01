#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum pixart_input_mode { MOVE = 0, SCROLL, SNIPE, BALL_ACTION };

/* device data structure */
struct pixart_data {
    const struct device *dev;

    enum pixart_input_mode curr_mode;
    uint32_t curr_cpi;
    /* Runtime-adjustable MOVE/SCROLL/BALL_ACTION CPI (initialised to
     * CONFIG_PMW3610_MCK_CPI, updated by zmk_pmw3610_cpi_change()). */
    uint32_t runtime_cpi;
    /* Runtime-adjustable SNIPE CPI (initialised to CONFIG_PMW3610_MCK_SNIPE_CPI,
     * updated by zmk_pmw3610_snipe_cpi_change()). */
    uint32_t runtime_snipe_cpi;
    /* Runtime-adjustable scroll tick threshold (initialised to
     * CONFIG_PMW3610_MCK_SCROLL_TICK, updated by zmk_pmw3610_scroll_tick_change()).
     * Smaller = faster scrolling, larger = slower. */
    uint32_t runtime_scroll_tick;
    /* Runtime-adjustable automouse layer timeout in milliseconds (initialised to
     * CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS, updated by
     * zmk_pmw3610_automouse_timeout_change()). Clamped to [100, 3000] ms. */
    uint32_t runtime_automouse_timeout;
    int32_t scroll_delta_x;
    int32_t scroll_delta_y;
    int32_t ball_action_delta_x;
    int32_t ball_action_delta_y;

#ifdef CONFIG_PMW3610_MCK_POLLING_RATE_125_SW
    int64_t last_poll_time;
    int16_t last_x;
    int16_t last_y;
#endif

#ifdef CONFIG_PMW3610_MCK_SCROLL_ACCELERATION
    int64_t last_scroll_time;
#endif

#if CONFIG_PMW3610_MCK_ACCELERATION_ALGORITHM == 2
    int64_t last_mouse_time;
#endif

    // motion interrupt isr
    struct gpio_callback irq_gpio_cb;
    // the work structure holding the trigger job
    struct k_work trigger_work;

    // the work structure for delayable init steps
    struct k_work_delayable init_work;
    int async_init_step;

    //
    bool ready;           // whether init is finished successfully
    bool last_read_burst; // todo: needed?
    int err;              // error code during async init

    // for pmw3610 smart algorithm
    bool sw_smart_flag;

    // for scroll acceleration
    int64_t last_remainder_time;
    
    // ✅ Hybrid power management states
    bool is_idle;         // ZMK_ACTIVITY_IDLE state
    bool is_sleeping;     // ZMK_ACTIVITY_SLEEP state (GPIO released)
};

// ball action config data structure
struct ball_action_cfg {
    size_t bindings_len;
    struct zmk_behavior_binding *bindings;
    uint8_t layers[ZMK_KEYMAP_LAYERS_LEN];
    size_t layers_len;
    uint32_t tick;
    uint32_t wait_ms;
    uint32_t tap_ms;
};

// device config data structure
struct pixart_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
    size_t scroll_layers_len;
    int32_t *scroll_layers;
    size_t snipe_layers_len;
    int32_t *snipe_layers;
    struct ball_action_cfg **ball_actions;
    size_t ball_actions_len;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
