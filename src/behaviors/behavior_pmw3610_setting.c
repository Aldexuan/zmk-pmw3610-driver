/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_pmw3610_mck_setting

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <dt-bindings/zmk/pmw3610_settings.h>
#include <zmk/pmw3610.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Default CPI step per press. The PMW3610 resolution register advances in
 * 200-cpi steps so this keeps increments aligned with what the sensor can
 * actually program. */
#ifndef CONFIG_PMW3610_MCK_CPI_STEP
#define CONFIG_PMW3610_MCK_CPI_STEP 200
#endif

#ifndef CONFIG_PMW3610_MCK_SNIPE_CPI_STEP
#define CONFIG_PMW3610_MCK_SNIPE_CPI_STEP 200
#endif

#ifndef CONFIG_PMW3610_MCK_SCROLL_TICK_STEP
#define CONFIG_PMW3610_MCK_SCROLL_TICK_STEP 5
#endif

/* Default automouse-timeout step (ms) per press. */
#ifndef CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_STEP
#define CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_STEP 50
#endif

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_parameter_value_metadata pms_param_values[] = {
    {.display_name = "CPI+",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_CPI_INCR},
    {.display_name = "CPI-",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_CPI_DECR},
    {.display_name = "Snipe CPI+",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_SNIPE_CPI_INCR},
    {.display_name = "Snipe CPI-",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_SNIPE_CPI_DECR},
    {.display_name = "Scroll Speed+",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_SCROLL_SPEED_UP},
    {.display_name = "Scroll Speed-",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_SCROLL_SPEED_DOWN},
    {.display_name = "Automouse Timeout+",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_AUTOMOUSE_TIMEOUT_INCR},
    {.display_name = "Automouse Timeout-",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = PMW_AUTOMOUSE_TIMEOUT_DECR},
};

static const struct behavior_parameter_metadata_set pms_param_set[] = {
    {
        .param1_values = pms_param_values,
        .param1_values_len = ARRAY_SIZE(pms_param_values),
    }};

static const struct behavior_parameter_metadata pms_metadata = {
    .sets = pms_param_set,
    .sets_len = ARRAY_SIZE(pms_param_set),
};

#endif /* CONFIG_ZMK_BEHAVIOR_METADATA */

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case PMW_CPI_INCR:
        return zmk_pmw3610_cpi_change(CONFIG_PMW3610_MCK_CPI_STEP);
    case PMW_CPI_DECR:
        return zmk_pmw3610_cpi_change(-CONFIG_PMW3610_MCK_CPI_STEP);
    case PMW_SNIPE_CPI_INCR:
        return zmk_pmw3610_snipe_cpi_change(CONFIG_PMW3610_MCK_SNIPE_CPI_STEP);
    case PMW_SNIPE_CPI_DECR:
        return zmk_pmw3610_snipe_cpi_change(-CONFIG_PMW3610_MCK_SNIPE_CPI_STEP);
    case PMW_SCROLL_SPEED_UP:
        /* Speed up = decrease tick threshold */
        return zmk_pmw3610_scroll_tick_change(-CONFIG_PMW3610_MCK_SCROLL_TICK_STEP);
    case PMW_SCROLL_SPEED_DOWN:
        /* Speed down = increase tick threshold */
        return zmk_pmw3610_scroll_tick_change(CONFIG_PMW3610_MCK_SCROLL_TICK_STEP);
    case PMW_AUTOMOUSE_TIMEOUT_INCR:
        /* Longer automouse layer timeout */
        return zmk_pmw3610_automouse_timeout_change(CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_STEP);
    case PMW_AUTOMOUSE_TIMEOUT_DECR:
        /* Shorter automouse layer timeout */
        return zmk_pmw3610_automouse_timeout_change(-CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_STEP);
    default:
        LOG_WRN("Unknown PMW3610 setting param %d", binding->param1);
        return -ENOTSUP;
    }
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static int zmk_behavior_pmw3610_setting_init(const struct device *dev) { return 0; }

static const struct behavior_driver_api zmk_behavior_pmw3610_setting_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &pms_metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, zmk_behavior_pmw3610_setting_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &zmk_behavior_pmw3610_setting_driver_api);
