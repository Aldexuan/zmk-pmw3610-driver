/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_pmw3610_mck

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/settings/settings.h>
#include <drivers/behavior.h>
#include <math.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/keys.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610_mck, CONFIG_INPUT_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10, // test shows > 5ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] =
        200,                          // 150 us required, test shows too short,
                                      // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50, // 10 ms required in spec,
                                      // test shows too short,
                                      // especially when integrated with display,
                                      // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

/* Forward declarations for settings-persistence helpers implemented near the
 * bottom of this file. Called from pmw3610_async_init_configure() so that a
 * previously-persisted CPI is applied before set_cpi() programs the sensor. */
static void pmw3610_settings_init(void);
static uint32_t pmw3610_get_persisted_cpi(void);
static int pmw3610_settings_schedule_save(void);

/* Forward declaration for the singleton device getter. Defined near the bottom
 * of this file and used by the runtime-settings API (always compiled) and by
 * the automouse layer activation helper. */
static const struct device *zmk_pmw3610_get_dev(void);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

// checked and keep
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

// checked and keep
static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

// primitive write without enable/disable spi clock on the sensor
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    // write the target register
    err = _reg_write(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Send motion burst address */
    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    /* Terminate burst */
    k_busy_wait(T_BEXIT);

    return 0;
}

/** Writing an array of registers in sequence, used in power-up register initialization and running
 * mode switching */
static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf,
                       size_t size) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        err = _reg_write(dev, addr[i], buf[i]);

        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};
    int err = burst_write(dev, addr, data, 3);
    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        return set_cpi(dev, cpi);
    }
    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_MCK_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_MCK_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_MCK_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_MCK_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    LOG_INF("async_init_power_up");

    /* Reset spi port */
    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    /* not required in datashet, but added any way to have a clear state */
    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    LOG_INF("async_init_clear_ob1");

    return reg_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    LOG_INF("async_init_check_ob1");

    uint8_t value;
    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    err = check_product_id(dev);
    if (err) {
        LOG_ERR("Failed checking product id");
        return err;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    LOG_INF("async_init_configure");

    int err = 0;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = reg_read(dev, reg, buf);
    }

    // cpi
    if (!err) {
        struct pixart_data *d = dev->data;
        /* Lazy-init the settings subsystem (safe now: we are running from
         * the system work queue, post-kernel, so the flash backend is up).
         * If a persisted CPI exists AND runtime_cpi is still the cold-boot
         * default, adopt the persisted value so the user's last choice
         * survives reboots / soft-off wake-ups. On PM RESUME we keep
         * whatever the user had set mid-session (RAM preserved). */
        pmw3610_settings_init();
        if (d->runtime_cpi == CONFIG_PMW3610_MCK_CPI) {
            uint32_t persisted = pmw3610_get_persisted_cpi();
            if (persisted >= PMW3610_MIN_CPI && persisted <= PMW3610_MAX_CPI) {
                d->runtime_cpi = persisted;
                LOG_INF("Applied persisted PMW3610 CPI=%u", d->runtime_cpi);
            }
        } else if (d->runtime_cpi == 0) {
            d->runtime_cpi = CONFIG_PMW3610_MCK_CPI;
        }
        /* Initialise snipe CPI from Kconfig default on first boot. */
        if (d->runtime_snipe_cpi == 0) {
            d->runtime_snipe_cpi = CONFIG_PMW3610_MCK_SNIPE_CPI;
        }
        /* Initialise scroll tick from Kconfig default on first boot. */
        if (d->runtime_scroll_tick == 0) {
            d->runtime_scroll_tick = CONFIG_PMW3610_MCK_SCROLL_TICK;
        }
        /* Initialise automouse timeout from Kconfig default on first boot. */
        if (d->runtime_automouse_timeout == 0) {
            d->runtime_automouse_timeout = CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS;
        }
        err = set_cpi(dev, d->runtime_cpi);
    }

    // set performace register: run mode, vel_rate, poshi_rate, poslo_rate
    // 强制清除 FORCE_AWAKE 位，防止传感器无法降频导致功耗升高
    if (!err) {
        // 确保 FORCE_AWAKE (bit 4) 被清除
        uint8_t perf_value = PMW3610_PERFORMANCE_VALUE & ~0x10;
        
        err = reg_write(dev, PMW3610_REG_PERFORMANCE, perf_value);
        LOG_INF("Set PERFORMANCE register: 0x%02x", perf_value);
        
        // 验证写入是否成功
        k_msleep(10);
        uint8_t readback = 0;
        int read_err = reg_read(dev, PMW3610_REG_PERFORMANCE, &readback);
        
        if (!read_err) {
            LOG_INF("PERFORMANCE readback: 0x%02x", readback);
            
            // 检查 FORCE_AWAKE 位
            if (readback & 0x10) {
                LOG_ERR("⚠️  FORCE_AWAKE bit is stuck! Forcing clear...");
                
                // 多次尝试清除（最多 5 次）
                for (int retry = 0; retry < 5; retry++) {
                    err = reg_write(dev, PMW3610_REG_PERFORMANCE, perf_value);
                    k_msleep(20);
                    reg_read(dev, PMW3610_REG_PERFORMANCE, &readback);
                    
                    LOG_INF("Clear retry %d: readback = 0x%02x", retry + 1, readback);
                    
                    if (!(readback & 0x10)) {
                        LOG_INF("✅ FORCE_AWAKE cleared successfully on retry %d", retry + 1);
                        break;
                    }
                }
                
                // 最终检查
                if (readback & 0x10) {
                    LOG_ERR("❌ Failed to clear FORCE_AWAKE after 5 retries!");
                    LOG_ERR("   Sensor will stay in RUN mode, power consumption ~500μA higher");
                    LOG_ERR("   Consider hardware reset or sensor replacement");
                } else {
                    LOG_INF("✅ FORCE_AWAKE successfully disabled");
                }
            } else {
                LOG_INF("✅ FORCE_AWAKE is disabled (bit 4 = 0)");
                LOG_INF("   Sensor can enter REST modes for power saving");
            }
            
            // 输出完整的寄存器解析
            LOG_INF("PERFORMANCE register breakdown:");
            LOG_INF("  FORCE_AWAKE (bit 4): %s", (readback & 0x10) ? "1 (ENABLED ⚠️)" : "0 (disabled ✅)");
            LOG_INF("  Polling rate: %s", 
                    (readback & 0x0C) == 0x0C ? "250Hz" : 
                    (readback & 0x0C) == 0x00 ? "125Hz" : "other");
        } else {
            LOG_WRN("Cannot read back PERFORMANCE register for verification");
        }
    }

    // required downshift and rate registers
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3610_MCK_RUN_DOWNSHIFT_TIME_MS);
    }
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_MCK_REST1_SAMPLE_TIME_MS);
    }
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3610_MCK_REST1_DOWNSHIFT_TIME_MS);
    }

    // downshift time for each rest mode
#if CONFIG_PMW3610_MCK_REST2_DOWNSHIFT_TIME_MS > 0
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                 CONFIG_PMW3610_MCK_REST2_DOWNSHIFT_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_MCK_REST2_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST2_PERIOD, CONFIG_PMW3610_MCK_REST2_SAMPLE_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_MCK_REST3_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST3_PERIOD, CONFIG_PMW3610_MCK_REST3_SAMPLE_TIME_MS);
    }
#endif
    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

// checked and keep
static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed");
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3610 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
struct k_timer automouse_layer_timer;
static bool automouse_triggered = false;

/* Effective automouse layer timeout (ms): the runtime-adjustable value when
 * set, otherwise the Kconfig default. Kept inside the AUTOMOUSE_LAYER guard so
 * it is only compiled when the feature is enabled. */
static uint32_t pmw3610_automouse_timeout_effective(void) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS;
    }
    struct pixart_data *data = dev->data;
    if (data->runtime_automouse_timeout == 0) {
        return CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS;
    }
    return data->runtime_automouse_timeout;
}

static void activate_automouse_layer() {
    automouse_triggered = true;
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    k_timer_start(&automouse_layer_timer, K_MSEC(pmw3610_automouse_timeout_effective()), K_NO_WAIT);
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

int ball_action_idx = -1;
static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    ball_action_idx = -1;
    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }
    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }
    for (size_t i = 0; i < config->ball_actions_len; i++) {
        for (size_t j = 0; j < config->ball_actions[i]->layers_len; j++) {
            if (curr_layer == config->ball_actions[i]->layers[j]) {
                ball_action_idx = i;
                return BALL_ACTION;
            }
        }
    }
    return MOVE;
}

static inline void calculate_scroll_acceleration(int16_t x, int16_t y, struct pixart_data *data,
                                                 int32_t *accel_x, int32_t *accel_y) {
    *accel_x = x;
    *accel_y = y;

#ifdef CONFIG_PMW3610_MCK_SCROLL_ACCELERATION
    int32_t movement = abs(x) + abs(y);
    int64_t current_time = k_uptime_get();
    int64_t delta_time = data->last_scroll_time > 0 ? current_time - data->last_scroll_time : 0;

    if (delta_time > 0 && delta_time < 100) {
        float speed = (float)movement / delta_time;
        float base_sensitivity = (float)CONFIG_PMW3610_MCK_SCROLL_ACCELERATION_SENSITIVITY;
        float acceleration =
            1.0f + (base_sensitivity - 1.0f) * (1.0f / (1.0f + expf(-0.2f * (speed - 10.0f))));

        *accel_x = (int32_t)(x * acceleration);
        *accel_y = (int32_t)(y * acceleration);

        if (abs(x) <= 1)
            *accel_x = x;
        if (abs(y) <= 1)
            *accel_y = y;
    }

    data->last_scroll_time = current_time;
#endif
}

static inline void calculate_mouse_acceleration(int16_t x, int16_t y, struct pixart_data *data,
                                                int32_t *accel_x, int32_t *accel_y) {
    *accel_x = x;
    *accel_y = y;

#if CONFIG_PMW3610_MCK_ACCELERATION_ALGORITHM > 0
    // Don't accelerate very small movements (preserve precision)
    if (abs(x) <= 1 && abs(y) <= 1) {
        return;
    }

#if CONFIG_PMW3610_MCK_ACCELERATION_ALGORITHM == 1
    // QMK-style quadratic acceleration: output = x * (1 + |x|/divider)
    // Sensitivity maps to divider: higher sensitivity = lower divider = more acceleration
    // divider = 22 - (sensitivity * 2)
    // Sensitivity 1: divider=20, Sensitivity 7: divider=8 (QMK tuned), Sensitivity 10: divider=2
    const int32_t divider = 22 - (CONFIG_PMW3610_MCK_ACCELERATION_SENSITIVITY * 2);

    *accel_x = (x > 0) ? (x * x / divider + x) : (-x * x / divider + x);
    *accel_y = (y > 0) ? (y * y / divider + y) : (-y * y / divider + y);

    // Preserve individual axis precision for small movements
    if (abs(x) <= 1)
        *accel_x = x;
    if (abs(y) <= 1)
        *accel_y = y;

#elif CONFIG_PMW3610_MCK_ACCELERATION_ALGORITHM == 2
    // Speed-based sigmoid acceleration with gentler low-speed curve
    int32_t movement = abs(x) + abs(y);
    int64_t current_time = k_uptime_get();
    int64_t delta_time = data->last_mouse_time > 0 ? current_time - data->last_mouse_time : 0;

    // Always apply some acceleration to avoid frame skipping
    float acceleration = 1.0f;

    if (delta_time > 0 && delta_time < 100) {
        float speed = (float)movement / delta_time;
        float base_sensitivity = (float)CONFIG_PMW3610_MCK_ACCELERATION_SENSITIVITY;
        acceleration =
            1.0f + (base_sensitivity - 1.0f) * (1.0f / (1.0f + expf(-0.25f * (speed - 10.0f))));
    }

    data->last_mouse_time = current_time;

    *accel_x = (int32_t)(x * acceleration);
    *accel_y = (int32_t)(y * acceleration);

    // Preserve individual axis precision for small movements
    if (abs(x) <= 1)
        *accel_x = x;
    if (abs(y) <= 1)
        *accel_y = y;
#endif
#endif
}

static inline void process_scroll_events(const struct device *dev, struct pixart_data *data,
                                         int32_t delta, bool is_horizontal) {
    uint32_t scroll_tick = data->runtime_scroll_tick > 0
                               ? data->runtime_scroll_tick
                               : CONFIG_PMW3610_MCK_SCROLL_TICK;
    if (abs(delta) > scroll_tick) {
        int event_count = abs(delta) / scroll_tick;
        const int MAX_EVENTS = 20;
        int32_t *target_delta = is_horizontal ? &data->scroll_delta_x : &data->scroll_delta_y;

        if (event_count > MAX_EVENTS) {
            event_count = MAX_EVENTS;
            *target_delta = (delta > 0) ? delta - (MAX_EVENTS * scroll_tick)
                                        : delta + (MAX_EVENTS * scroll_tick);
            data->last_remainder_time = k_uptime_get();
        } else {
            *target_delta = delta % scroll_tick;
        }

        for (int i = 0; i < event_count; i++) {
            input_report_rel(
                dev, is_horizontal ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL,
                delta > 0 ? (is_horizontal ? PMW3610_SCROLL_X_NEGATIVE : PMW3610_SCROLL_Y_NEGATIVE)
                          : (is_horizontal ? PMW3610_SCROLL_X_POSITIVE : PMW3610_SCROLL_Y_POSITIVE),
                (i == event_count - 1), K_MSEC(10));
        }

        if (is_horizontal) {
            data->scroll_delta_y = 0;
        } else {
            data->scroll_delta_x = 0;
        }
    }
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    switch (input_mode) {
    case MOVE:
        set_cpi_if_needed(dev, data->runtime_cpi);
        dividor = CONFIG_PMW3610_MCK_CPI_DIVIDOR;
        break;
    case SCROLL:
        set_cpi_if_needed(dev, data->runtime_cpi);
        if (input_mode_changed) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        }
        dividor = 1; // this should be handled with the ticks rather than dividors
        break;
    case SNIPE:
        set_cpi_if_needed(dev, data->runtime_snipe_cpi);
        dividor = CONFIG_PMW3610_MCK_SNIPE_CPI_DIVIDOR;
        break;
    case BALL_ACTION:
        set_cpi_if_needed(dev, data->runtime_cpi);
        if (input_mode_changed) {
            data->ball_action_delta_x = 0;
            data->ball_action_delta_y = 0;
        }
        dividor = 1;
        break;
    default:
        return -ENOTSUP;
    }

    data->curr_mode = input_mode;

    int16_t x = 0;
    int16_t y = 0;

#if AUTOMOUSE_LAYER > 0
    if (input_mode == MOVE &&
        (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER) &&
        (abs(x) + abs(y) > CONFIG_PMW3610_MCK_MOVEMENT_THRESHOLD)) {
        activate_automouse_layer();
    }
#endif

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    int16_t raw_x =
        TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / dividor;
    int16_t raw_y =
        TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / dividor;

    if (IS_ENABLED(CONFIG_PMW3610_MCK_ORIENTATION_0)) {
        x = -raw_x;
        y = raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_MCK_ORIENTATION_90)) {
        x = raw_y;
        y = -raw_x;
    } else if (IS_ENABLED(CONFIG_PMW3610_MCK_ORIENTATION_180)) {
        x = raw_x;
        y = -raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_MCK_ORIENTATION_270)) {
        x = -raw_y;
        y = raw_x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_MCK_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_MCK_INVERT_Y)) {
        y = -y;
    }

    int64_t current_time = k_uptime_get();
    if (data->last_remainder_time > 0) {
        int64_t elapsed = current_time - data->last_remainder_time;
        if (elapsed > 100) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
            data->last_remainder_time = 0;
        }
    }

#ifdef CONFIG_PMW3610_MCK_SMART_ALGORITHM
    int16_t shutter =
        ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        reg_write(dev, 0x32, 0x00);

        data->sw_smart_flag = false;
    }

    if (!data->sw_smart_flag && shutter > 45) {
        reg_write(dev, 0x32, 0x80);

        data->sw_smart_flag = true;
    }
#endif

#ifdef CONFIG_PMW3610_MCK_POLLING_RATE_125_SW
    int64_t curr_time = k_uptime_get();
    if (data->last_poll_time == 0 || curr_time - data->last_poll_time > 128) {
        data->last_poll_time = curr_time;
        data->last_x = x;
        data->last_y = y;
        return 0;
    } else {
        x += data->last_x;
        y += data->last_y;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;
    }
#endif

    if (x != 0 || y != 0) {
        if (input_mode == MOVE || input_mode == SNIPE) {
            int32_t accel_x, accel_y;
            calculate_mouse_acceleration(x, y, data, &accel_x, &accel_y);

#if AUTOMOUSE_LAYER > 0
            // トラックボールの動きの大きさを計算
            int16_t movement_size = abs(x) + abs(y);
            if (input_mode == MOVE &&
                (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER) &&
                movement_size > CONFIG_PMW3610_MCK_MOVEMENT_THRESHOLD) {
                activate_automouse_layer();
            }
#endif
            input_report_rel(dev, INPUT_REL_X, accel_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, accel_y, true, K_FOREVER);
        } else if (input_mode == SCROLL) {
            int32_t accel_x, accel_y;
            calculate_scroll_acceleration(x, y, data, &accel_x, &accel_y);

            data->scroll_delta_x += accel_x;
            data->scroll_delta_y += accel_y;

            process_scroll_events(dev, data, data->scroll_delta_y, false);
            process_scroll_events(dev, data, data->scroll_delta_x, true);
        } else if (input_mode == BALL_ACTION) {
            data->ball_action_delta_x += x;
            data->ball_action_delta_y += y;

            const struct pixart_config *config = dev->config;

            if (ball_action_idx != -1) {
                const struct ball_action_cfg action_cfg = *config->ball_actions[ball_action_idx];

                LOG_DBG("invoking ball action [%d], layer=%d", ball_action_idx,
                        zmk_keymap_highest_layer_active());

                struct zmk_behavior_binding_event event = {
                    .position = INT32_MAX,
                    .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
                    .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif

                };

                // determine which binding to invoke
                int idx = -1;
                if (abs(data->ball_action_delta_x) > action_cfg.tick) {
                    idx = data->ball_action_delta_x > 0 ? 0 : 1;
                } else if (abs(data->ball_action_delta_y) > action_cfg.tick) {
                    idx = data->ball_action_delta_y > 0 ? 3 : 2;
                }

                if (idx != -1) {
                    zmk_behavior_queue_add(&event, action_cfg.bindings[idx], true,
                                           action_cfg.tap_ms);
                    zmk_behavior_queue_add(&event, action_cfg.bindings[idx], false,
                                           action_cfg.wait_ms);

                    data->ball_action_delta_x = 0;
                    data->ball_action_delta_y = 0;
                }
            }
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // submit the real handler work
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);
    
    // 🔧 修复：只在ACTIVE状态下重新启用中断
    // 如果系统已经进入IDLE/SLEEP，不要重新启用中断，让传感器降频
    if (!data->is_idle && !data->is_sleeping) {
        set_interrupt(dev, true);
    } else {
        LOG_DBG("Skip re-enabling IRQ: is_idle=%d, is_sleeping=%d", 
                data->is_idle, data->is_sleeping);
    }
}

static int pmw3610_init_irq(const struct device *dev) {
    LOG_INF("Configure irq...");

    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3610_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    // init smart algorithm flag;
    data->sw_smart_flag = false;

    // init runtime (dynamic) cpi to the Kconfig default
    data->runtime_cpi = CONFIG_PMW3610_MCK_CPI;

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // check readiness of cs gpio pin and init it to inactive
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

/* =============================================================================
 * HYBRID POWER MANAGEMENT: ZMK Activity Listener (替代 PM_DEVICE)
 * =============================================================================
 * 使用 ZMK 活动状态监听器实现三级功耗管理，比 PM_DEVICE 更可靠：
 * 
 * 1. ZMK_ACTIVITY_ACTIVE (活跃)
 *    - 传感器：RUN 模式，正常工作
 *    - GPIO：全部连接
 *    - 中断：启用（实时响应运动）
 *    - 功耗：~2.5mA
 * 
 * 2. ZMK_ACTIVITY_IDLE (空闲)
 *    - 传感器：自动降频 RUN → REST1 (128ms) → REST2 (9.6s) → REST3 (28.8s)
 *    - GPIO：保持连接（避免唤醒时完整初始化）
 *    - 中断：**禁用**（关键！避免传感器被不断唤醒，无法进入 REST 模式）
 *    - 功耗：~200-250μA（REST3 模式）
 *    - 优势：快速唤醒（传感器仍供电，无需重新初始化）
 * 
 * 3. ZMK_ACTIVITY_SLEEP (深度睡眠)
 *    - 传感器：保持在 REST3 模式
 *    - GPIO：全部释放（完全断开状态），防止电流回灌和漏电
 *    - 中断：禁用并移除回调
 *    - 功耗：~300-350μA（目标）
 *    - 组成：传感器 REST3 (~200μA) + MCU (~50μA) + BLE (~50μA) + 其他 (~50μA)
 *    - 优势：低功耗，唤醒时间约 50ms（需要恢复 GPIO 和重新初始化）
 * 
 * 注意：
 * - IDLE 状态必须禁用中断，否则传感器会被持续唤醒，无法降频
 * - 启用完整的 REST1 → REST2 → REST3 降频链（Kconfig 配置）
 * - 不使用硬件 SHUTDOWN 功能（避免之前遇到的功耗升高问题）
 * ============================================================================= */

// 前置声明
static int pmw3610_restore_gpio_on_wake(const struct device *dev);

/* 处理 ZMK_ACTIVITY_SLEEP → ACTIVE/IDLE 的唤醒 */
static int pmw3610_on_wake_from_sleep(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    struct pixart_data *data = dev->data;
    int err = 0;

    LOG_INF("PMW3610 waking from SLEEP - restoring GPIO and reinitializing");

    // 1. 恢复 CS 引脚（SPI 片选）- 输出，非激活状态（高电平）
    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot restore CS GPIO: %d", err);
        return err;
    }

    // 2. 恢复 IRQ 引脚 - 输入模式
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot restore IRQ GPIO: %d", err);
        return err;
    }

    // 3. 重新添加 GPIO 中断回调
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot re-add IRQ GPIO callback: %d", err);
        return err;
    }

    // 4. SPI 数据引脚（MOSI/MISO/SCK）会由 Zephyr SPI 驱动自动恢复
    //    因为在第一次 SPI 传输时，驱动会重新配置引脚为 SPI 功能
    LOG_DBG("SPI data pins will be reconfigured by SPI driver on first transaction");

    // 5. 触发完整的重新初始化序列（与 PM_DEVICE_ACTION_RESUME 相同）
    //    从 POWER_UP 步骤开始，确保传感器状态正确
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->ready = false;  // 在初始化完成前保持未就绪状态
    
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
    
    LOG_INF("PMW3610 wake: GPIO restored, full reinitialization started");
    
    // 注意：中断会在初始化完成后自动启用（在 pmw3610_async_init 的最后步骤）
    data->is_sleeping = false;

    return 0;
}

/* 处理 ACTIVE → IDLE 的转换 */
static int pmw3610_on_enter_idle(const struct device *dev) {
    struct pixart_data *data = dev->data;

    LOG_INF("PMW3610 entering IDLE - disabling IRQ to allow sensor downshift");
    
    // 🔧 关键修复：先设置状态标志，阻止work callback重新启用中断
    data->is_idle = true;
    data->is_sleeping = false;
    
    // 然后禁用中断，避免传感器被不断唤醒
    // 这样传感器才能自动降频： RUN → REST1 (128ms) → REST2 (9.6s) → REST3 (28.8s)
    set_interrupt(dev, false);
    
    // 取消待处理的工作（避免残留的中断处理）
    // 即使有work正在执行，因为is_idle已经设置，它也不会重新启用中断
    k_work_cancel(&data->trigger_work);
    
    LOG_INF("PMW3610 IDLE: IRQ disabled, sensor will downshift to REST modes");

    return 0;
}

/* 处理 IDLE/SLEEP → ACTIVE 的转换 */
static int pmw3610_on_enter_active(const struct device *dev) {
    struct pixart_data *data = dev->data;
    int err = 0;

    // 如果从 SLEEP 唤醒，需要恢复 GPIO
    if (data->is_sleeping) {
        err = pmw3610_on_wake_from_sleep(dev);
        if (err) {
            return err;
        }
    } else if (data->is_idle) {
        // 从 IDLE 恢复：重新启用中断
        LOG_INF("PMW3610 resuming from IDLE - re-enabling IRQ");
        set_interrupt(dev, true);
    }

    LOG_INF("PMW3610 entering ACTIVE - sensor will auto-upshift to RUN mode");

    // 传感器会在检测到运动时自动从 REST 恢复到 RUN 模式
    data->is_idle = false;
    data->is_sleeping = false;

    return 0;
}

/* 处理 IDLE/ACTIVE → SLEEP 的转换 */
static int pmw3610_on_enter_sleep(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    struct pixart_data *data = dev->data;
    int err = 0;

    LOG_INF("PMW3610 entering SLEEP - releasing GPIO to prevent back-feed");

    // 1. 取消所有待处理的工作
    k_work_cancel_delayable(&data->init_work);
    k_work_cancel(&data->trigger_work);

    // 2. 禁用并移除 GPIO 中断
    gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_DISABLE);
    gpio_remove_callback(config->irq_gpio.port, &data->irq_gpio_cb);

    // 3. 释放 IRQ 引脚为完全断开状态（防止电流回灌和漏电）
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_DISCONNECTED);
    if (err) {
        LOG_WRN("Failed to release IRQ pin: %d", err);
    }

    // 4. 释放 CS 引脚为完全断开状态
    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_DISCONNECTED);
    if (err) {
        LOG_WRN("Failed to release CS pin: %d", err);
    }

    // 5. 释放 SPI 数据引脚为高阻态（防止通过 ESD 二极管回灌电流）
    {
        const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

        if (device_is_ready(gpio0)) {
            // MOSI/MISO on P0.10
            gpio_pin_configure(gpio0, 10, GPIO_DISCONNECTED);
            LOG_INF("Released P0.10 (MOSI/MISO) to high-Z");
        }
        if (device_is_ready(gpio1)) {
            // SCK on P1.13
            gpio_pin_configure(gpio1, 13, GPIO_DISCONNECTED);
            LOG_INF("Released P1.13 (SCK) to high-Z");
        }
    }

    // 6. 标记设备为未就绪（与 PM_DEVICE_ACTION_SUSPEND 相同）
    //    防止在深睡眠期间有任何传感器访问
    data->ready = false;

    // 注意：传感器仍然供电，会自动进入 REST3 模式（经过 REST1→REST2→REST3）
    // 不调用 SHUTDOWN 命令，避免之前的功耗升高问题
    data->is_idle = false;
    data->is_sleeping = true;

    LOG_INF("PMW3610 SLEEP complete - GPIO fully disconnected, sensor in REST3, target ~300-350μA");

    return 0;
}

/* ZMK 活动状态变化监听器 */
static int pmw3610_activity_state_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // 获取 PMW3610 设备实例（假设使用实例 0）
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    if (!device_is_ready(dev)) {
        LOG_WRN("PMW3610 device not ready");
        return ZMK_EV_EVENT_BUBBLE;
    }

    // 根据 ZMK 活动状态执行对应操作
    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        LOG_INF("ZMK state: ACTIVE");
        pmw3610_on_enter_active(dev);
        break;

    case ZMK_ACTIVITY_IDLE:
        LOG_INF("ZMK state: IDLE");
        pmw3610_on_enter_idle(dev);
        break;

    case ZMK_ACTIVITY_SLEEP:
        LOG_INF("ZMK state: SLEEP");
        pmw3610_on_enter_sleep(dev);
        break;

    default:
        LOG_WRN("Unknown ZMK activity state: %d", ev->state);
        break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/* 注册 ZMK 活动状态监听器 */
ZMK_LISTENER(pmw3610_activity_listener, pmw3610_activity_state_listener);
ZMK_SUBSCRIPTION(pmw3610_activity_listener, zmk_activity_state_changed);

#define TRANSFORMED_BINDINGS(n)                                                                    \
    {LISTIFY(DT_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), n)}

#define BALL_ACTIONS_INST(n)                                                                       \
    static struct zmk_behavior_binding                                                             \
        ball_action_config_##n##_bindings[DT_PROP_LEN(n, bindings)] = TRANSFORMED_BINDINGS(n);     \
                                                                                                   \
    static struct ball_action_cfg ball_action_cfg_##n = {                                          \
        .bindings_len = DT_PROP_LEN(n, bindings),                                                  \
        .bindings = ball_action_config_##n##_bindings,                                             \
        .layers = DT_PROP(n, layers),                                                              \
        .layers_len = DT_PROP_LEN(n, layers),                                                      \
        .tick = DT_PROP_OR(n, tick, CONFIG_PMW3610_MCK_BALL_ACTION_TICK),                              \
        .wait_ms = DT_PROP_OR(n, wait_ms, 0),                                                      \
        .tap_ms = DT_PROP_OR(n, tap_ms, 0),                                                        \
    };

DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_INST)

#define BALL_ACTIONS_ITEM(n) &ball_action_cfg_##n,
#define BALL_ACTIONS_UTIL_ONE(n) 1 +

#define BALL_ACTIONS_LEN (DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_UTIL_ONE) 0)

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);                    \
    static int32_t snipe_layers##n[] = DT_PROP(DT_DRV_INST(n), snipe_layers);                      \
    static struct ball_action_cfg *ball_actions[] = {DT_INST_FOREACH_CHILD(0, BALL_ACTIONS_ITEM)}; \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
        .ball_actions = ball_actions,                                                              \
        .ball_actions_len = BALL_ACTIONS_LEN,                                                      \
    };                                                                                             \
                                                                                                   \
    /* 功耗管理：使用 ZMK 活动状态监听器 (替代 PM_DEVICE)                                    */   \
    /* - ACTIVE: 传感器 RUN 模式，正常工作 (~2.5mA)                                          */   \
    /* - IDLE: 传感器自动降频到 REST1 (~360μA)                                                */   \
    /* - SLEEP: 传感器降至 REST3，GPIO 完全断开 (~300-350μA)                                  */   \
    /* 注意：未使用硬件 SHUTDOWN 功能，以避免之前遇到的功耗升高问题 (700-800μA)              */   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL,                                                   \
                          &data##n, &config##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

/* ---------------------------------------------------------------------------
 *  Public runtime-settings API
 * ------------------------------------------------------------------------- */

#include <zmk/pmw3610.h>

/* Use instance 0 as the singleton target for runtime adjustments. */
#define PMW3610_RUNTIME_INST 0

static const struct device *zmk_pmw3610_get_dev(void);

/* ---------------------------------------------------------------------------
 *  Settings persistence (flash-backed)
 * ---------------------------------------------------------------------------
 * The runtime CPI lives in RAM (struct pixart_data::runtime_cpi), so it is
 * lost whenever the keyboard does a full reset - which happens on every
 * soft-off wake-up. To make the user-selected CPI survive resets we stash it
 * in Zephyr's settings subsystem (flash-backed), mirroring what the PS/2
 * mouse driver does for its trackpoint settings.
 *
 * Writes are debounced through a k_work_delayable so bursty key presses
 * collapse into a single flash write (CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE,
 * 60 s by default in ZMK). This is the exact pattern ZMK uses for BLE
 * profile storage and keeps flash wear at a minimum.
 */

#if IS_ENABLED(CONFIG_SETTINGS)

#define PMW3610_SETTINGS_SUBTREE "pmw3610_mck"
#define PMW3610_SETTINGS_KEY_CPI "cpi"

/* Value read from flash during settings_load, applied during async init
 * before the initial set_cpi() call runs. Zero means "no stored value". */
static uint32_t pmw3610_persisted_cpi;

static struct k_work_delayable pmw3610_save_work;

static void pmw3610_save_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return;
    }
    struct pixart_data *data = dev->data;

    uint32_t value = data->runtime_cpi;
    int err = settings_save_one(PMW3610_SETTINGS_SUBTREE "/" PMW3610_SETTINGS_KEY_CPI, &value,
                                sizeof(value));
    if (err) {
        LOG_ERR("Failed to persist PMW3610 CPI: %d", err);
    } else {
        LOG_INF("Persisted PMW3610 CPI=%u to flash", value);
    }
}

static int pmw3610_settings_schedule_save(void) {
    int ret = k_work_reschedule(&pmw3610_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static int pmw3610_settings_restore(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    if (strcmp(name, PMW3610_SETTINGS_KEY_CPI) != 0) {
        return 0;
    }
    if (len != sizeof(pmw3610_persisted_cpi)) {
        LOG_WRN("Stored CPI has unexpected size %u; ignoring", (unsigned int)len);
        return 0;
    }
    uint32_t value;
    int rc = read_cb(cb_arg, &value, sizeof(value));
    if (rc < 0) {
        LOG_ERR("Failed to read stored CPI: %d", rc);
        return rc;
    }
    pmw3610_persisted_cpi = value;
    LOG_INF("Loaded persisted PMW3610 CPI=%u", value);
    return 0;
}

static struct settings_handler pmw3610_settings_conf = {
    .name = PMW3610_SETTINGS_SUBTREE,
    .h_set = pmw3610_settings_restore,
};

static uint32_t pmw3610_get_persisted_cpi(void) { return pmw3610_persisted_cpi; }

/* One-shot lazy init. Safe to call repeatedly - only the first call does
 * anything. Invoked from pmw3610_async_init_configure() so it runs on the
 * system work queue, after kernel startup, when the flash backend is up. */
static void pmw3610_settings_init(void) {
    static bool done;
    if (done) {
        return;
    }
    done = true;

    k_work_init_delayable(&pmw3610_save_work, pmw3610_save_work_handler);

    int err = settings_subsys_init();
    if (err) {
        LOG_WRN("settings_subsys_init failed (%d); CPI will not persist", err);
        return;
    }
    err = settings_register(&pmw3610_settings_conf);
    if (err && err != -EEXIST) {
        LOG_WRN("settings_register failed (%d); CPI will not persist", err);
        return;
    }
    err = settings_load_subtree(PMW3610_SETTINGS_SUBTREE);
    if (err) {
        LOG_WRN("settings_load_subtree failed (%d); using default CPI", err);
    }
}

#else /* !CONFIG_SETTINGS */

static inline void pmw3610_settings_init(void) {}
static inline int pmw3610_settings_schedule_save(void) { return 0; }
static inline uint32_t pmw3610_get_persisted_cpi(void) { return 0; }

#endif /* CONFIG_SETTINGS */

static const struct device *zmk_pmw3610_get_dev(void) {
#if DT_NODE_HAS_STATUS(DT_DRV_INST(PMW3610_RUNTIME_INST), okay)
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(PMW3610_RUNTIME_INST));
    if (!device_is_ready(dev)) {
        return NULL;
    }
    return dev;
#else
    return NULL;
#endif
}

int zmk_pmw3610_cpi_change(int amount) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        LOG_ERR("PMW3610 device not available for CPI change");
        return -ENODEV;
    }

    struct pixart_data *data = dev->data;

    int32_t current = (int32_t)data->runtime_cpi;
    int32_t target = current + amount;

    /* Snap to 200-cpi step and clamp to sensor limits. */
    if (target < (int32_t)PMW3610_MIN_CPI) {
        target = PMW3610_MIN_CPI;
    } else if (target > (int32_t)PMW3610_MAX_CPI) {
        target = PMW3610_MAX_CPI;
    }
    target = (target / 200) * 200;
    if (target < (int32_t)PMW3610_MIN_CPI) {
        target = PMW3610_MIN_CPI;
    }

    if ((uint32_t)target == data->runtime_cpi) {
        LOG_INF("PMW3610 CPI unchanged at %d (requested delta %d)", target, amount);
        return 0;
    }

    data->runtime_cpi = (uint32_t)target;
    LOG_INF("PMW3610 runtime CPI -> %d (delta %d)", target, amount);

    /* Persist the new value. The actual flash write is debounced inside
     * pmw3610_save_work_handler() - bursty presses collapse into one write. */
    pmw3610_settings_schedule_save();

    /* If current input mode actually uses the runtime CPI, push immediately
     * so the change is audible without waiting for the next mode switch. */
    if (data->ready) {
        switch (data->curr_mode) {
        case MOVE:
        case SCROLL:
        case BALL_ACTION:
            return set_cpi_if_needed(dev, data->runtime_cpi);
        default:
            break;
        }
    }

    return 0;
}

int zmk_pmw3610_cpi_get(void) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return -ENODEV;
    }
    struct pixart_data *data = dev->data;
    return (int)data->runtime_cpi;
}

int zmk_pmw3610_snipe_cpi_change(int amount) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        LOG_ERR("PMW3610 device not available for snipe CPI change");
        return -ENODEV;
    }

    struct pixart_data *data = dev->data;

    /* Initialise from Kconfig default if not yet set. */
    if (data->runtime_snipe_cpi == 0) {
        data->runtime_snipe_cpi = CONFIG_PMW3610_MCK_SNIPE_CPI;
    }

    int32_t current = (int32_t)data->runtime_snipe_cpi;
    int32_t target = current + amount;

    /* Snap to 200-cpi step and clamp to sensor limits. */
    if (target < (int32_t)PMW3610_MIN_CPI) {
        target = PMW3610_MIN_CPI;
    } else if (target > (int32_t)PMW3610_MAX_CPI) {
        target = PMW3610_MAX_CPI;
    }
    target = (target / 200) * 200;
    if (target < (int32_t)PMW3610_MIN_CPI) {
        target = PMW3610_MIN_CPI;
    }

    if ((uint32_t)target == data->runtime_snipe_cpi) {
        LOG_INF("PMW3610 snipe CPI unchanged at %d (requested delta %d)", target, amount);
        return 0;
    }

    data->runtime_snipe_cpi = (uint32_t)target;
    LOG_INF("PMW3610 snipe CPI -> %d (delta %d)", target, amount);

    /* If currently in SNIPE mode, push the change to the sensor immediately. */
    if (data->ready && data->curr_mode == SNIPE) {
        return set_cpi_if_needed(dev, data->runtime_snipe_cpi);
    }

    return 0;
}

int zmk_pmw3610_snipe_cpi_get(void) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return -ENODEV;
    }
    struct pixart_data *data = dev->data;
    if (data->runtime_snipe_cpi == 0) {
        return CONFIG_PMW3610_MCK_SNIPE_CPI;
    }
    return (int)data->runtime_snipe_cpi;
}

int zmk_pmw3610_scroll_tick_change(int amount) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        LOG_ERR("PMW3610 device not available for scroll tick change");
        return -ENODEV;
    }

    struct pixart_data *data = dev->data;

    /* Initialise from Kconfig default if not yet set. */
    if (data->runtime_scroll_tick == 0) {
        data->runtime_scroll_tick = CONFIG_PMW3610_MCK_SCROLL_TICK;
    }

    /* Positive amount = increase tick (slower).
     * Negative amount = decrease tick (faster). */
    int32_t target = (int32_t)data->runtime_scroll_tick + amount;

    /* Clamp to a sensible range: 1 (fastest) .. 200 (slowest). */
    if (target < 1) {
        target = 1;
    } else if (target > 200) {
        target = 200;
    }

    if ((uint32_t)target == data->runtime_scroll_tick) {
        LOG_INF("PMW3610 scroll tick unchanged at %d (requested delta %d)", target, amount);
        return 0;
    }

    data->runtime_scroll_tick = (uint32_t)target;
    LOG_INF("PMW3610 scroll tick -> %d (delta %d)", target, amount);

    return 0;
}

int zmk_pmw3610_scroll_tick_get(void) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return -ENODEV;
    }
    struct pixart_data *data = dev->data;
    if (data->runtime_scroll_tick == 0) {
        return CONFIG_PMW3610_MCK_SCROLL_TICK;
    }
    return (int)data->runtime_scroll_tick;
}

/* ---------------------------------------------------------------------------
 *  Automouse layer timeout
 * ------------------------------------------------------------------------- */

int zmk_pmw3610_automouse_timeout_change(int amount) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        LOG_ERR("PMW3610 device not available for automouse timeout change");
        return -ENODEV;
    }

    struct pixart_data *data = dev->data;

    /* Initialise from Kconfig default if not yet set. */
    if (data->runtime_automouse_timeout == 0) {
        data->runtime_automouse_timeout = CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS;
    }

    /* Positive amount = longer timeout (layer stays active longer).
     * Negative amount = shorter timeout. */
    int32_t target = (int32_t)data->runtime_automouse_timeout + amount;

    /* Clamp to the allowed range: 100 (shortest) .. 3000 (longest). */
    if (target < (int32_t)PMW3610_MIN_AUTOMOUSE_TIMEOUT_MS) {
        target = PMW3610_MIN_AUTOMOUSE_TIMEOUT_MS;
    } else if (target > (int32_t)PMW3610_MAX_AUTOMOUSE_TIMEOUT_MS) {
        target = PMW3610_MAX_AUTOMOUSE_TIMEOUT_MS;
    }

    if ((uint32_t)target == data->runtime_automouse_timeout) {
        LOG_INF("PMW3610 automouse timeout unchanged at %d (requested delta %d)", target, amount);
        return 0;
    }

    data->runtime_automouse_timeout = (uint32_t)target;
    LOG_INF("PMW3610 automouse timeout -> %d ms (delta %d)", target, amount);

    return 0;
}

int zmk_pmw3610_automouse_timeout_get(void) {
    const struct device *dev = zmk_pmw3610_get_dev();
    if (dev == NULL) {
        return -ENODEV;
    }
    struct pixart_data *data = dev->data;
    if (data->runtime_automouse_timeout == 0) {
        return CONFIG_PMW3610_MCK_AUTOMOUSE_TIMEOUT_MS;
    }
    return (int)data->runtime_automouse_timeout;
}
