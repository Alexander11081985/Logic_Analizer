#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

#define CHANNEL_COUNT 8U
#define GPIO_FIRST GPIO_NUM_4
#define GPIO_MASK  UINT64_C(0x0000000000000ff0)

static const gpio_num_t s_channel_gpio[CHANNEL_COUNT] = {
    GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7,
    GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11,
};

#define LEGACY_SAMPLE_RATE_HZ 5000000U
#define LEGACY_SAMPLE_COUNT   8192U
#define MAX_RAW_SAMPLES       32768U
#define MAX_EDGE_EVENTS       8192U
#define MAX_EDGE_DURATION_US  5000000U

#define V1_HEADER_SIZE 28U
#define V2_HEADER_SIZE 56U
#define COMMAND_V2_SIZE 36U
#define LEGACY_PAYLOAD_SIZE ((LEGACY_SAMPLE_COUNT + 1U) / 2U)

#define VERSION_V1 1U
#define VERSION_V2 2U
#define ACQ_RAW  1U
#define ACQ_EDGE 2U
#define PACKING_U8    1U
#define PACKING_EDGE8 2U

#define TRIGGER_RISING    0U
#define TRIGGER_FALLING   1U
#define TRIGGER_EITHER    2U
#define TRIGGER_IMMEDIATE 3U
#define TRIGGER_CHANNEL_IMMEDIATE 0xffU

#define V2_FLAG_TRIGGERED (1U << 0)
#define V2_FLAG_TIMEOUT   (1U << 1)
#define V2_FLAG_OVERFLOW  (1U << 2)
#define V2_FLAG_TRUNCATED (1U << 3)

#define COMMAND_REARM 'R'

typedef struct {
    uint32_t request_id;
    uint32_t sample_rate_hz;
    uint32_t sample_count;
    uint32_t duration_us;
    uint32_t trigger_timeout_ms;
    uint8_t acquisition;
    uint8_t trigger_channel;
    uint8_t trigger_edge;
    bool legacy;
} capture_config_t;

typedef struct {
    uint32_t timestamp_ticks;
    uint8_t state;
    uint8_t changed_mask;
    uint16_t reserved;
} edge_event_t;

_Static_assert(sizeof(edge_event_t) == 8U, "edge event wire format must be 8 bytes");
_Static_assert(MAX_RAW_SAMPLES <= 32768U, "raw ISR window must remain bounded");

static DRAM_ATTR uint8_t s_raw_samples[MAX_RAW_SAMPLES];
static DRAM_ATTR edge_event_t s_edge_events[MAX_EDGE_EVENTS];
static DRAM_ATTR uint8_t s_legacy_payload[LEGACY_PAYLOAD_SIZE];
static uint8_t s_header[V2_HEADER_SIZE];
static uint8_t s_command_buffer[COMMAND_V2_SIZE];

static portMUX_TYPE s_capture_mux = portMUX_INITIALIZER_UNLOCKED;
static capture_config_t s_config;
static volatile bool s_armed;
static volatile bool s_capture_busy;
static volatile bool s_capture_ready;
static volatile bool s_triggered;
static volatile uint32_t s_cycles_per_sample;
static volatile uint32_t s_edge_count;
static volatile uint32_t s_lost_events;
static volatile uint32_t s_capture_start_cycle;
static volatile uint32_t s_capture_duration_ticks;
static volatile uint8_t s_initial_state;
static volatile uint8_t s_edge_last_state;
static volatile uint8_t s_result_flags;
static int64_t s_arm_time_us;
static uint32_t s_cpu_hz;
static uint32_t s_capture_number;
static size_t s_command_length;

static inline uint8_t IRAM_ATTR read_analyzer_pins(void)
{
    return (uint8_t)((GPIO.in >> GPIO_FIRST) & 0xffU);
}

static inline bool IRAM_ATTR trigger_matches(uint8_t previous, uint8_t current)
{
    if (s_config.trigger_channel >= CHANNEL_COUNT) {
        return true;
    }
    const uint8_t mask = (uint8_t)(1U << s_config.trigger_channel);
    const bool before = (previous & mask) != 0U;
    const bool now = (current & mask) != 0U;
    if (before == now) {
        return false;
    }
    return s_config.trigger_edge == TRIGGER_EITHER ||
           (s_config.trigger_edge == TRIGGER_RISING && now) ||
           (s_config.trigger_edge == TRIGGER_FALLING && !now);
}

static void IRAM_ATTR capture_raw_from_isr(uint8_t first_state)
{
    s_armed = false;
    s_capture_busy = true;
    s_triggered = true;
    s_initial_state = first_state;
    s_result_flags = V2_FLAG_TRIGGERED;
    portENTER_CRITICAL_ISR(&s_capture_mux);
    uint32_t next_cycle = esp_cpu_get_cycle_count();
    const uint32_t start_cycle = next_cycle;
    const uint32_t step = s_cycles_per_sample;
    const uint32_t count = s_config.sample_count;
    for (uint32_t index = 0; index < count; ++index) {
        s_raw_samples[index] = read_analyzer_pins();
        next_cycle += step;
        while ((int32_t)(esp_cpu_get_cycle_count() - next_cycle) < 0) {
            __asm__ __volatile__("nop");
        }
    }
    s_capture_duration_ticks = esp_cpu_get_cycle_count() - start_cycle;
    portEXIT_CRITICAL_ISR(&s_capture_mux);
    s_capture_busy = false;
    s_capture_ready = true;
}

static void IRAM_ATTR analyzer_gpio_isr(void *argument)
{
    (void)argument;
    if (!s_armed || s_capture_busy || s_capture_ready) {
        return;
    }
    const uint8_t current = read_analyzer_pins();
    const uint8_t previous = s_edge_last_state;
    const uint8_t changed = (uint8_t)(current ^ previous);
    s_edge_last_state = current;

    if (s_config.acquisition == ACQ_RAW) {
        if (trigger_matches(previous, current)) {
            capture_raw_from_isr(current);
        }
        return;
    }

    const uint32_t now = esp_cpu_get_cycle_count();
    if (!s_triggered) {
        if (!trigger_matches(previous, current)) {
            return;
        }
        s_triggered = true;
        s_capture_start_cycle = now;
        s_initial_state = previous;
        s_result_flags = V2_FLAG_TRIGGERED;
    }
    const uint32_t index = s_edge_count;
    if (index < MAX_EDGE_EVENTS) {
        s_edge_events[index].timestamp_ticks = now - s_capture_start_cycle;
        s_edge_events[index].state = current;
        s_edge_events[index].changed_mask = changed;
        s_edge_events[index].reserved = 0U;
        s_edge_count = index + 1U;
    } else {
        ++s_lost_events;
        s_result_flags |= V2_FLAG_OVERFLOW | V2_FLAG_TRUNCATED;
    }
}

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16_le(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *source)
{
    return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
           ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static uint32_t crc32_ieee(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffU;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (0xedb88320U & mask);
        }
    }
    return ~crc;
}

static bool send_bytes(const uint8_t *data, size_t length)
{
    size_t offset = 0U;
    while (offset < length) {
        const int written = usb_serial_jtag_write_bytes(
            data + offset, length - offset, pdMS_TO_TICKS(1000));
        if (written <= 0) {
            return false;
        }
        offset += (size_t)written;
    }
    return true;
}

static bool send_capture_packet(void)
{
    if (!usb_serial_jtag_is_connected()) {
        return false;
    }
    const uint8_t *payload;
    uint32_t payload_size;
    if (s_config.legacy) {
        for (uint32_t index = 0; index < LEGACY_PAYLOAD_SIZE; ++index) {
            s_legacy_payload[index] = (s_raw_samples[index * 2U] & 0x0fU) |
                (uint8_t)((s_raw_samples[index * 2U + 1U] & 0x0fU) << 4);
        }
        payload = s_legacy_payload;
        payload_size = LEGACY_PAYLOAD_SIZE;
        memcpy(s_header, "RX75", 4U);
        s_header[4] = VERSION_V1;
        s_header[5] = 3U;
        put_u16_le(&s_header[6], V1_HEADER_SIZE);
        put_u32_le(&s_header[8], s_capture_number);
        put_u32_le(&s_header[12], LEGACY_SAMPLE_RATE_HZ);
        put_u32_le(&s_header[16], LEGACY_SAMPLE_COUNT);
        put_u32_le(&s_header[20], payload_size);
        put_u32_le(&s_header[24], crc32_ieee(payload, payload_size));
        if (!send_bytes(s_header, V1_HEADER_SIZE) || !send_bytes(payload, payload_size)) {
            return false;
        }
    } else {
        const uint32_t item_count = (s_config.acquisition == ACQ_RAW) ?
            ((s_result_flags & V2_FLAG_TIMEOUT) ? 0U : s_config.sample_count) : s_edge_count;
        if (s_config.acquisition == ACQ_RAW) {
            payload = s_raw_samples;
            payload_size = item_count;
        } else {
            payload = (const uint8_t *)s_edge_events;
            payload_size = item_count * (uint32_t)sizeof(edge_event_t);
        }
        memset(s_header, 0, V2_HEADER_SIZE);
        memcpy(s_header, "RX75", 4U);
        s_header[4] = VERSION_V2;
        s_header[5] = s_result_flags;
        put_u16_le(&s_header[6], V2_HEADER_SIZE);
        put_u32_le(&s_header[8], s_capture_number);
        put_u32_le(&s_header[12], s_config.request_id);
        put_u32_le(&s_header[16], s_cpu_hz);
        s_header[20] = s_config.acquisition;
        s_header[21] = CHANNEL_COUNT;
        s_header[22] = (s_config.acquisition == ACQ_RAW) ? PACKING_U8 : PACKING_EDGE8;
        s_header[23] = s_config.trigger_channel;
        s_header[24] = s_config.trigger_edge;
        s_header[25] = s_initial_state;
        put_u16_le(&s_header[26], 0U);
        put_u32_le(&s_header[28], (s_config.acquisition == ACQ_RAW) ? s_config.sample_rate_hz : 0U);
        put_u32_le(&s_header[32], item_count);
        put_u32_le(&s_header[36], s_capture_duration_ticks);
        put_u32_le(&s_header[40], payload_size);
        put_u32_le(&s_header[44], s_lost_events);
        put_u32_le(&s_header[48], s_config.trigger_timeout_ms);
        put_u32_le(&s_header[52], crc32_ieee(payload, payload_size));
        if (!send_bytes(s_header, V2_HEADER_SIZE) || !send_bytes(payload, payload_size)) {
            return false;
        }
    }
    return usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(2000)) == ESP_OK;
}

static void disable_all_interrupts(void)
{
    for (size_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        (void)gpio_intr_disable(s_channel_gpio[channel]);
    }
}

static void arm_capture(const capture_config_t *configuration)
{
    if (s_capture_busy) {
        return;
    }
    disable_all_interrupts();
    s_config = *configuration;
    s_capture_ready = false;
    s_triggered = false;
    s_edge_count = 0U;
    s_lost_events = 0U;
    s_capture_duration_ticks = 0U;
    s_result_flags = 0U;
    s_initial_state = read_analyzer_pins();
    s_edge_last_state = s_initial_state;
    s_arm_time_us = esp_timer_get_time();
    s_cycles_per_sample = (s_config.acquisition == ACQ_RAW) ?
        s_cpu_hz / s_config.sample_rate_hz : 0U;
    s_armed = true;

    if (s_config.trigger_edge == TRIGGER_IMMEDIATE ||
        s_config.trigger_channel == TRIGGER_CHANNEL_IMMEDIATE) {
        if (s_config.acquisition == ACQ_RAW) {
            capture_raw_from_isr(s_initial_state);
        } else {
            s_triggered = true;
            s_result_flags = V2_FLAG_TRIGGERED;
            s_capture_start_cycle = esp_cpu_get_cycle_count();
            s_edge_events[0] = (edge_event_t){0U, s_initial_state, 0U, 0U};
            s_edge_count = 1U;
        }
        return;
    }

    if (s_config.acquisition == ACQ_RAW) {
        const gpio_num_t pin = s_channel_gpio[s_config.trigger_channel];
        /*
         * Keep the remembered trigger level synchronized even when the input
         * starts at the target level while the external device is unpowered.
         * Example: arm for CS falling while CS is already LOW.  If only the
         * negedge interrupt is enabled, the intervening rise is never seen,
         * so trigger_matches() later compares LOW with LOW and rejects the
         * real falling edge.  ANYEDGE updates s_edge_last_state on the
         * opposite edge; trigger_matches() still starts sampling only on the
         * edge requested by the host.
         */
        ESP_ERROR_CHECK(gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE));
        ESP_ERROR_CHECK(gpio_intr_enable(pin));
    } else {
        for (size_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
            ESP_ERROR_CHECK(gpio_set_intr_type(s_channel_gpio[channel], GPIO_INTR_ANYEDGE));
            ESP_ERROR_CHECK(gpio_intr_enable(s_channel_gpio[channel]));
        }
    }
}

static void arm_legacy(void)
{
    const capture_config_t legacy = {
        .request_id = 0U,
        .sample_rate_hz = LEGACY_SAMPLE_RATE_HZ,
        .sample_count = LEGACY_SAMPLE_COUNT,
        .duration_us = 0U,
        .trigger_timeout_ms = 0U,
        .acquisition = ACQ_RAW,
        .trigger_channel = 0U,
        .trigger_edge = TRIGGER_RISING,
        .legacy = true,
    };
    arm_capture(&legacy);
}

static bool config_is_valid(const capture_config_t *config)
{
    const bool trigger_valid = (config->trigger_channel < CHANNEL_COUNT ||
                                config->trigger_channel == TRIGGER_CHANNEL_IMMEDIATE) &&
                               config->trigger_edge <= TRIGGER_IMMEDIATE;
    if (!trigger_valid || config->trigger_timeout_ms > 60000U) {
        return false;
    }
    if (config->acquisition == ACQ_RAW) {
        return config->sample_rate_hz >= 100000U &&
               config->sample_rate_hz <= 5000000U &&
               (s_cpu_hz % config->sample_rate_hz) == 0U &&
               config->sample_count >= 256U && config->sample_count <= MAX_RAW_SAMPLES &&
               config->duration_us == 0U;
    }
    return config->acquisition == ACQ_EDGE && config->sample_rate_hz == 0U &&
           config->sample_count == 0U && config->duration_us >= 1000U &&
           config->duration_us <= MAX_EDGE_DURATION_US;
}

static void process_v2_command(void)
{
    const uint8_t *command = s_command_buffer;
    if (memcmp(command, "LAC2", 4U) != 0 || command[4] != VERSION_V2 ||
        command[5] != 1U || get_u16_le(&command[6]) != COMMAND_V2_SIZE ||
        crc32_ieee(command, COMMAND_V2_SIZE - 4U) != get_u32_le(&command[32])) {
        return;
    }
    const capture_config_t config = {
        .request_id = get_u32_le(&command[8]),
        .sample_rate_hz = get_u32_le(&command[16]),
        .sample_count = get_u32_le(&command[20]),
        .duration_us = get_u32_le(&command[24]),
        .trigger_timeout_ms = get_u32_le(&command[28]),
        .acquisition = command[12],
        .trigger_channel = command[13],
        .trigger_edge = command[14],
        .legacy = false,
    };
    if (config_is_valid(&config)) {
        arm_capture(&config);
    }
}

static void feed_command_byte(uint8_t byte)
{
    if (s_command_length == 0U) {
        if (byte == (uint8_t)COMMAND_REARM) {
            arm_legacy();
        } else if (byte == (uint8_t)'L') {
            s_command_buffer[0] = byte;
            s_command_length = 1U;
        }
        return;
    }
    s_command_buffer[s_command_length++] = byte;
    if (s_command_length <= 4U &&
        s_command_buffer[s_command_length - 1U] != (uint8_t)"LAC2"[s_command_length - 1U]) {
        s_command_length = (byte == (uint8_t)'L') ? 1U : 0U;
        if (s_command_length == 1U) s_command_buffer[0] = byte;
        return;
    }
    if (s_command_length == COMMAND_V2_SIZE) {
        process_v2_command();
        s_command_length = 0U;
    }
}

static void service_capture_deadlines(void)
{
    if (!s_armed || s_capture_ready || s_capture_busy || s_config.legacy) {
        return;
    }
    const int64_t elapsed_us = esp_timer_get_time() - s_arm_time_us;
    if (!s_triggered && s_config.trigger_timeout_ms > 0U &&
        elapsed_us >= (int64_t)s_config.trigger_timeout_ms * 1000) {
        disable_all_interrupts();
        s_armed = false;
        s_result_flags = V2_FLAG_TIMEOUT;
        s_capture_duration_ticks = 0U;
        s_edge_count = 0U;
        s_capture_ready = true;
        return;
    }
    if (s_config.acquisition == ACQ_EDGE && s_triggered) {
        const uint32_t elapsed_ticks = esp_cpu_get_cycle_count() - s_capture_start_cycle;
        const uint32_t requested_ticks = (uint32_t)(((uint64_t)s_config.duration_us * s_cpu_hz) / 1000000U);
        if (elapsed_ticks >= requested_ticks) {
            disable_all_interrupts();
            s_armed = false;
            s_capture_duration_ticks = elapsed_ticks;
            s_capture_ready = true;
        }
    }
}

static void configure_gpio(void)
{
    const gpio_config_t inputs = {
        .pin_bit_mask = GPIO_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    for (size_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_channel_gpio[channel], analyzer_gpio_isr, NULL));
    }
}

static void configure_usb(void)
{
    usb_serial_jtag_driver_config_t configuration = {
        .tx_buffer_size = 8192,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&configuration));
    usb_serial_jtag_vfs_use_driver();
}

void app_main(void)
{
    s_cpu_hz = esp_rom_get_cpu_ticks_per_us() * 1000000U;
    configure_usb();
    configure_gpio();
    arm_legacy();

    while (true) {
        service_capture_deadlines();
        if (s_capture_ready) {
            s_capture_ready = false;
            ++s_capture_number;
            (void)send_capture_packet();
        }
        uint8_t byte;
        while (usb_serial_jtag_read_bytes(&byte, 1U, 0U) == 1) {
            feed_command_byte(byte);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
