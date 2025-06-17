#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "ws2812.pio.h"

// ----- Configuration -----
#define UART_PORT           uart0
#define UART_IRQ_ID         UART0_IRQ
#define UART_TX_PIN         0
#define UART_RX_PIN         1

#define DEFAULT_BITRATE     921600u
#define DEFAULT_DATA_BITS   8
#define DEFAULT_PARITY      0
#define DEFAULT_STOP_BITS   1

#define RING_BUFFER_SIZE    (1<<10)

static uint8_t data_feedback = 0;

//--------------------------------------------------------------------
// NeoPixel control
//--------------------------------------------------------------------
#define IS_RGBW false

static inline void debug_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline void feedback_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// ----- Ring buffer for thread-safe transfers -----
typedef struct {
    uint8_t buf[RING_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    mutex_t lock;
} ring_buffer_t;

static inline void rb_init(ring_buffer_t *rb) {
    rb->head = rb->tail = 0;
    mutex_init(&rb->lock);
}

static inline bool rb_put(ring_buffer_t *rb, uint8_t data) {
    bool ok = false;
    mutex_enter_blocking(&rb->lock);
    uint32_t next = (rb->head + 1) % RING_BUFFER_SIZE;
    if (next != rb->tail) {
        rb->buf[rb->head] = data;
        rb->head = next;
        ok = true;
    }
    mutex_exit(&rb->lock);
    return ok;
}

static inline bool rb_get(ring_buffer_t *rb, uint8_t *data) {
    bool ok = false;
    mutex_enter_blocking(&rb->lock);
    if (rb->head != rb->tail) {
        *data = rb->buf[rb->tail];
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        ok = true;
    }
    mutex_exit(&rb->lock);
    return ok;
}

static inline uint32_t rb_count(const ring_buffer_t *rb) {
    return (rb->head + RING_BUFFER_SIZE - rb->tail) % RING_BUFFER_SIZE;
}

// ----- Bridge data -----
typedef struct {
    cdc_line_coding_t usb_cfg;
    cdc_line_coding_t uart_cfg;
    mutex_t cfg_lock;
    ring_buffer_t usb_to_uart;
    ring_buffer_t uart_to_usb;
} bridge_t;

static bridge_t bridge;

// ----- UART IRQ: read incoming UART data to ring buffer -----
static void uart_irq_handler(void) {
    while (uart_is_readable(UART_PORT)) {
        uint8_t c = uart_getc(UART_PORT);
        rb_put(&bridge.uart_to_usb, c);
    }
}

// ----- Core1: USB/TinyUSB processing -----
static void core1_main(void) {    
    uint8_t rcol = 1;

    while (true) {
        tud_task();
        if (tud_cdc_connected()) {
            // read USB->UART
            uint8_t buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));
            for (uint32_t i = 0; i < count; i++) {
                rb_put(&bridge.usb_to_uart, buf[i]);
            }
        }
        // red flashing while data comes in, green and blue connection indicators
        debug_pixel(urgb_u32(data_feedback&0b11, tud_mounted()<<1, tud_ready()<<1));
        sleep_ms(1);
    }
}

// ----- Initialization -----
static void init_bridge(void) {
    // init buffers
    rb_init(&bridge.usb_to_uart);
    rb_init(&bridge.uart_to_usb);
    mutex_init(&bridge.cfg_lock);

    // default configs
    bridge.usb_cfg.bit_rate  = DEFAULT_BITRATE;
    bridge.usb_cfg.data_bits = DEFAULT_DATA_BITS;
    bridge.usb_cfg.parity    = DEFAULT_PARITY;
    bridge.usb_cfg.stop_bits = 0; // 0 is actually 1 in the usb config
    bridge.uart_cfg = bridge.usb_cfg;

    // init UART
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_init(UART_PORT, DEFAULT_BITRATE);
    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, DEFAULT_DATA_BITS,
                    DEFAULT_STOP_BITS, DEFAULT_PARITY);
    uart_set_fifo_enabled(UART_PORT, false);

    // IRQ
    irq_set_exclusive_handler(UART_IRQ_ID, uart_irq_handler);
    irq_set_enabled(UART_IRQ_ID, true);
    uart_set_irq_enables(UART_PORT, true, false);
}

int main(void) {
    // Set up debug neopixel
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
    debug_pixel(urgb_u32(0, 0, 0));

    //stdio_init_all();  // enables USB serial
    init_bridge();
    tusb_init();
    // handle USB with core1
    multicore_launch_core1(core1_main);

    while (true) {
        bool some_data = false;
        // flush USB->UART
        uint8_t c;
        while (rb_get(&bridge.usb_to_uart, &c)) {
            uart_putc_raw(UART_PORT, c);
            some_data = true;
        }

        // flush UART->USB
        while (rb_get(&bridge.uart_to_usb, &c)) {
            tud_cdc_write_char(c);
            some_data = true;
        }
        tud_cdc_write_flush();
        data_feedback += some_data;
    }
    return 0;
}
