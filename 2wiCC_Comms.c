#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "ws2812.pio.h"

// UART pins + settings
#define UART_PORT uart0
#define UART_IRQ_ID UART0_IRQ
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUDRATE 460800u

// ring‐buffer size
#define RING_BUFFER_SIZE (1 << 10)

// to change the LED feedback
uint16_t data_feedback_1 = 0, data_feedback_2 = 0;

typedef struct
{
	uint8_t buf[RING_BUFFER_SIZE];
	uint32_t head, tail;
	mutex_t lock;
} ring_buffer_t;

// ring buffer
static ring_buffer_t uart_to_usb_rb;
static ring_buffer_t usb_to_uart_rb;

static inline void rb_init(ring_buffer_t *rb)
{
	rb->head = rb->tail = 0;
	mutex_init(&rb->lock);
}

static inline bool rb_put(ring_buffer_t *rb, uint8_t c)
{
	bool ok = false;
	mutex_enter_blocking(&rb->lock);
	uint32_t next = (rb->head + 1) % RING_BUFFER_SIZE;
	if (next != rb->tail)
	{
		rb->buf[rb->head] = c;
		rb->head = next;
		ok = true;
	}
	mutex_exit(&rb->lock);
	return ok;
}

static inline bool rb_get(ring_buffer_t *rb, uint8_t *c)
{
	bool ok = false;
	mutex_enter_blocking(&rb->lock);
	if (rb->head != rb->tail)
	{
		*c = rb->buf[rb->tail];
		rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
		ok = true;
	}
	mutex_exit(&rb->lock);
	return ok;
}

// NeoPixel helpers
#define IS_RGBW false
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
}
static inline void debug_pixel(uint32_t grb)
{
	pio_sm_put_blocking(pio0, 0, grb << 8);
}

// UART IRQ handler captures bytes into uart_to_usb_rb
static void uart_irq_handler(void)
{
	while (uart_is_readable(UART_PORT))
	{
		uint8_t c = uart_getc(UART_PORT);
		rb_put(&uart_to_usb_rb, c);
	}
}

// Core 1: USB + bridge logic
void core1_main()
{

	tusb_init();

	while (1)
	{
		tud_task();

		// ---- Check for data USB to UART ----
		if (tud_cdc_connected())
		{
			uint8_t buf[64];
			uint32_t cnt = tud_cdc_read(buf, sizeof(buf));
			for (uint32_t i = 0; i < cnt; i++)
			{
				rb_put(&usb_to_uart_rb, buf[i]);
			}
		}

		// drain USB to UART ring buffer into UART when writable
		{
			uint8_t uc;
			bool did_send = false;
			while (uart_is_writable(UART_PORT) && rb_get(&usb_to_uart_rb, &uc))
			{
				uart_putc_raw(UART_PORT, uc);
				did_send = true;
			}
			data_feedback_1 += did_send;
		}

		// ---- UART to USB ----
		{
			uint8_t c;
			bool newdata = false;
			while (rb_get(&uart_to_usb_rb, &c))
			{
				tud_cdc_write_char(c);
				newdata = true;
			}
			data_feedback_2 += newdata;
			tud_cdc_write_flush();
		}

		sleep_us(10);
	}
}

// Heartbeat
#define HB_STEPS 32
static const uint8_t heartbeat[HB_STEPS] = {
	2, 0, 0, 0, 0, 0, 0, 0, 16, 2, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main()
{
	// ---- PIO/NeoPixel init ----
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &ws2812_program);
	ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
	debug_pixel(urgb_u32(0, 0, 0));

	// ---- UART + IRQ init ----
	rb_init(&uart_to_usb_rb);
	rb_init(&usb_to_uart_rb); // <-- initialize new buffer

	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_init(UART_PORT, BAUDRATE);
	uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
	uart_set_hw_flow(UART_PORT, false, false);
	uart_set_fifo_enabled(UART_PORT, false);

	irq_set_exclusive_handler(UART_IRQ_ID, uart_irq_handler);
	irq_set_enabled(UART_IRQ_ID, true);
	uart_set_irq_enables(UART_PORT, true, false);

	// ---- USB init & launch core 1 ----
	usbd_serial_init();
	multicore_launch_core1(core1_main);

	// Core 0: status LED “heartbeat” + rx/tx indicators
	uint8_t conn_feedback = 0;
	uint16_t last_feedback_1 = 0, last_feedback_2 = 0;
	uint8_t rx_indicator = 0, tx_indicator = 0;
	uint8_t hb_idx = 0;
	uint8_t data_check_delay = 10;

	while (1)
	{
		if (--data_check_delay == 0) {
			data_check_delay = 2;
			// Feedback that data is coming in from USB
			if (data_feedback_1 != last_feedback_1)
			{
				last_feedback_1 = data_feedback_1;
				rx_indicator = 1;
			}
			else
			{
				rx_indicator = 0;
			}
			// Feedback that SwiCC is sending data to PC
			if (data_feedback_2 != last_feedback_2)
			{
				last_feedback_2 = data_feedback_2;
				tx_indicator = 1;
			}
			else
			{
				tx_indicator = 0;
			}
		}

		// Look up heartbeat brightness, but only if USB is connected
		uint8_t mount_ok = (tud_mounted() && tud_ready()) ? 1 : 0;
		uint8_t g = heartbeat[hb_idx] * mount_ok;
		hb_idx = (hb_idx + 1) % HB_STEPS;

		uint8_t r = tx_indicator << 2;
		uint8_t b = rx_indicator << 3;
		debug_pixel(urgb_u32(r, g, b));

		sleep_ms(30);
	}
	return 0;
}