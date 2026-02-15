/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "FreeRTOS.h"
#include "autobaud.h"
#include "task.h"
#include "tusb.h"
#include <pico/stdlib.h>


#include "probe_config.h"

TaskHandle_t uart_taskhandle;
TickType_t last_wake, interval = 100;
volatile TickType_t break_expiry;
volatile bool timed_break;

/* Max 1 FIFO worth of data */
// static uint8_t tx_buf[32];
// static uint8_t rx_buf[32];
static uint8_t tx_buf[CDC_UARTS][CFG_TUD_CDC_TX_BUFSIZE];
static uint8_t rx_buf[CDC_UARTS][CFG_TUD_CDC_RX_BUFSIZE];
static int was_connected[CDC_UARTS];

// Actually s^-1 so 25ms
#define DEBOUNCE_MS 40
static uint debounce_ticks = 5;

#ifdef PROBE_UART_TX_LED
static volatile uint tx_led_debounce;
#endif

#ifdef PROBE_UART_RX_LED
static uint rx_led_debounce;
#endif

static BaudInfo_t baud_info;

void cdc_uart_init(void) {
  gpio_set_function(PROBE_UART_TX, GPIO_FUNC_UART);
  gpio_set_function(PROBE_UART_RX, GPIO_FUNC_UART);
  gpio_set_pulls(PROBE_UART_TX, 1, 0);
  gpio_set_pulls(PROBE_UART_RX, 1, 0);
  uart_init(PROBE_UART_INTERFACE, PROBE_UART_BAUDRATE);

#if CDC_UARTS == 2
  gpio_set_function(PROBE_EXTRA_UART_TX, GPIO_FUNC_UART);
  gpio_set_function(PROBE_EXTRA_UART_RX, GPIO_FUNC_UART);
  gpio_set_pulls(PROBE_EXTRA_UART_TX, 1, 0);
  gpio_set_pulls(PROBE_EXTRA_UART_RX, 1, 0);
  uart_init(PROBE_EXTRA_UART_INTERFACE, PROBE_EXTRA_UART_BAUDRATE);
#endif
  for (int n = 0; n < CDC_UARTS; n++) {
    was_connected[n] = 0;
  }

#ifdef PROBE_UART_TX_LED
  tx_led_debounce = 0;
  gpio_init(PROBE_UART_TX_LED);
  gpio_set_dir(PROBE_UART_TX_LED, GPIO_OUT);
#endif
#ifdef PROBE_UART_RX_LED
  rx_led_debounce = 0;
  gpio_init(PROBE_UART_RX_LED);
  gpio_set_dir(PROBE_UART_RX_LED, GPIO_OUT);
#endif

#ifdef PROBE_UART_HWFC
  /* HWFC implies that hardware flow control is implemented and the
   * UART operates in "full-duplex" mode (See USB CDC PSTN120 6.3.12).
   * Default to pulling in the active direction, so an unconnected CTS
   * behaves the same as if CTS were not enabled. */
  gpio_set_pulls(PROBE_UART_CTS, 0, 1);
  gpio_set_function(PROBE_UART_RTS, GPIO_FUNC_UART);
  gpio_set_function(PROBE_UART_CTS, GPIO_FUNC_UART);
  uart_set_hw_flow(PROBE_UART_INTERFACE, true, true);
#else
#ifdef PROBE_UART_RTS
  gpio_init(PROBE_UART_RTS);
  gpio_set_dir(PROBE_UART_RTS, GPIO_OUT);
  gpio_put(PROBE_UART_RTS, 1);
#endif
#endif

#ifdef PROBE_UART_DTR
  gpio_init(PROBE_UART_DTR);
  gpio_set_dir(PROBE_UART_DTR, GPIO_OUT);
  gpio_put(PROBE_UART_DTR, 1);
#endif
}

bool cdc_task(uint8_t tty) {
  // static int was_connected = 0;
  static uint cdc_tx_oe = 0;
  uint rx_len = 0;
  bool keep_alive = false;

  uart_inst_t *uart_ptr = PROBE_UART_INTERFACE;
#if CDC_UARTS == 2
  if (tty) {
    uart_ptr = PROBE_EXTRA_UART_INTERFACE;
  }
#endif

  // Consume uart fifo regardless even if not connected
  // while(uart_is_readable(PROBE_UART_INTERFACE) && (rx_len < sizeof(rx_buf)))
  // {
  //     rx_buf[rx_len++] = uart_getc(PROBE_UART_INTERFACE);
  // }
  while (uart_is_readable(uart_ptr) && (rx_len < sizeof(rx_buf[tty]))) {
    rx_buf[tty][rx_len++] = uart_getc(uart_ptr);
  }

  if (tud_cdc_connected()) {
    //     was_connected = 1;
    // if (tud_cdc_n_connected(tty)) {
    was_connected[tty] = 1;
    int written = 0;
    /* Implicit overflow if we don't write all the bytes to the host.
     * Also throw away bytes if we can't write... */
    if (rx_len) {
      if (!tty) {
#ifdef PROBE_UART_RX_LED
        // gpio_put(PROBE_UART_RX_LED, 1);
        // rx_led_debounce = debounce_ticks;
        gpio_put(PROBE_UART_RX_LED, 1);
        rx_led_debounce = debounce_ticks;
#endif
      }
      written = MIN(tud_cdc_write_available(), rx_len);

      written = MIN(tud_cdc_n_write_available(tty), rx_len);
      if (rx_len > written)
        cdc_tx_oe++;

      if (written > 0) {
        // tud_cdc_write(rx_buf, written);
        // tud_cdc_write_flush();
        tud_cdc_n_write(tty, rx_buf[tty], written);
        tud_cdc_n_write_flush(tty);
      }
    } else {
      if (!tty) {
#ifdef PROBE_UART_RX_LED
        // if (rx_led_debounce)
        //   rx_led_debounce--;
        // else
        //   gpio_put(PROBE_UART_RX_LED, 0);
        if (rx_led_debounce)
          rx_led_debounce--;
        else
          gpio_put(PROBE_UART_RX_LED, 0);
#endif
      }
    }

    /* Reading from a firehose and writing to a FIFO. */
    // size_t watermark = MIN(tud_cdc_available(), sizeof(tx_buf));
    size_t watermark = MIN(tud_cdc_n_available(tty), sizeof(tx_buf[tty]));
    if (watermark > 0) {
      size_t tx_len;
      if (!tty) {
#ifdef PROBE_UART_TX_LED
        gpio_put(PROBE_UART_TX_LED, 1);
        tx_led_debounce = debounce_ticks;
#endif
      }
      /* Batch up to half a FIFO of data - don't clog up on RX */
      watermark = MIN(watermark, 16);
      // tx_len = tud_cdc_read(tx_buf, watermark);
      // uart_write_blocking(PROBE_UART_INTERFACE, tx_buf, tx_len);
      tx_len = tud_cdc_n_read(tty, tx_buf[tty], watermark);
      uart_write_blocking(uart_ptr, tx_buf[tty], tx_len);
    } else {
      if (!tty) {
#ifdef PROBE_UART_TX_LED
        if (tx_led_debounce)
          tx_led_debounce--;
        else
          gpio_put(PROBE_UART_TX_LED, 0);
#endif
      }
    }
    /* Pending break handling */
    if (timed_break) {
      if (((int)break_expiry - (int)xTaskGetTickCount()) < 0) {
        timed_break = false;
        uart_set_break(PROBE_UART_INTERFACE, false);
#ifdef PROBE_UART_TX_LED
        tx_led_debounce = 0;
#endif
      } else {
        keep_alive = true;
      }
    }
  } else if (was_connected[tty]) {
    // tud_cdc_write_clear();
    tud_cdc_n_write_clear(tty);
    uart_set_break(PROBE_UART_INTERFACE, false);
    timed_break = false;
    was_connected[tty] = 0;
#ifdef PROBE_UART_TX_LED
    tx_led_debounce = 0;
#endif
    cdc_tx_oe = 0;
  }
  return keep_alive;
}

void cdc_uart_set_baudrate(uint32_t baudrate, uint8_t tty) {
  /* Set the tick thread interval to the amount of time it takes to
   * fill up half a FIFO. Millis is too coarse for integer divide.
   */
  uint32_t micros = (1000 * 1000 * 16 * 10) / MAX(baudrate, 1);
  interval = MAX(1, micros / ((1000 * 1000) / configTICK_RATE_HZ));
  debounce_ticks = MAX(1, configTICK_RATE_HZ / (interval * DEBOUNCE_MS));
  probe_info("New baud rate %ld micros %ld interval %lu\n", baudrate, micros,
             interval);
  // uart_deinit(PROBE_UART_INTERFACE);
  // tud_cdc_write_clear();
  // tud_cdc_read_flush();

  // uart_init(PROBE_UART_INTERFACE, baudrate);
  uart_inst_t *uart_ptr = PROBE_UART_INTERFACE;
#if CDC_UARTS == 2
  if (tty) {
    uart_ptr = PROBE_EXTRA_UART_INTERFACE;
  }
#endif

  uart_deinit(uart_ptr);
  tud_cdc_n_write_clear(tty);
  tud_cdc_n_read_flush(tty);
  uart_init(uart_ptr, baudrate);
}

void cdc_tasks(void) {
  for (int tty = 0; tty < CDC_UARTS; tty++) {
    cdc_task(tty);
  }
}

void cdc_thread(void *ptr) {
  BaseType_t delayed;
  last_wake = xTaskGetTickCount();
  bool keep_alive;
  /* Threaded with a polling interval that scales according to linerate */
  while (1) {
    for (int tty = 0; tty < CDC_UARTS; tty++) {
      keep_alive = cdc_task(tty);
      if (!keep_alive) {
        delayed = xTaskDelayUntil(&last_wake, interval);
        if (delayed == pdFALSE)
          last_wake = xTaskGetTickCount();
        if (autobaud_running) {
          // Receive baud information from autobaud thread
          if (xQueueReceive(baudQueue, &baud_info, 0) == pdTRUE) {
            cdc_uart_set_baudrate(baud_info.baud, tty);
            // Assume 8N1
            uart_set_format(PROBE_UART_INTERFACE, 8, 1, UART_PARITY_NONE);
          }
        }
      }
    }
  }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *line_coding) {
  uint8_t tty = itf;
  if (line_coding->bit_rate == MAGIC_BAUD) {
    if (!autobaud_running)
      autobaud_start();
    return;
  } else if (autobaud_running) {
    autobaud_wait_stop();
  }
  uart_parity_t parity;
  uint data_bits, stop_bits;

  /* Modifying state, so park the thread before changing it. */
  if (tud_cdc_connected())
    vTaskSuspend(uart_taskhandle);

  cdc_uart_set_baudrate(line_coding->bit_rate, tty);

  switch (line_coding->parity) {
  case CDC_LINE_CODING_PARITY_ODD:
    parity = UART_PARITY_ODD;
    break;
  case CDC_LINE_CODING_PARITY_EVEN:
    parity = UART_PARITY_EVEN;
    break;
  default:
    probe_info("invalid parity setting %u\n", line_coding->parity);
    /* fallthrough */
  case CDC_LINE_CODING_PARITY_NONE:
    parity = UART_PARITY_NONE;
    break;
  }

  switch (line_coding->data_bits) {
  case 5:
  case 6:
  case 7:
  case 8:
    data_bits = line_coding->data_bits;
    break;
  default:
    probe_info("invalid data bits setting: %u\n", line_coding->data_bits);
    data_bits = 8;
    break;
  }

  /* The PL011 only supports 1 or 2 stop bits. 1.5 stop bits is translated to 2,
   * which is safer than the alternative. */
  switch (line_coding->stop_bits) {
  case CDC_LINE_CONDING_STOP_BITS_1_5:
  case CDC_LINE_CONDING_STOP_BITS_2:
    stop_bits = 2;
    break;
  default:
    probe_info("invalid stop bits setting: %u\n", line_coding->stop_bits);
    /* fallthrough */
  case CDC_LINE_CONDING_STOP_BITS_1:
    stop_bits = 1;
    break;
  }

  uart_set_format(PROBE_UART_INTERFACE, data_bits, stop_bits, parity);
  /* Windows likes to arbitrarily set/get line coding after dtr/rts changes, so
   * don't resume if we shouldn't */
  if (tud_cdc_connected())
    vTaskResume(uart_taskhandle);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  uint8_t tty = itf;
#ifdef PROBE_UART_RTS
  gpio_put(PROBE_UART_RTS, !rts);
#endif
#ifdef PROBE_UART_DTR
  gpio_put(PROBE_UART_DTR, !dtr);
#endif

  /* CDC drivers use linestate as a bodge to activate/deactivate the interface.
   * Resume our UART polling on activate, stop on deactivate */
  if (!dtr) {
    vTaskSuspend(uart_taskhandle);
    if (!tty) {
#ifdef PROBE_UART_RX_LED
      gpio_put(PROBE_UART_RX_LED, 0);
      rx_led_debounce = 0;
#endif
#ifdef PROBE_UART_TX_LED
      gpio_put(PROBE_UART_TX_LED, 0);
      tx_led_debounce = 0;
#endif
    }
  } else
    vTaskResume(uart_taskhandle);
}

void tud_cdc_send_break_cb(uint8_t itf, uint16_t wValue) {
  switch (wValue) {
  case 0:
    uart_set_break(PROBE_UART_INTERFACE, false);
    timed_break = false;
#ifdef PROBE_UART_TX_LED
    tx_led_debounce = 0;
#endif
    break;
  case 0xffff:
    uart_set_break(PROBE_UART_INTERFACE, true);
    timed_break = false;
#ifdef PROBE_UART_TX_LED
    gpio_put(PROBE_UART_TX_LED, 1);
    tx_led_debounce = 1 << 30;
#endif
    break;
  default:
    uart_set_break(PROBE_UART_INTERFACE, true);
    timed_break = true;
#ifdef PROBE_UART_TX_LED
    gpio_put(PROBE_UART_TX_LED, 1);
    tx_led_debounce = 1 << 30;
#endif
    break_expiry = xTaskGetTickCount() + (wValue * (configTICK_RATE_HZ / 1000));
    break;
  }
}
