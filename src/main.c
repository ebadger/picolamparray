/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"
#include "lamparrayhiddescriptor.h"

#include "ws2812.pio.h"

#define WS2812_PIN    2
#define FREQ          1200000

#define ARRAY_COLUMNS 144
#define ARRAY_ROWS    1
#define ARRAY_SIZE    (ARRAY_COLUMNS * ARRAY_ROWS)

#define X_MARGIN      10000 // micrometers
#define Y_MARGIN      10000 // micrometers

LampArrayColor *_cachedStateWriteTo = NULL;
LampArrayColor *_cachedStateReadFrom = NULL;
LampAttributesResponseReport *_lampAttributesReports = NULL;
LampArrayAttributesReport _lampArrayAttributesReport = {};

int dma_0 = 0;
int sm_0 = 0;

uint32_t _mempixels[ARRAY_SIZE] = {};

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern for onboard LED to relay USB state
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) 
{
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void put_pixel(int pixel, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t pixel_grb = urgb_u32(r,g,b);
    _mempixels[pixel] = pixel_grb << 8u;
}

void startdma()
{
  // Channel Zero (sends color data to PIO VGA machine)
  dma_channel_config c0 = dma_channel_get_default_config(dma_0);  // default configs
  channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);             // 32-bit txfers
  channel_config_set_read_increment(&c0, true);                        // yes read incrementing
  channel_config_set_write_increment(&c0, false);                      // no write incrementing
  channel_config_set_dreq(&c0, DREQ_PIO0_TX0) ;                        // DREQ_PIO0_TX2 pacing (FIFO)

  dma_channel_configure(
      dma_0,                      // Channel to be configured
      &c0,                        // The configuration we just created
      &pio0->txf[sm_0],           // write address (RGB PIO TX FIFO)
      &_mempixels,                // The initial read address (pixel color array)
      ARRAY_SIZE,                 // Number of transfers; in this case each is 4 bytes.
      true                        // start immediately.
  );
}

void stopdma()
{
  hw_clear_bits(&dma_hw->ch[dma_0].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);

  //dma_channel_abort(dma_0);
  dma_hw->abort = (1 << dma_0);
  // Wait for all aborts to complete
  while (dma_hw->abort) 
  { 
    sleep_us(1);
  }

  // Wait for transfer channel to not be busy.
  while (dma_hw->ch[dma_0].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS)
  {
    sleep_us(1);
  }
}

// dynamically generate the array attributes to enable various array configurations
// cols = lamps in x dimension
// rows = lamps in y dimension
// xSpaceInMicroMeters is distance between lamps in x dimension
// ySpaceInMicroMeters is distance between lamps in y dimension
void GenerateLampArrayAttributes(int cols, int rows, int xSpaceInMicroMeters, int ySpaceInMicroMeters)
{
  _lampArrayAttributesReport.LampCount = cols * rows;
  _lampArrayAttributesReport.LampArrayKind = 0x1;
  _lampArrayAttributesReport.BoundingBoxDepth = 0x1;
  _lampArrayAttributesReport.BoundingBoxHeight = rows * ySpaceInMicroMeters;
  _lampArrayAttributesReport.BoundingBoxWidth = cols * xSpaceInMicroMeters;
  _lampArrayAttributesReport.MinUpdateIntervalInMicroseconds = 33000;

  _cachedStateReadFrom = (LampArrayColor *)malloc(sizeof(LampArrayColor) * _lampArrayAttributesReport.LampCount);
  _cachedStateWriteTo = (LampArrayColor *)malloc(sizeof(LampArrayColor) * _lampArrayAttributesReport.LampCount);
  _lampAttributesReports = (LampAttributesResponseReport *)malloc(sizeof(LampAttributesResponseReport) * _lampArrayAttributesReport.LampCount);

  memset(_cachedStateReadFrom, 0, sizeof(LampArrayColor) * _lampArrayAttributesReport.LampCount);
  memset(_cachedStateWriteTo, 0, sizeof(LampArrayColor) * _lampArrayAttributesReport.LampCount);
  memset(_lampAttributesReports, 0, sizeof(LampAttributesResponseReport) * _lampArrayAttributesReport.LampCount);
  
  for (int y = 0; y < rows; y++)
  {
    for(int x = 0; x < cols; x++)
    {
        int index = (y * cols) + x;

        int xpos = 0;

        // assume array starts at top left and proceeds in a zig-zag pattern  
        if ((y + 1) % 2 != 0)
        {
           xpos = x * xSpaceInMicroMeters;
        }
        else
        {
           xpos = (cols - x - 1) * xSpaceInMicroMeters;
        }
        
        LampAttributesResponseReport r = {};

        r.LampId = index;
        r.PositionX = xpos;
        r.PositionY = y * ySpaceInMicroMeters;
        r.PositionZ = 0;
        r.UpdateLatency = 0x4000;
        r.LampPurpose = 0x02;
        r.RedChannelsCount = 0xFF;
        r.GreenChannelsCount = 0xFF;
        r.BlueChannelsCount = 0xFF;
        r.GainChannelsCount = 0xFF;
        r.IsProgrammable = 0x01;
        r.LampKey = 0x00;           // keyboard HID usage mapping

        _lampAttributesReports[index] = r;
    }
  }
}


//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}


// Core1 triggers DMA in a tight loop, with 350us delay to signal reset to LED strip
void core1(void)
{
  stdio_init_all();
  printf("WS2812 Smoke Test, using pin %d", WS2812_PIN);

  PIO pio = pio0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm_0, offset, WS2812_PIN, FREQ);

  pio_sm_set_enabled(pio0, sm_0, true);

  while (1)
  {
      startdma();
      while(dma_hw->ch[dma_0].transfer_count > 0)
      {
        sleep_us(1);
      }      
      stopdma();
      sleep_us(350);
  }
}

// Main handles USB and HID
int main(void)
{
  GenerateLampArrayAttributes(ARRAY_COLUMNS, ARRAY_ROWS, X_MARGIN, Y_MARGIN);

  tusb_init();
  board_init();

  multicore_launch_core1(core1);

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    // populate the memory with grb data for LED strip in the main loop

    for (int i = 0; i < _lampArrayAttributesReport.LampCount; i++)
    {
        float r = _cachedStateReadFrom[i].RedChannel;
        float g = _cachedStateReadFrom[i].GreenChannel;
        float b = _cachedStateReadFrom[i].BlueChannel;
        float gain = (float)_cachedStateReadFrom[i].GainChannel / _lampAttributesReports[i].GainChannelsCount;
        
        if (_cachedStateReadFrom[i].GainChannel <= 2)
        {
          gain = 0.0f;
        }

        put_pixel(
            i,
            (uint8_t)(r*gain),
            (uint8_t)(g*gain),
            (uint8_t)(b*gain));
    }
  }

  return 0;
}

//--------------------------------------------------------------------+
// USB callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

