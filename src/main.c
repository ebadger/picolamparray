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

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"
#include "lamparrayhiddescriptor.h"

LampArrayColor *_cachedStateWriteTo = NULL;
LampArrayColor *_cachedStateReadFrom = NULL;
LampAttributesResponseReport *_lampAttributesReports = NULL;
LampArrayAttributesReport _lampArrayAttributesReport = {};


//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};


#define ARRAY_COLUMNS 144
#define ARRAY_ROWS 1

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

int led_init(void);
void put_pixel(uint8_t r, uint8_t g, uint8_t b);


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


void core1(void)
{
  led_init();

  while (1)
  {
    // this could be set up as DMA
    for (int i = 0; i < _lampArrayAttributesReport.LampCount; i++)
    {
        put_pixel(
            _cachedStateReadFrom[i].RedChannel, 
            _cachedStateReadFrom[i].GreenChannel, 
            _cachedStateReadFrom[i].BlueChannel);
    }

    sleep_ms(50);
  }
}

/*------------- MAIN -------------*/
int main(void)
{
  set_sys_clock_khz(192000, false);

  GenerateLampArrayAttributes(ARRAY_COLUMNS, ARRAY_ROWS, 10000, 10000);

  tusb_init();
  board_init();

  multicore_launch_core1(core1);

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
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

