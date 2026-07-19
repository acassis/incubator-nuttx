/****************************************************************************
 * drivers/lcd/gc9xxx.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/gc9xxx.h>

#include "gc9xxx.h"

#ifdef CONFIG_LCD_GC9XXX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify that all configuration requirements have been met */

#ifndef CONFIG_LCD_GC9XXX_SPIMODE
#  define CONFIG_LCD_GC9XXX_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_LCD_GC9XXX_FREQUENCY
#  define CONFIG_LCD_GC9XXX_FREQUENCY 1000000
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) ||\
      defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_LANDSCAPE)
#  ifdef CONFIG_LCD_RLANDSCAPE
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RLANDSCAPE)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

/* Display Resolution */

#if !defined(CONFIG_LCD_GC9XXX_XRES)
#  define CONFIG_LCD_GC9XXX_XRES 240
#endif

#if !defined(CONFIG_LCD_GC9XXX_YRES)
#  define CONFIG_LCD_GC9XXX_YRES 320
#endif

#define GC9XXX_LUT_SIZE    CONFIG_LCD_GC9XXX_YRES

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define GC9XXX_XRES       CONFIG_LCD_GC9XXX_YRES
#  define GC9XXX_YRES       CONFIG_LCD_GC9XXX_XRES
#  define GC9XXX_XOFFSET    CONFIG_LCD_GC9XXX_YOFFSET
#  define GC9XXX_YOFFSET    CONFIG_LCD_GC9XXX_XOFFSET
#else
#  define GC9XXX_XRES       CONFIG_LCD_GC9XXX_XRES
#  define GC9XXX_YRES       CONFIG_LCD_GC9XXX_YRES
#  define GC9XXX_XOFFSET    CONFIG_LCD_GC9XXX_XOFFSET
#  define GC9XXX_YOFFSET    CONFIG_LCD_GC9XXX_YOFFSET
#endif

/* Color depth and format */

#ifdef CONFIG_LCD_GC9XXX_BPP
#  if (CONFIG_LCD_GC9XXX_BPP == 12)
#    define GC9XXX_BPP           12
#    define GC9XXX_COLORFMT      FB_FMT_RGB12_444
#    define GC9XXX_BYTESPP       2
#  elif (CONFIG_LCD_GC9XXX_BPP == 16)
#    define GC9XXX_BPP           16
#    define GC9XXX_COLORFMT      FB_FMT_RGB16_565
#    define GC9XXX_BYTESPP       2
#  else
#    define GC9XXX_BPP           16
#    define GC9XXX_COLORFMT      FB_FMT_RGB16_565
#    define GC9XXX_BYTESPP       2
#    warning "Invalid color depth.  Falling back to 16bpp"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_LCD_GC9A01
static const uint8_t g_gc9xxx_init[] =
{
  /* cmd , len,/[value(s)], time delay (ms) */

  GC9XXX_SWRESET, 0, 120,
  GC9XXX_ENIREG2, 0, 0,
  0xeb, 1, 0x14, 0,
  GC9XXX_ENIREG1, 0, 0,
  GC9XXX_ENIREG2, 0, 0,
  0xeb, 1, 0x14, 0,
  0x84, 1, 0x40, 0,
  0x85, 1, 0xff, 0,
  0x86, 1, 0xff, 0,
  0x87, 1, 0xff, 0,
  0x88, 1, 0x0a, 0,
  0x89, 1, 0x21, 0,
  0x8a, 1, 0x00, 0,
  0x8b, 1, 0x80, 0,
  0x8c, 1, 0x01, 0,
  0x8d, 1, 0x01, 0,
  0x8e, 1, 0xff, 0,
  0x8f, 1, 0xff, 0,
  0xb6, 2, 0x00, 0x00, 0,
  0x3a, 1, 0x55, 0,
  0x90, 4, 0x08, 0x08, 0x08, 0x08, 0,
  0xbd, 1, 0x06, 0,
  0xbc, 1, 0x00, 0,
  0xff, 3, 0x60, 0x01, 0x04, 0,
  0xc3, 1, 0x13, 0,
  0xc4, 1, 0x13, 0,
  0xc9, 1, 0x22, 0,
  0xbe, 1, 0x11, 0,
  0xe1, 2, 0x10, 0x0e, 0,
  0xdf, 3, 0x21, 0x0c, 0x02, 0,
  0xf0, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2a, 0,
  0xf1, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6f, 0,
  0xf2, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2a, 0,
  0xf3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6f, 0,
  0xed, 2, 0x1b, 0x0b, 0,
  0xae, 1, 0x77, 0,
  0xcd, 1, 0x63, 0,
  0x70, 9, 0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09,
           0x07, 0x08, 0x03, 0,
  0xe8, 1, 0x34, 0,
  0x62, 12, 0x18, 0x0d, 0x71, 0xed, 0x70, 0x70,
            0x18, 0x0f, 0x71, 0xef, 0x70, 0x70, 0,
  0x63, 12, 0x18, 0x11, 0x71, 0xf1, 0x70, 0x70,
            0x18, 0x13, 0x71, 0xf3, 0x70, 0x70, 0,
  0x64, 7, 0x28, 0x29, 0xf1, 0x01, 0xf1, 0x00,
           0x07, 0,
  0x66, 10, 0x3c, 0x00, 0xcd, 0x67, 0x45, 0x45,
            0x10, 0x00, 0x00, 0x00, 0,
  0x67, 10, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x01,
            0x54, 0x10, 0x32, 0x98, 0,
  0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4e,
           0x00, 0,
  0x98, 2, 0x3e, 0x07, 0,
  GC9XXX_TEON, 0, 120
};
#else
#ifdef CONFIG_LCD_GC9B71
static const uint8_t g_gc9xxx_init[] =
{
  /* cmd , len,/[value(s)], time delay (ms) */

};
#endif

/* This structure describes the state of this driver */

struct gc9xxx_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;  /* SPI device */
  uint8_t bpp;                /* Selected color depth */
  uint8_t power;              /* Current power setting */

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane. This memory will hold one raster line of data.
   * The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.  However,
   * if there are multiple LCD devices, they must each have unique run
   * buffers.
   */

  uint16_t runbuffer[GC9XXX_LUT_SIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Misc. Helpers */

static void gc9xxx_select(FAR struct spi_dev_s *spi, int bits);
static void gc9xxx_deselect(FAR struct spi_dev_s *spi);

static inline void gc9xxx_sendcmd(FAR struct gc9xxx_dev_s *dev, uint8_t cmd);
static void gc9xxx_cmddata(FAR struct gc9xxx_dev_s *dev, uint8_t cmd,
                               const uint8_t *data, int len);
static void gc9xxx_init(FAR struct gc9xxx_dev_s *dev);
static void gc9xxx_sleep(FAR struct gc9xxx_dev_s *dev, bool sleep);
static void gc9xxx_setorientation(FAR struct gc9xxx_dev_s *dev);
static void gc9xxx_display(FAR struct gc9xxx_dev_s *dev, bool on);
static void gc9xxx_setarea(FAR struct gc9xxx_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void gc9xxx_bpp(FAR struct gc9xxx_dev_s *dev, int bpp);
static void gc9xxx_wrram(FAR struct gc9xxx_dev_s *dev,
                         FAR const uint8_t *buff, size_t size , size_t skip,
                         size_t count);
#ifndef CONFIG_LCD_NOGETRUN
static void gc9xxx_rdram(FAR struct gc9xxx_dev_s *dev,
                         FAR uint16_t *buff, size_t size);
#endif
static void gc9xxx_fill(FAR struct gc9xxx_dev_s *dev, uint16_t color);

/* LCD Data Transfer Methods */

static int gc9xxx_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels);
static int gc9xxx_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride);
#ifndef CONFIG_LCD_NOGETRUN
static int gc9xxx_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels);
#endif

/* LCD Configuration */

static int gc9xxx_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int gc9xxx_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int gc9xxx_getpower(FAR struct lcd_dev_s *dev);
static int gc9xxx_setpower(FAR struct lcd_dev_s *dev, int power);
static int gc9xxx_getcontrast(FAR struct lcd_dev_s *dev);
static int gc9xxx_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gc9xxx_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gc9xxx_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi   - Reference to the SPI driver structure
 *   bits  - Number of SPI bits
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void gc9xxx_select(FAR struct spi_dev_s *spi, int bits)
{
  /* Select GC9XXX chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the GC9XXX (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_GC9XXX_SPIMODE);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, CONFIG_LCD_GC9XXX_FREQUENCY);
}

/****************************************************************************
 * Name: gc9xxx_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void gc9xxx_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select GC9XXX chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: gc9xxx_sendcmd
 *
 * Description:
 *   Send a command to the driver.
 *
 ****************************************************************************/

static inline void gc9xxx_sendcmd(FAR struct gc9xxx_dev_s *dev, uint8_t cmd)
{
  gc9xxx_select(dev->spi, 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name: gc9xxx_cmddata
 *
 * Description:
 *   Send a command and a series of data to the driver.
 *
 ****************************************************************************/

static void gc9xxx_cmddata(FAR struct gc9xxx_dev_s *dev, uint8_t cmd,
                                      const uint8_t *data, int len)
{
  gc9xxx_select(dev->spi, 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_SNDBLOCK(dev->spi, data, len);
  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name: gc9xxx_init
 *
 * Description:
 *   Send gc9xxx internal init commands.
 *
 ****************************************************************************/

static void gc9xxx_init(FAR struct gc9xxx_dev_s *dev)
{
  const uint8_t *p = g_gc9xxx_init;
  const uint8_t *end = p + sizeof(g_gc9xxx_init);

  while (p < end)
  {
    uint8_t cmd   = *p++;
    uint8_t len   = *p++;
    uint8_t delay = 0;

    /* If the command doesn't have paramenters */

    if (len == 0)
      {
        gc9xxx_sendcmd(dev, cmd);
      }
    else
      {
        gc9xxx_cmddata(dev, cmd, p, len);
        p += len;
      }

    delay = *p++;

    /* If the command require some delay before continuing */

    if (delay != 0)
      {
        up_mdelay(delay);
      }
  }
}

/****************************************************************************
 * Name: gc9xxx_sleep
 *
 * Description:
 *   Sleep or wake up the driver.
 *
 ****************************************************************************/

static void gc9xxx_sleep(FAR struct gc9xxx_dev_s *dev, bool sleep)
{
  gc9xxx_sendcmd(dev, sleep ? GC9XXX_SLPIN : GC9XXX_SLPOUT);
  up_mdelay(120);
}

/****************************************************************************
 * Name: gc9xxx_display
 *
 * Description:
 *   Turn on or off the display.
 *
 ****************************************************************************/

static void gc9xxx_display(FAR struct gc9xxx_dev_s *dev, bool on)
{
  gc9xxx_sendcmd(dev, on ? GC9XXX_DISPON : GC9XXX_DISPOFF);
  gc9xxx_sendcmd(dev, GC9XXX_INVON);
}

/****************************************************************************
 * Name: gc9xxx_setorientation
 *
 * Description:
 *   Set screen orientation.
 *
 ****************************************************************************/

static void gc9xxx_setorientation(FAR struct gc9xxx_dev_s *dev)
{
  uint8_t reg = GC9XXX_MADCTL_MX;
  gc9xxx_sendcmd(dev, GC9XXX_MADCTL);
  gc9xxx_select(dev->spi, 8);

#if !defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_GC9XXX_BGR)

#  if defined(CONFIG_LCD_RLANDSCAPE)

  reg = GC9XXX_MADCTL_MX | GC9XXX_MADCTL_MY | GC9XXX_MADCTL_MV;

#  elif defined(CONFIG_LCD_LANDSCAPE)

  reg = GC9XXX_MADCTL_MV;

#  elif defined(CONFIG_LCD_RPORTRAIT)

  reg = GC9XXX_MADCTL_MY;

#  endif

#  if defined(CONFIG_LCD_GC9XXX_BGR)

  reg |= GC9XXX_MADCTL_BGR;

#  endif

#endif

  SPI_SEND(dev->spi, reg);
  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name: gc9xxx_setarea
 *
 * Description:
 *   Set the rectangular area for an upcoming read or write from RAM.
 *
 ****************************************************************************/

static void gc9xxx_setarea(FAR struct gc9xxx_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
  /* Set row address */

  gc9xxx_sendcmd(dev, GC9XXX_RASET);
  gc9xxx_select(dev->spi, 8);
  SPI_SEND(dev->spi, (y0 + GC9XXX_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y0 + GC9XXX_YOFFSET) & 0xff);
  SPI_SEND(dev->spi, (y1 + GC9XXX_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y1 + GC9XXX_YOFFSET) & 0xff);
  gc9xxx_deselect(dev->spi);

  /* Set column address */

  gc9xxx_sendcmd(dev, GC9XXX_CASET);
  gc9xxx_select(dev->spi, 8);
  SPI_SEND(dev->spi, (x0 + GC9XXX_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x0 + GC9XXX_XOFFSET) & 0xff);
  SPI_SEND(dev->spi, (x1 + GC9XXX_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x1 + GC9XXX_XOFFSET) & 0xff);
  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name: gc9xxx_bpp
 *
 * Description:
 *   Set the color depth of the device.
 *
 ****************************************************************************/

static void gc9xxx_bpp(FAR struct gc9xxx_dev_s *dev, int bpp)
{
  uint8_t depth;

  /* Don't send any command if the depth hasn't changed. */

  if (dev->bpp != bpp)
    {
      depth = bpp / 2 - 3;
      gc9xxx_sendcmd(dev, GC9XXX_COLMOD);
      gc9xxx_select(dev->spi, 8);
      SPI_SEND(dev->spi, depth);
      gc9xxx_deselect(dev->spi);

      /* Cache the new BPP */

      dev->bpp = bpp;
    }
}

/****************************************************************************
 * Name: gc9xxx_wrram
 *
 * Description:
 *   Write to the driver's RAM. It is possible to write multiples of size
 *   while skipping some values.
 *
 ****************************************************************************/

static void gc9xxx_wrram(FAR struct gc9xxx_dev_s *dev,
                         FAR const uint8_t *buff, size_t size, size_t skip,
                         size_t count)
{
  size_t i;

  gc9xxx_sendcmd(dev, GC9XXX_RAMWR);

  gc9xxx_select(dev->spi, 8);

  for (i = 0; i < count; i++)
    {
      SPI_SNDBLOCK(dev->spi, buff + (i * (size + skip)), size);
    }

  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name: gc9xxx_rdram
 *
 * Description:
 *   Read from the driver's RAM.
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void gc9xxx_rdram(FAR struct gc9xxx_dev_s *dev,
                         FAR uint16_t *buff, size_t size)
{
  gc9xxx_sendcmd(dev, GC9XXX_RAMRD);

  gc9xxx_select(dev->spi, GC9XXX_BYTESPP * 8);
  SPI_RECVBLOCK(dev->spi, buff, size);
  gc9xxx_deselect(dev->spi);
}
#endif

/****************************************************************************
 * Name: gc9xxx_fill
 *
 * Description:
 *   Fill the display with the specified color.
 *
 ****************************************************************************/

static void gc9xxx_fill(FAR struct gc9xxx_dev_s *dev, uint16_t color)
{
  int i;

  gc9xxx_setarea(dev, 0, 0, GC9XXX_XRES - 1, GC9XXX_YRES - 1);

  gc9xxx_sendcmd(dev, GC9XXX_RAMWR);
  gc9xxx_select(dev->spi, GC9XXX_BYTESPP *8);

  for (i = 0; i < GC9XXX_XRES * GC9XXX_YRES; i++)
    {
      SPI_SEND(dev->spi, color);
    }

  gc9xxx_deselect(dev->spi);
}

/****************************************************************************
 * Name:  gc9xxx_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int gc9xxx_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  gc9xxx_setarea(priv, col, row, col + npixels - 1, row);
  gc9xxx_wrram(priv, buffer, npixels, 0, 1);

  return OK;
}

/****************************************************************************
 * Name:  gc9xxx_putarea
 *
 * Description:
 *   This method can be used to write a partial area to the LCD:
 *
 *   dev       - The lcd device
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *   stride    - Length of a line in bytes. This parameter may be necessary
 *               to allow the LCD driver to calculate the offset for partial
 *               writes when the buffer needs to be split for row-by-row
 *               writing.
 *
 ****************************************************************************/

static int gc9xxx_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;
  size_t cols = col_end - col_start + 1;
  size_t rows = row_end - row_start + 1;
  size_t row_size = cols * (priv->bpp >> 3);

  ginfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
         row_start, row_end, col_start, col_end);

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  gc9xxx_setarea(priv, col_start, row_start, col_end, row_end);

  /* If the stride is the same of the row, a single SPI transfer is enough.
   * That is always true for lcddev. For framebuffer, that indicates a full
   * screen or full row update.
   */

  if (stride == row_size)
    {
      /* simpler case, we can just send the whole buffer */

      ginfo("Using full screen/full row mode\n");
      gc9xxx_wrram(priv, buffer, rows * row_size, 0, 1);
    }
  else
    {
      /* We have to go row by row */

      ginfo("Falling-back to row by row mode\n");
      gc9xxx_wrram(priv, buffer, row_size, stride - row_size, rows);
    }

  return OK;
}

/****************************************************************************
 * Name:  gc9xxx_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  dev     - The lcd device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static int gc9xxx_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  gc9xxx_setarea(priv, col, row, col + npixels - 1, row);
  gc9xxx_rdram(priv, dest, npixels);

  return OK;
}
#endif

/****************************************************************************
 * Name:  gc9xxx_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int gc9xxx_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          GC9XXX_COLORFMT, GC9XXX_XRES, GC9XXX_YRES);

  vinfo->fmt     = GC9XXX_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = GC9XXX_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = GC9XXX_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                  /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  gc9xxx_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int gc9xxx_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, GC9XXX_BPP);

  pinfo->putrun = gc9xxx_putrun;                  /* Put a run into LCD memory */
  pinfo->putarea = gc9xxx_putarea;                /* Put an area into LCD */
#ifndef CONFIG_LCD_NOGETRUN
  pinfo->getrun = gc9xxx_getrun;                  /* Get a run from LCD memory */
#endif
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = priv->bpp;                      /* Bits-per-pixel */
  pinfo->dev    = dev;                            /* The lcd device */
  return OK;
}

/****************************************************************************
 * Name:  gc9xxx_getpower
 ****************************************************************************/

static int gc9xxx_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;

  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  gc9xxx_setpower
 ****************************************************************************/

static int gc9xxx_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct gc9xxx_dev_s *priv = (FAR struct gc9xxx_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Turn on the display */

      gc9xxx_display(priv, true);

      /* Save the power */

      priv->power = power;
    }
  else
    {
      /* Turn off the display */

      gc9xxx_display(priv, false);

      /* Save the power */

      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Name:  gc9xxx_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int gc9xxx_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  gc9xxx_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int gc9xxx_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  gc9xxx_initialize
 *
 * Description:
 *   Initialize the GC9XXX video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *gc9xxx_lcdinitialize(FAR struct spi_dev_s *spi)
{
  FAR struct gc9xxx_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = gc9xxx_getvideoinfo;
  priv->dev.getplaneinfo = gc9xxx_getplaneinfo;
  priv->dev.getpower     = gc9xxx_getpower;
  priv->dev.setpower     = gc9xxx_setpower;
  priv->dev.getcontrast  = gc9xxx_getcontrast;
  priv->dev.setcontrast  = gc9xxx_setcontrast;
  priv->spi              = spi;

  /* Init the hardware and clear the display */

  gc9xxx_init(priv);
  gc9xxx_sleep(priv, false);
  gc9xxx_bpp(priv, GC9XXX_BPP);
  gc9xxx_setorientation(priv);
  gc9xxx_display(priv, true);
  gc9xxx_fill(priv, 0xffff);

  return &priv->dev;
}

#endif /* CONFIG_LCD_GC9XXX */
