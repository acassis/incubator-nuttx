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
  gc9xxx_sendcmd(dev, GC9XXX_SWRESET);
  up_mdelay(120);
  gc9xxx_sendcmd(dev, GC9XXX_ENIREG2);
  gc9xxx_cmddata(dev, 0xeb, (const uint8_t *) "\x14", 1);
  gc9xxx_sendcmd(dev, GC9XXX_ENIREG1);
  gc9xxx_sendcmd(dev, GC9XXX_ENIREG2);
  gc9xxx_cmddata(dev, 0xeb, (const uint8_t *) "\x14", 1);
  gc9xxx_cmddata(dev, 0x84, (const uint8_t *) "\x40", 1);
  gc9xxx_cmddata(dev, 0x85, (const uint8_t *) "\xFF", 1);
  gc9xxx_cmddata(dev, 0x86, (const uint8_t *) "\xFF", 1);
  gc9xxx_cmddata(dev, 0x87, (const uint8_t *) "\xFF", 1);
  gc9xxx_cmddata(dev, 0x88, (const uint8_t *) "\x0A", 1);
  gc9xxx_cmddata(dev, 0x89, (const uint8_t *) "\x21", 1);
  gc9xxx_cmddata(dev, 0x8a, (const uint8_t *) "\x00", 1);
  gc9xxx_cmddata(dev, 0x8b, (const uint8_t *) "\x80", 1);
  gc9xxx_cmddata(dev, 0x8c, (const uint8_t *) "\x01", 1);
  gc9xxx_cmddata(dev, 0x8d, (const uint8_t *) "\x01", 1);
  gc9xxx_cmddata(dev, 0x8e, (const uint8_t *) "\xFF", 1);
  gc9xxx_cmddata(dev, 0x8f, (const uint8_t *) "\xFF", 1);
  gc9xxx_cmddata(dev, 0xb6, (const uint8_t *) "\x00\x00", 2);
  gc9xxx_cmddata(dev, 0x3a, (const uint8_t *) "\x55", 1);
  gc9xxx_cmddata(dev, 0x90, (const uint8_t *) "\x08\x08\x08\x08", 4);
  gc9xxx_cmddata(dev, 0xbd, (const uint8_t *) "\x06", 1);
  gc9xxx_cmddata(dev, 0xbc, (const uint8_t *) "\x00", 1);
  gc9xxx_cmddata(dev, 0xff, (const uint8_t *) "\x60\x01\x04", 3);
  gc9xxx_cmddata(dev, 0xc3, (const uint8_t *) "\x13", 1);
  gc9xxx_cmddata(dev, 0xc4, (const uint8_t *) "\x13", 1);
  gc9xxx_cmddata(dev, 0xc9, (const uint8_t *) "\x22", 1);
  gc9xxx_cmddata(dev, 0xbe, (const uint8_t *) "\x11", 1);
  gc9xxx_cmddata(dev, 0xe1, (const uint8_t *) "\x10\x0E", 2);
  gc9xxx_cmddata(dev, 0xdf, (const uint8_t *) "\x21\x0c\x02", 3);
  gc9xxx_cmddata(dev, 0xf0, (const uint8_t *) "\x45\x09\x08\x08\x26\x2A", 6);
  gc9xxx_cmddata(dev, 0xf1, (const uint8_t *) "\x43\x70\x72\x36\x37\x6F", 6);
  gc9xxx_cmddata(dev, 0xf2, (const uint8_t *) "\x45\x09\x08\x08\x26\x2A", 6);
  gc9xxx_cmddata(dev, 0xf3, (const uint8_t *) "\x43\x70\x72\x36\x37\x6F", 6);
  gc9xxx_cmddata(dev, 0xed, (const uint8_t *) "\x1B\x0B", 2);
  gc9xxx_cmddata(dev, 0xae, (const uint8_t *) "\x77", 1);
  gc9xxx_cmddata(dev, 0xcd, (const uint8_t *) "\x63", 1);
  gc9xxx_cmddata(dev, 0x70, (const uint8_t *)
                 "\x07\x07\x04\x0E\x0F\x09\x07\x08\x03", 9);
  gc9xxx_cmddata(dev, 0xe8, (const uint8_t *) "\x34", 1);
  gc9xxx_cmddata(dev, 0x62, (const uint8_t *)
                 "\x18\x0D\x71\xED\x70\x70\x18\x0F\x71\xEF\x70\x70", 12);
  gc9xxx_cmddata(dev, 0x63, (const uint8_t *)
                 "\x18\x11\x71\xF1\x70\x70\x18\x13\x71\xF3\x70\x70", 12);
  gc9xxx_cmddata(dev, 0x64, (const uint8_t *)
                 "\x28\x29\xF1\x01\xF1\x00\x07", 7);
  gc9xxx_cmddata(dev, 0x66, (const uint8_t *)
                 "\x3C\x00\xCD\x67\x45\x45\x10\x00\x00\x00", 10);
  gc9xxx_cmddata(dev, 0x67, (const uint8_t *)
                 "\x00\x3C\x00\x00\x00\x01\x54\x10\x32\x98", 10);
  gc9xxx_cmddata(dev, 0x74, (const uint8_t *)
                 "\x10\x85\x80\x00\x00\x4E\x00", 7);
  gc9xxx_cmddata(dev, 0x98, (const uint8_t *) "\x3e\x07", 2);
  gc9xxx_sendcmd(dev, GC9XXX_TEON);
  up_mdelay(120);
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
